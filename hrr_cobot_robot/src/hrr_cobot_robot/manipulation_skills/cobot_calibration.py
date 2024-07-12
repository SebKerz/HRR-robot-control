#!/usr/bin/env python
"""
Calibration Service as an Action Service => Dummy Tutorial
------------------------------------------------------------

In order to implement the remaining actions-services, this
class simply implements the calibration service of the cobot as an action-service

ROS-parameters
^^^^^^^^^^^^^^^^^^^^

========================= ========================= ===================================================
Parameter                 default value             notes
========================= ========================= ===================================================
~v_max                    0.01                      translation velocity limit in [m/s]
~F_max_post_calib         5.0                       maximum wrench magnitude after calibration
~F_max_noise              2.0                       maximum acceptable noise
~bit_mask_hack            [1, 1, 1, 1, 1, 0]        binary mask for the post-evaluation
~default_joint_velocities [0.5, 0.5, 0.5]           maximum joint-velocity values
~scale_joint_range        [1.0, 0.9, 1.5]           scaling of joint movement, where 1 is approximately
========================= ========================= ===================================================
"""
import datetime
from pathlib import Path

import numpy as np
import spatialmath as sm

import actionlib
import rospy

import hrr_common
from hrr_common.ros_utils.conversions import np2wrench, np2vec3
from hrr_common.ros_utils.helper_handles import get_param
from hrr_common.utils import pose_error
from hrr_controllers import free_space_offset_regression
from hrr_msgs.msg import (
    CalibrateCobotAction,
    CalibrateCobotResult, CalibrateCobotFeedback,
)

from hrr_cobot_robot.manipulation_skills.skill_base import SkillBase

__all__ = ["CalibrationServer"]


class CalibrationServer(SkillBase):
    """Calibration Action"""

    def __init__(self):
        super(CalibrationServer, self).__init__(name="ft_sensor_calibration", cobot=None, observer=None)
        self._calibration_file = ""
        self._calibrated = True
        self.v_max = 0.1
        self._q_calib_offset = [np.deg2rad(121), 0, 0, 0, -0.5, 0] #[0, 0, 0, 0, -0.5, 0] #
        self.F_max_post_calib = 5.0
        self.F_max_noise = 2.0
        self.bit_mask_hack = np.ones(6)
        self.bit_mask_hack[-1] = 0
        self.q_dot_max = 0.5 * np.ones(3)
        self.q_dot_default = 0.5 * np.ones(3)
        self.scale_q = np.r_[1., 0.7, 1.5]

    def init_ros(self, calibrate_action_srv_name="~calibrate"):
        """
        Initialize ROS API -> action service

        Args:
            calibrate_action_srv_name(str): name of action.
        """
        self._as = actionlib.SimpleActionServer(calibrate_action_srv_name, CalibrateCobotAction,
                                                execute_cb=self.execute_skill_cb, auto_start=False)
        self._as.start()

    @classmethod
    def _from_ros(cls, *args, cobot=None, cobot_prefix, calibration_prefix="~", **kwargs):
        out = cls()
        out.init_skill_base(cobot_prefix=cobot_prefix, cobot=cobot)
        try:
            assert out.cobot.FT is not None, "cannot start this class handle without an interface to the F/T-sensor"
            assert out.cobot.has_sns_vel, "cannot calibrate cobot without sensor-tracking velocity control interface"
            out.cobot.set_EE_FT_transform()
        except AssertionError as e:
            rospy.logerr(f"could not create F/T-calibration instance due to {e}")
            return None
        except AttributeError as e:
            rospy.logerr(f"could not create F/T-calibration instance due to {e}")
            return None

        calibration_prefix = hrr_common.ros_utils.fix_prefix(calibration_prefix)
        out.init_ros(
            hrr_common.get_param(f"{cobot_prefix}calibration_action_srv_name", f"{calibration_prefix}calibrate"))
        out.v_max = get_param(f"{calibration_prefix}v_max", out.v_max)
        out.F_max_post_calib = get_param(f"{calibration_prefix}F_max_post_calibration", out.F_max_post_calib)
        out.F_max_noise = get_param(f"{calibration_prefix}F_max_noise", out.F_max_noise)
        out.bit_mask_hack = get_param(f"{calibration_prefix}bit_mask_hack", out.bit_mask_hack)
        out.q_dot_default[:] = get_param(f"{calibration_prefix}default_joint_velocities", out.q_dot_default)
        out.scale_q[:] = get_param(f"{calibration_prefix}scale_joint_range", out.scale_q)
        out._q_calib_offset[:] = get_param(f"{calibration_prefix}calibration_joint_offset", out._q_calib_offset)
        out.observer.set_buffers(out.cobot)
        return out

    def execute_skill_cb(self, goal):
        """
        Actual Skill Execution as an action-service

        Args:
            goal(CalibrateCobotGoal): Action-service goal

        """

        def save_data():
            try:
                obs_data = self.observer.drop(self.cobot)
                if any(len(x) > 0 for x in obs_data.values()):
                    np.save(str(data_file), {'recordings': data, 'observer': obs_data})
                else:
                    np.save(str(data_file), data)
            except PermissionError as e:
                rospy.logerr(f"cannot save data in {data_file} (Permission denied: {e})")
            except FileNotFoundError as e:
                rospy.logerr(f"cannot save data in {data_file} (Path seems incorrect: {e})")

        def get_data_size():
            try:
                return data.wrench.shape[0]
            except (AttributeError, TypeError):
                return 0

        def dead():
            # helper function to check for cancel / error
            return any([not self.action_sever_valid, not self.empty_error_message(err)])

        self._q_calib_offset = [np.deg2rad(121), 0, 0, 0, -0.5, 0] # [0, 0, 0, 0, -0.5, 0]
        self.pre_skill_execution()
        err = ""
        if self.cobot.R_FT_E is None:
            rospy.logwarn("E->FT rotation not set")
            self.cobot.set_EE_FT_transform()
        if np.all(self.cobot.R_FT_E == np.eye(3)):
            self.cancel("E->FT rotation is identity matrix. Check the robot state publisher and joint-state-publisher")
        calibration_file = Path("/tmp", f"calibration_{datetime.datetime.now()}")
        if goal.calibration_file:
            calibration_file = Path(goal.calibration_file)
            if not goal.recalibrate:
                try:
                    self.cobot.FT.load_offset(calibration_file)
                    return self.end_skill(msg=f"loaded previous calibration from {calibration_file}")
                except (FileNotFoundError, AssertionError):
                    rospy.loginfo(f"calibration file {calibration_file} not found. Record new data")

        # run action
        if dead():
            return
        self.publish_feedback(f"start calibration")
        # move to calibration pose
        self.observer.reset()
        if self.action_sever_valid:
            self.publish_feedback("move to calibration pose")
        if not goal.keep_current_pose:

            q_des = self.cobot.q_calib + self._q_calib_offset
            # q_des = np.r_[-1.29851,  0.00112, -1.87362,  0.00495, -0.30392,  1.56607]
            if not self.cobot.legal_joint_config(q_des):
                return self.cancel(msg=f"calibration joint-configuration {q_des} is invalid")
            self.cobot.move_to_joint_pose(q_des)
            if np.linalg.norm(pose_error(self.cobot.FK(q_des), self.cobot.T_B_E_robot)) > 1e-2:
                err += "cannot run calibration -> robot not in calibration pose"
        if dead():
            return
        self.publish_feedback("reached calibration pose")

        # record data
        self.cobot.init_sns_vel()
        if goal.data_file is None:
            data_file = None
            self.publish_feedback(f"start data collection without saving data")
        else:
            data_file = Path(goal.data_file)
            self.publish_feedback(f"start data collection (saving: {data_file})")
        if dead():
            return

        q_dot_des = self.q_dot_default.copy()
        scaling = self.scale_q.copy()
        q_dot_des[0] = min(goal.q4_dot_des, self.q_dot_max[0]) if goal.q4_dot_des > 0.0 else self.q_dot_default[0]
        q_dot_des[1] = min(goal.q5_dot_des, self.q_dot_max[1]) if goal.q5_dot_des > 0.0 else self.q_dot_default[1]
        q_dot_des[2] = min(goal.q6_dot_des, self.q_dot_max[2]) if goal.q6_dot_des > 0.0 else self.q_dot_default[2]
        scaling[0] = goal.scale_q4 if goal.scale_q4 > 0.0 else scaling[0]
        scaling[1] = goal.scale_q5 if goal.scale_q5 > 0.0 else scaling[1]
        scaling[2] = goal.scale_q6 if goal.scale_q6 > 0.0 else scaling[2]
        data = self.cobot.collect_calibration_data(q_dot=q_dot_des, scale_range=scaling)
        # process recorded data and return feedback
        if data_file is not None:
            save_data()
        D = get_data_size()
        if dead():
            return
        self.publish_feedback(f"collected (and optionally saved data) of size {D}", data_size=D)

        # run F/T-regression
        self.cobot.update_tf()
        self.cobot.FT.set_offset(*free_space_offset_regression(data, self.cobot.R_FT_E, plot=False))
        self.cobot.update_tf()

        if np.any(np.abs(self.cobot.FT.noise * self.bit_mask_hack) > self.F_max_noise):
            err += f"recorded sensor-data resulted in a noise above {self.F_max_noise}: {self.cobot.FT.noise}!\n"
        if np.linalg.norm(self.cobot.B_F_msr * self.bit_mask_hack) > self.F_max_post_calib:
            err += f"F/T-data is above {self.F_max_post_calib} after calibration {self.cobot.B_F_msr}!\n"
        if dead():
            return
        self._calibrated = True
        self.publish_feedback(f"regressed sensor parameters from data", data_size=D)
        # save data and end
        self._calibration_file = calibration_file
        self.cobot.FT.set_params()
        self.cobot.FT.save_offset(self._calibration_file)
        self.end_skill(msg=f"finalized calibration: new F/T reading: {self.cobot.B_F_msr}")

    @property
    def result(self) -> CalibrateCobotResult:
        """Generate CalibrateActionResult from code, 
        c.f. :py:meth:`~hrr_cobot_robot.ros_interfaces.sensor_handles.FTData.srv_response`

        Returns:
            CalibrateCobotResult: action-service result
        """
        return CalibrateCobotResult(bias=np2wrench(self.cobot.FT.calibration_data.bias),
                                    noise=np2wrench(self.cobot.FT.calibration_data.noise),
                                    tool_com=np2vec3(self.cobot.FT.calibration_data.FT_com),
                                    tool_grav=np2vec3(self.cobot.FT.calibration_data.B_grav_vec),
                                    calibration_file=str(self._calibration_file))

    def publish_feedback(self, step_msg, data_size=0):
        rospy.loginfo(f"current step: {step_msg}")
        fb = CalibrateCobotFeedback(step=step_msg, tool=self.cobot.tool, data_size=data_size)
        self._as.publish_feedback(fb)
