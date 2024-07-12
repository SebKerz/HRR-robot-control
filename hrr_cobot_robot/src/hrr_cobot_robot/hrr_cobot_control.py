# -*- coding: utf-8 -*-
"""
HRR Cobot Control
-------------------

This class extends the basic hrr-cobot interface manager :py:class:`hrr_cobot_robot.hrr_cobot_control.HrrCobotIf`
by allowing to send control profiles to the robot such as

- Cartesian servoing with force-sensitive stops
- hybrid force-velocity control
- Tool control via the Arduino

Todo:
    * responsive /reactive joint control -> interface to external planner
"""
# inbuilt imports
from typing import Optional

# 3rd party imports
import numpy as np
import quaternion
# ROS imports
import rospy
import spatialmath as sm
from hr_recycler_msgs.msg import CobotState, ToolType
from wsg_50_hw import DsaGripperCommandHandle

import hrr_common
# hrr-cobot-robot imports
from hrr_cobot_robot.hrr_cobot_handle import HrrCobotIf
from hrr_controllers.sensor_handles import free_space_offset_regression
# hrr-cobot imports
from hrr_controllers.utils import ForceControlHandle, VelocityControlHandle, matrix_parser
from comau_msgs.srv import SetIO, SetIORequest


__all__ = ["HrrCobotControl", "JointLimitException", "JointLimitWarning", "UnreachableException"]


class JointLimitWarning(Exception):

    def __init__(self, *a):
        super(JointLimitWarning, self).__init__(*a)


class JointLimitException(Exception):

    def __init__(self, *a):
        super(JointLimitException, self).__init__(*a)


class UnreachableException(Exception):

    def __init__(self, *a):
        super(UnreachableException, self).__init__(*a)


class HrrCobotControl(HrrCobotIf):
    """
    Extended version of the HRR-cobot that allows additional commands, that exceed default capabilities

    This instance has an internal update function that is intended to be called in a loop-style manner.

    By alternating the state attribute the cobot-behavior can then be changed to the required liking.
    """

    _CART_SERVO = "cart_servo"
    _HYBRID_CMD_SNS_VEL = "hybrid_vel_cmd"
    _HYBRID_CMD_SNS_CMPL = "hybrid_sns_cmpl"
    _JOINT_CTRL = "joint_control"

    def __init__(self, *args, **kwargs):
        super(HrrCobotControl, self).__init__(*args, **kwargs)
        self._gripper = None  # type: Optional[DsaGripperCommandHandle]
        self.transl_precision = 5e-4
        self.rot_precision = 1e-3
        self.err_gain = 1.0
        self.ctrl_frame = self.base_link
        self.compensate_joint_limits = False
        self.F_max = 180
        self._T_des = None
        self._state = None
        self._rate = rospy.Rate(100)
        self._pub_state = None
        self._pub_rpm = None
        self._F_ctrl = ForceControlHandle()
        self._v_ctrl = VelocityControlHandle(self.v_max, self.ω_max)
        self.B_F_des = self.B_F_ext
        self._current_control_mode = ""

    @property
    def gripper(self) -> Optional[DsaGripperCommandHandle]:
        if self.tool_id != ToolType.WSG_50_DSA:
            rospy.logwarn(f"accessing gripper, even thou current handle is set to {self.tool}")
        return self._gripper

    @gripper.setter
    def gripper(self, value):
        if value is None:
            rospy.logdebug("disabled gripper-handle")
        elif isinstance(value, DsaGripperCommandHandle):
            rospy.logdebug("added gripper-handle to current cobot instance")
        else:
            rospy.logerr(f"cannot assign gripper handle by type {type(value)}")
        self._gripper = value

    @property
    def T_des(self):
        if self._T_des is None:
            return self.T_B_E_robot
        return self._T_des

    @T_des.setter
    def T_des(self, value):
        if isinstance(value, sm.SE3):
            self._T_des = value
        elif isinstance(value, np.ndarray):
            assert value.shape == (4, 4)
            self._T_des = sm.SE3()
            self._T_des.A[:] = value
        else:
            raise TypeError(f"cannot set desired pose to {value}")

    def init_ros(self,
                 *args,
                 cobot_state_topic_name=None,
                 **kwargs):
        """
        Method to initialize Cobot ROS-API. No parameters are collected automatically.

        Args:
            cobot_state_topic_name (str, optional): topic-name for internal cobot-state-publisher. Defaults to None.
        """
        super().init_ros(*args, **kwargs)
        if cobot_state_topic_name is not None:
            self._pub_state = rospy.Publisher(
                cobot_state_topic_name, CobotState, queue_size=10)

    @classmethod
    def _from_ros(cls, cobot_prefix, **_):
        """
        Initialize class from ROS-parameter server values.

        Args:
            prefix (str) : Sets prefix for the cobot_state_topic name. Defaults to ~.
            **_(dict, optional): optional arguments for parent class

        Returns:
            Result of parent class _from_ros method: fully initialized cobot interface.
        """
        kwargs = dict(cobot_state_topic_name=rospy.get_param(f"{cobot_prefix}cobot_state_topic_name", None))
        out = super()._from_ros(cobot_prefix=cobot_prefix, **kwargs, **_)
        out.ctrl_frame = out.base_link
        return out

    def _reset_set_values(self):
        """reset all set-values in current control handles"""
        self._v_ctrl.reset()
        self._F_ctrl.reset()
        self._T_des = None
        self.B_F_des = self.B_F_ext
        self.ctrl_frame = self.base_link

    def reset(self):
        """Resets the current set values and sets the robot to an ``idle`` state"""
        self._state = None
        self._reset_set_values()

    def emergency_stop(self):
        """Puts robot to halt, and raises an error-state to be noticeable via the :py:meth:`~state` property"""
        srv = rospy.ServiceProxy("/hrr_cobot/set_digital_io", SetIO)
        srv(SetIORequest(pin=11, state=False))
        rospy.sleep(0.2)
        srv(SetIORequest(pin=12, state=False))
        srv(SetIORequest(pin=13, state=False))
        self.stop()
        self._reset_set_values()
        self._state = "FAILURE"

    @property
    def R_B_C(self):
        if self.ctrl_frame != "" and self.ctrl_frame != self.base_link:
            _, q = self._tf.T_A_B(A=self.base_link, B=self.ctrl_frame)
            return quaternion.as_rotation_matrix(q)
        return np.eye(3)

    @property
    def hz(self):
        return 1.0 / self.dt

    @property
    def dt(self):
        return self._rate.sleep_dur.to_sec()

    @hz.setter
    def hz(self, value):
        assert value > 0, "rate needs to eb positive"
        self._rate = rospy.Rate(value)

    @property
    def state(self):
        if self._state == "FAILURE":
            return "FAILURE"
        return self._state

    @state.setter
    def state(self, value):
        if value == self._state:
            return
        elif value is None or value == 0.0 or value == "idle":
            self._state = None
        elif self._state is None:
            self._state = value
        elif value in (self._CART_SERVO, self._HYBRID_CMD_SNS_VEL) and \
                self._state in (self._CART_SERVO, self._HYBRID_CMD_SNS_VEL):
            self._state = value
        elif self.state not in (self._JOINT_CTRL, self._HYBRID_CMD_SNS_CMPL):
            self._state = value
        else:
            raise ValueError(
                f"cannot alter current cobot-state {self.state} to {value}")

    def safety_check(self) -> bool:
        if np.linalg.norm(self.B_F_msr) >= 140:
            rospy.logerr_throttle(0.2,
                                  f"Force threshold exceeded: |F|={np.linalg.norm(self.B_F_msr):.2f} > 140. Stop motion?")
            rospy.logerr_throttle(0.2,f"Raw force {self.FT_F}")
        if np.linalg.norm(self.FT_F) == 0:
            rospy.logerr_throttle(0.2,f"Force is zero. No F/T sensor connected? Stop motion?")
        return True

    def _cartesian_servo_command(self):
        """
        Single update step for a Cartesian servo command using the
        sensor-track velocity control interface.

        Raises:
            JointLimitWarning: in case the robot is within soft-joint limit.
            JointLimitException: refer to :py:meth:`~cartesian_correction_velocity`
        """
        v_err, err = hrr_common.cartesian_servo_set_value(
            self.T_des, self.T_B_E_robot, self.err_gain)
        T_B_E_jacob = self.T_B_E_robot.jacob()
        sns_cmd = T_B_E_jacob @ v_err
        sns_err = T_B_E_jacob @ err
        v_correct = self.cartesian_correction_velocity(gain=1.0)
        if self.legal_joint_config(self.q) and np.linalg.norm(v_correct) != 0.0:
            if not self.legal_joint_config(self.q + self.Jac().T @ sns_err, soft_limits=False):
                if self.compensate_joint_limits:
                    sns_cmd += v_correct
                else:
                    raise JointLimitWarning("cobot is within soft joint limits."
                                            f"stop simple servoing with current error: {err}")
        v_norm = np.linalg.norm(sns_cmd[:3])
        if v_norm > np.sqrt(3) * self.v_max:
            sns_cmd[:3] *= (np.sqrt(3) * self.v_max) / v_norm
        ω_norm = np.linalg.norm(sns_cmd[3:])
        if ω_norm > self.ω_max:
            sns_cmd[3:] *= (np.sqrt(3) * self.ω_max) / ω_norm
        v_sns = sns_cmd[:3]
        ω_sns = sns_cmd[3:]
        v_sns[np.where(np.abs(sns_err[:3]) <= self.transl_precision)] = 0.0
        ω_sns[np.where(np.abs(sns_err[3:]) <= self.rot_precision)] = 0.0
        if (np.abs(v_sns) + np.abs(ω_sns)).sum() == 0.:
            self.stop()
        else:
            self._sns_trk_vel_handle.update_cmd(
                np.r_[np.clip(v_sns, -self.v_max, self.v_max),
                      np.clip(ω_sns, -self.ω_max, self.ω_max)], self.ctrl_frame)

    @property
    def B_err_F(self):
        """Access-handle for the current F/T error w.r.t base frame"""
        return self.B_F_des - self.B_F_ext

    def calc_hybrid_force_vel_command(self):
        """Calculate hybrid force-velocity command via the """
        u_f_t, u_f_r = self._F_ctrl.u_F(self.B_err_F, self.R_B_C)
        u_v_t, u_v_r = self._v_ctrl.u_v(self.R_B_C)
        return np.r_[u_v_t + u_f_t, u_v_r + u_f_r]

    def _hybrid_force_vel_command(self):
        r"""Apply hybrid force-velocity control via the
        **sensor-track velocity-controller**
        """
        self._sns_trk_vel_handle.update_cmd(self.calc_hybrid_force_vel_command(), self.ctrl_frame)

    def _joint_ctrl_command(self):
        # todo: implement planner interface or proper joint control method
        rospy.logerr(
            f"mapping joints to a proper control signal in {self.sns_link} not implemented")
        pass

    def move_to_pose(self, T_des, err_gain=None, v_max=None, rot_max=None,
                     pos_precision_threshold=None, rot_precision_threshold=None, **_):
        """
        Previous ``goTo`` implementation.
        Sets the desired pose to ``T_des`` and alternates accuracy limits, gains and velocity limits as needed.

        Args:
            T_des(sm.SE3):  desired EE-pose
            err_gain(float, optional):  sets the error-gain for the Cartesian servo command. Defaults to None
            v_max(float, optional):  maximum velocity in m/s. Defaults to None.
            rot_max: maximum angular velocity in rad/s. Defaults to None
            pos_precision_threshold: position threshold to accept pose as reached. Defaults to None
            rot_precision_threshold: rotation threshold to accept pose as reached. Defaults to None

        Raises:
            AssertionError: if sensor-track velocity handle is not initialized
        """
        assert self._sns_trk_vel_handle is not None, \
            "Cartesian servoing expects the sns-vel-ctrl interface"
        if self._ft_handle is None:
            rospy.logwarn("this instance does not have a FT-sensor handle => high collision forces may be undetected")
        try:
            self.init_sns_vel()
            self.state = self._CART_SERVO
            self.T_des = T_des
            if err_gain is not None:
                self.err_gain = err_gain
            if v_max is not None:
                self.v_max = v_max
            if rot_max is not None:
                self.ω_max = rot_max
            if pos_precision_threshold is not None:
                self.transl_precision = pos_precision_threshold
            if rot_precision_threshold:
                self.rot_precision = rot_precision_threshold
        except ValueError as e:
            rospy.logerr(f"could not initiate Cartesian Servo command: {e}")

    def set_py_hybrid_force_vel_command(self, B_F_des, K_f, K_t, scale_pos, scale_rot, wrench_dir, vel_dir):
        """
        This sets the pythonized hybrid force-velocity control according to :py:meth:`hybrid_force_vel_command`
        """
        assert self._ft_handle is not None, "the hybrid FT-control expects a ft-sensor handle"
        assert self._sns_trk_vel_handle is not None, \
            "Pythonized hybrid force/velocity control expects the sns-vel-ctrl interface"
        try:
            self.init_sns_vel()
            self.state = self._HYBRID_CMD_SNS_VEL
            self._v_ctrl.B_v_des = vel_dir[0:3] * scale_pos * self.v_max
            self._v_ctrl.B_ω_des = vel_dir[3:6] * scale_rot * self.ω_max
            R_B_C = self.R_B_C
            self._v_ctrl.S = np.r_[R_B_C.T @ np.abs(vel_dir[0:3]) > 0.0,
                                   R_B_C.T @ np.abs(vel_dir[3:6]) > 0.0]
            self._F_ctrl.S = np.r_[R_B_C.T @ np.abs(wrench_dir[0:3]) > 0.0,
                                   R_B_C.T @ np.abs(wrench_dir[3:6]) > 0.0]
            self._F_ctrl.K_trans = K_f
            self._F_ctrl.K_rot = K_t
            matrix_parser(self.B_F_des, B_F_des, self.F_max)
        except ValueError as e:
            rospy.logerr(
                f"could not initiate Hybrid Force-velocity control: {e}")

    def cobot_state_msg(self, action_primitive="", status=None):
        """Parse current state to ``CobotState Status message``
        This can then be used via the state-publisher via

        .. code-block::
            python

            skill = SkillBase.from_ros()
            skill._pub_state.publish(skill.cobot_state_msg())

        Args:
            action_primitive: action_primitive if robot is currently executing specific sub-skills
            status(optional, int): optional robot status

        Returns:
            CobotState: current cobot state as publishable ROS-status message
        """
        msg = CobotState()
        msg.stamp = rospy.get_rostime()
        msg.tool.type = self.tool_id
        msg.primitive = action_primitive
        if status is None:
            msg.state = msg.IDLE
        else:
            msg.state = max(-128, min(status, 127))
        msg.ctrl = msg.READ_ONLY
        if self.state == self._CART_SERVO or self.state == self._HYBRID_CMD_SNS_VEL:
            msg.ctrl = CobotState.SNS_VEL_CTRL
        elif self.state == self._HYBRID_CMD_SNS_CMPL:
            msg.ctrl = CobotState.SNS_COMPL_CTRL
        elif self.state == self._JOINT_CTRL:
            msg.ctrl = CobotState.JOINT_CTRL
        return msg

    def publish_state(self, action_primitive="", status=None):
        """publish current cobot state"""
        if self._pub_state is None:
            rospy.logerr_once(
                "cannot publish robot state -> publisher has not been initialized")
            return
        self._pub_state.publish(self.cobot_state_msg(
            action_primitive=action_primitive, status=status))

    def update(self, sleep=True, u_cmd=None, u_cmd_frame=None, skip_forward=False, **_) -> None:
        r"""
        General control update routine
        Selects current strategy either internally via current state and dedicated command-function
        or alternatively forwards an optional ``u_cmd`` Cartesian velocity command w.r.t. optional reference
        frame ``u_cmd_frame``.
        In case the sleep argument is set, this function will set the class to sleep for the remainder of the
        current update step via the :py:meth:`sleep` function.

        Note:
            This command is sensitive against safety metrics as implemented in :py:meth:`~safety_check`

        Warnings:
            Against common notation, the control command is expected to be a Cartesian twist / velocity in SE(3).

        Args:
            sleep(bool, optional): will set the control-loop to sleep for remainder of `:math:\delta t`
            u_cmd(np.ndarray, optional): externally calculated control command.
            u_cmd_frame(np.ndarray, optional): reference frame for said control frame.
            skip_forward(bool, optional): skip update_tf. Defaults to False.
        """
        if not skip_forward:
            self.update_tf()
        if self.safety_check():
            if u_cmd is not None:
                self.publish_u_x_dot(u_cmd, frame_id=u_cmd_frame)
            elif self.state == self._CART_SERVO:
                self._cartesian_servo_command()
            elif self.state == self._HYBRID_CMD_SNS_CMPL:
                self._sns_trk_compl_handle.update_cmd()
            elif self.state == self._HYBRID_CMD_SNS_VEL:
                self._hybrid_force_vel_command()
            elif self.state == self._JOINT_CTRL:
                self._joint_ctrl_command()
        if sleep:
            #rospy.logerr(self._rate.remaining())
            # if self._rate.remaining() < rospy.Duration.from_sec(0):
            #     rospy.logerr(f"Update command found loop rate violation! We're lagging behind {1e-6*self._rate.remaining()}ms to keep frequency!")
            self.sleep()

    # use move_to_joint_pose(q_des, stochastic=True)
    # def stochastic_move_to_pose(self, q_des, **cart_servo_args) -> None:
    #     joint_traj = None
    #     if self._planner_interface.has_matlab:
    #         #joint_traj = self._planner_interface.get_joint_trajectory_to_pose(T_B_E_des)
    #          joint_traj = self._planner_interface.get_joint_trajectory_to_joint_configuration(q_des)
    #     # if not self.execute_joint_trajectory(joint_traj=joint_traj, wait_for_feedback=True):
    #     #     self.move_to_pose(T_B_E_des, **cart_servo_args)

    def stop(self):
        """
        Stop the robot via

        * reset current controller via ``stop`` handle
        * fill controller parameters as needed

        Note:
            This function needs to be called after reaching a goal-state as the robot will simply continue to drift
            with current set-velocity otherwise
        """
        if self.state == self._CART_SERVO or self.state == self._HYBRID_CMD_SNS_VEL:
            assert self._sns_trk_vel_handle is not None, "cannot stop sns-trk vel controller"
            self._sns_trk_vel_handle.stop()
        elif self.state == self._HYBRID_CMD_SNS_CMPL:
            assert self._sns_compl_compliance is not None, "cannot stop sns-trk compliance controller"
            self._sns_trk_compl_handle.S_v.S_diag.fill(0.0)
            self._sns_trk_compl_handle.S_F.S_diag.fill(0.0)
            self._sns_trk_compl_handle.update_cmd()
        self.state = None

    def sleep(self):
        """set robot to sleep via ``rospy.Rate.sleep``"""
        self._rate.sleep()

    def go_to_joint_hack(self):
        self.state = self._CART_SERVO
        T_des = self.T_des
        err_gain = self.err_gain
        v_max = self.v_max
        rot_max = self.ω_max
        prec_t, prec_r = self.transl_precision, self.rot_precision
        dq = self.joint_limit_avoidance_needed(d_limit=self._joint_limits_soft_dist)
        rospy.logerr(f"encountered joint limit violation on joints {np.where(dq != 0.0)}")
        q_des = self.q.copy() - dq
        self.move_to_joint_pose(q_des, stochastic=False)
        rospy.logerr(f"reset joint configuration to {q_des}. Return to motion with decreased precision")
        prec_t *= 1.1
        prec_r *= 1.1
        self.state = self._CART_SERVO
        self.T_des = T_des
        self.err_gain = err_gain
        self.v_max = v_max
        self.ω_max = rot_max
        self.transl_precision, self.rot_precision = prec_t, prec_r

    def goTo(self, T_des, *args, soft_limits=True, check_reachable=False,
             compensate_joint_limits=False, **kwargs):
        """backwards compatibility function"""
        if check_reachable:
            if not self.is_reachable(T_des):
                raise UnreachableException(f"cannot reach {T_des}")
        try:
            self.move_to_pose(T_des, *args, **kwargs)
        except AssertionError:
            rospy.logerr("could not initiate goTo motion")
            return
        self.compensate_joint_limits = compensate_joint_limits
        while self._state is not None:
            try:
                self.update()
            except (JointLimitWarning, JointLimitException):
                rospy.sleep(10)
                self.go_to_joint_hack()
            if self.state == "FAILURE":
                return rospy.logerr("cobot encountered an error. Check for collisions etc!")
        rospy.logdebug(f"reached desired goal-pose:\n{T_des}")

    def _sns_vel_compliance(self, compliance, K_f=8e-4, K_t=1e-3):
        self.set_py_hybrid_force_vel_command(np.zeros(6), K_f, K_t, wrench_dir=compliance,
                                             scale_pos=0.0, scale_rot=0.0, vel_dir=np.zeros(6))

    def _sns_compl_compliance(self, compliance, calibration_file, **kwargs):
        self.init_sns_compl(calibration_file=calibration_file, **kwargs)
        self.B_F_des.fill(0.0)
        self._sns_trk_compl_handle.F_des = self.B_F_des
        self._sns_trk_compl_handle.S_F.S_diag = compliance
        self._sns_trk_compl_handle.S_v.S_diag[:] = 0.0
        self.state = self._HYBRID_CMD_SNS_CMPL

    def set_compliant(self, compliance=np.zeros(6), calibration_file="/tmp/current.yaml", use_py_vel=False, **kwargs):
        if use_py_vel:
            self._sns_vel_compliance(compliance, **kwargs)
            return
        try:
            self._sns_compl_compliance(
                compliance, calibration_file=calibration_file, **kwargs)
        except (AssertionError, AttributeError, TypeError):
            rospy.logerr(
                "could not initiate compliance using the sensor-track compliance controller")
            self._sns_vel_compliance(compliance, **kwargs)

    def set_hybrid_force_vel_command(self, B_F_des, K_f, K_t, scale_pos, scale_rot, wrench_dir, vel_dir, **init_kwargs):
        """
        This sets the ROS-control hybrid force-velocity control according to :py:meth:`hybrid_force_vel_command`
        """
        assert self._ft_handle is not None, "the hybrid FT-control expects a ft-sensor handle"
        assert self._sns_trk_compl_handle is not None, \
            "hybrid force/velocity control expects the sns-ctrl-compliance interface"
        try:
            self.init_sns_compl(**init_kwargs)
            self.state = self._HYBRID_CMD_SNS_CMPL
            self._sns_trk_compl_handle.S_F = wrench_dir
            self._sns_trk_compl_handle.S_v = vel_dir
            matrix_parser(self.B_F_des, B_F_des, self.F_max)
            self._sns_trk_compl_handle.F_des = self.B_F_des
            self._sns_trk_compl_handle.x_dot_des = np.r_[scale_pos * self.v_max * vel_dir[0:3],
                                                         scale_rot * self.ω_max * vel_dir[3:6]]
            self._sns_trk_compl_handle.K_p_diag[0:3] = K_f
            self._sns_trk_compl_handle.K_p_diag[3:6] = K_t
        except ValueError as e:
            rospy.logerr(
                f"could not initiate Hybrid Force-velocity control: {e}")

    @property
    def B_x_dot_des(self):
        """Access-handle for the current feed-forward desired velocity terms w.r.t. base-frame"""
        return np.r_[self._v_ctrl.B_v_des, self._v_ctrl.B_ω_des]

    @B_x_dot_des.setter
    def B_x_dot_des(self, value):
        """Setter function for the desired feed-forward velocity terms w.r.t base-frame"""
        assert value.shape == (6,), "x_dot is in SE3 / 6-dimensional"
        self._v_ctrl.B_v_des = value[0:3]
        self._v_ctrl.B_ω_des = value[3:6]

    def init_sns_vel(self) -> None:
        """Initialize sensor-track velocity controller and save current control mode to spare checking with
        controller-manager at high frequencies"""
        if self._current_control_mode != "sns-vel":
            super().init_sns_vel()
            self._current_control_mode = "sns-vel"

    def init_sns_compl(self, *args, **kwargs):
        """Initialize sensor-track compliance controller and save current control mode to spare checking with
        controller-manager at high frequencies"""
        if self._current_control_mode != "sns-cmpl":
            super().init_sns_compl(*args, **kwargs)
            self._current_control_mode = "sns-cmpl"

    def deactivate_controllers(self):
        """deactivate controllers and reset control mode"""
        super(HrrCobotControl, self).deactivate_controllers()
        self._current_control_mode = ""

    @property
    def K_F_p(self):
        r"""access handle for proportional force control gain matrix in :math:`\mathbb{R}{^{6\times,6}`"""
        K_F_p = np.eye(6)
        np.fill_diagonal(K_F_p, np.r_[self._F_ctrl.K_pos_diag, self._F_ctrl.K_rot_diag])
        return K_F_p

    def publish_u_x_dot(self, u_x_dot, frame_id=None):
        """
        Publish desired velocity directly to robot w.r.t `tool` frame,
        c.f. :py:meth:`~tool`.

        Args:
            u_x_dot(np.ndarray): control velocity
            frame_id(Union[str, None]): reference frame
        """
        assert self._sns_trk_vel_handle is not None, \
            "Velocity command expects access to the sns-vel-ctrl interface"
        assert u_x_dot.shape == (6,), \
            f"dimension mismatch {u_x_dot.shape} for direct velocity feed-forward commands"
        if frame_id is None:
            frame_id = self.tool
        self._sns_trk_vel_handle.update_cmd(u_x_dot, frame=frame_id)

    def __str__(self) -> str:
        """human-readable string of current class instance to improve debug / and logging"""
        out = super().__str__()
        tmp = out.split("\n")
        if self.state is not None:
            tmp.insert(1, f"command-state:\t {self.state}")
        return "\n".join(tmp)

    def reset_gripper(self, w0=None, try_reset=True):
        if self.gripper is None:
            return
        self.gripper.set_vel(0)
        if w0 is None:
            if self.gripper.width > 15e-3:
                w0 = min(self.gripper.width + 5e-3, 109.99e-3)
            else:
                w0 = 80e-3
        self.gripper.send_pos(w0 + np.random.rand() * 1e-6, si=True)
        for _ in range(20):
            if np.linalg.norm(self.gripper.width - w0) < 1e-4:
                return
            rospy.sleep(0.1)
        if try_reset:
            for i in range(10):
                try:
                    self.gripper.reset()
                    rospy.sleep(0.25)
                    return self.reset_gripper(w0, False)
                except rospy.ServiceException:
                    continue
        else:
            raise RuntimeError("please restart gripper manually")

    def run_screwdriver_program(self, prog_num, run_time=1.0, sleep_time=1.0):
        """
        Run Screwdriver program.
        Workaround to trigger screwdriver rotation as Arduino controller may
        switch current control mode to idle, once the timeout is reached.
        Thus, this method runs:

        For the arduino-controller

        #. set the control mode via the ``tool`` attribute  of the ``tool_controller``.
        #. update the current screwdriver-program
        #. wait for a cycle to allow the tool-controller to be updated
        #. start the screwdriver in forward or reverse manner depending on arguments.

        Note:

            The argument ``run_time`` is only valid for the ROS-control interface and
            assures that the screwdriver is disabled after said time.

        Args:
            prog_num(int): program number
            run_time(float, optional): time to run screwdriver in seconds.. Defaults to 1.0.
            sleep_time(float, optional): sleep time before sending start command (Arduino runs at 100Hz)
        Raises:
            AssertionError: in case current tool is not set to screwdriver
            AssertionError: in case program is outside of limits
        """
        if self.tool_id != ToolType.SCREW_DRIVER:
            rospy.logerr(f"current tool is {self.tool}")
            return
        self.tool_controller.tool = self.tool
        try:
            self.tool_controller.run_screwdriver(dt=run_time, prog=prog_num, sleep_time=sleep_time)
        except AttributeError:
            self.tool_controller.screwdriver_program = prog_num
            rospy.sleep(sleep_time)
            self.tool_controller.screwdriver_start()

    def screwdriver_stop(self):
        if self.tool_id == ToolType.SCREW_DRIVER:
            self._tool_controller.screwdriver_stop()

    def collect_calibration_data(self, q_dot=np.r_[0.3, 0.4, 0.4], scale_range=np.r_[1.0, 0.9, 1.5]):
        r"""
        Apply a rotary motion around last three joints via

        .. math::

            {}^{B}{\dot{x}} = {\bf J}({\bf q}) \dot{\bf{q}}_{\mathrm{des}}

        .. warning::

            Joint velocities should not be put to high as underlying controller has active limits

        Args:
            q_dot(np.ndarray, optional): desired joint velocities.
            scale_range(np.ndarray, optional): scaling to adjust elevation during calibration

        Returns:
            :py:class:`~hrr_common.utils.data_types.CalibDataFull`: recorded data stream of joints, positions, rotations and FT-readings that can be used for calibration.
        """

        def rotate_via_q(q_dot_max, direction, T):
            start = rospy.get_time()
            for j in range(T):
                x_dot_ee = self.Jac() @ np.r_[np.zeros(3), direction] * q_dot_max * np.sin(2 * j * np.pi / T)
                self.update(u_cmd=x_dot_ee, u_cmd_frame=self.base_link, sleep=True)
                try:
                    data.add(np.copy(self._ft_handle.wrench), self.FK(self.q), self.q)
                except AttributeError:
                    pass
                if rospy.get_time() - start > T:
                    break
            self.stop()

        def check_joint(q_i, q_dot_max, scale_max=1.0):
            assert 3 <= q_i < 6, "rotation limited to last three joints"
            limits = np.min(np.c_[np.pi * scale_max * np.ones(2), np.abs(self.q[q_i] - self.joint_limits)[q_i]], axis=1)
            select_joints = np.eye(3)
            for j, limit in enumerate(limits):
                direction = -1 if j == 0 else 1
                rotate_via_q(direction=select_joints[q_i - 3] * direction,
                             T=int(limit * self.hz / q_dot_max),
                             q_dot_max=q_dot_max)

        self.init_sns_vel()
        data = hrr_common.DataBuffer()
        for i, (q_dot_des, scale) in enumerate(zip(q_dot, scale_range)):
            check_joint(i + 3, q_dot_des, scale)
        return data.all_data

    def run_calibration_routine(self, file_name=None, plot=False, bias_hack=False, force_execution=False,
                                **kwargs) -> None:
        """run calibration routine

        #. check controllers as needed
        #. update FT->EE transformation
        #. in case the file-name exists, load values and return
        #. move robot to calibration pose via :py:meth:`~move_to_calibration_pose` / :py:meth:`~move_to_joint_pose`
        #. collect calibration data according to :py:meth:`~collect_calibration_data`
        #. regress sensor offset according to
           :py:meth:`~hrr_cobot.ros_interfaces.sensor_handles.free_space_offset_regression`
        #. (optionally) add remaining offset to bias
        #. update ros-parameters
        #. (optionally) safe result in file

        the storing / loading in a data file is intended to decrease time needed / prevent frequent recalibration.

        Args:
            file_name(Union[Path, str, None], optional): load calibration data from file. Defaults to None.
            plot(bool, optional): plotting flag for :py:func:`free_space_offset_regression`. Defaults to False.
            force_execution(bool, optional): Executes calibration routine even though, the transformation from EE->Ft is not set. Defaults to False.
            bias_hack(bool, optional): small hack for remaining calibration offset. Defaults to True.
            **kwargs: optional arguments, forwarded to :py:meth:`~collect_calibration_data`.
        """
        assert self._sns_trk_vel_handle is not None, "this routine expects a sensor-track velocity control handle"
        assert self._ft_handle is not None, "this routine expects a FT-sensor handle"
        cur_controllers = self.active_controllers
        self.deactivate_controllers()
        if self.R_FT_E is None or np.all(self.R_FT_E == np.eye(3)):
            self.set_EE_FT_transform()
        if np.all(self.R_FT_E == np.eye(3)):
            if not force_execution:
                rospy.logerr(f"[{rospy.get_name()}] rotation from EE->FT is set to identity. Stop calibration.")
                return
            rospy.logwarn(f"[{rospy.get_name()}] rotation from EE->FT is set to identity. Check your setup carefully.")
        if file_name:
            try:
                self._ft_handle.load_offset(file_name)
                self.update_tf()
                return
            except FileNotFoundError:
                rospy.logerr(f"could not find file {file_name}")
        if not self.can_calibrate:
            self.move_to_calibration_pose()
        self._sns_trk_vel_handle.activate()
        data = self.collect_calibration_data(**kwargs)
        self.update_tf()
        self._ft_handle.set_offset(
            *free_space_offset_regression(data, self.R_FT_E, plot=plot, **kwargs))
        if bias_hack:
            self._ft_handle.bias += self._ft_handle.filtered
        self._ft_handle.set_params()
        if file_name:
            self._ft_handle.save_offset(file_name)
        self.update_tf()
        self.reactivate_controllers(cur_controllers)

    def set_tool_frame(self, tool_type, tool_name=None, robot_urdf_prefix="hrr_cobot.") -> None:
        super().set_tool_frame(tool_type=tool_type, tool_name=tool_name, robot_urdf_prefix=robot_urdf_prefix)
        if tool_type == ToolType.WSG_50_DSA:
            self.gripper = DsaGripperCommandHandle.from_ros(ros_param_prefix="/")
        else:
            self.gripper = None

    @staticmethod
    def default_hybrid_kwargs() -> dict:
        """Default key-value based arguments for the hybrid control command, i.e.

        * :py:meth:`~set_py_hybrid_force_vel_command`
        * :py:meth:`~set_hybrid_force_vel_command`
        """
        return dict(B_F_des=np.zeros(6), wrench_dir=np.zeros(6),
                    K_f=np.zeros(3), K_t=np.zeros(3),
                    vel_dir=np.zeros(6), scale_pos=np.zeros(3), scale_rot=np.zeros(3))

    def joint_limit_repelling_velocity_term(self, gain=1.0):
        """

        Returns:
            np.ndarray:

        Raises:
            JointLimitException: in case any joint state is below hard limit
        """
        dq = self.joint_limit_distance()
        dp_critical = self.joint_limit_avoidance_needed(d_limit=self._joint_limits_soft_dist, dq=dq)
        if np.all(dp_critical == 0.0):
            return np.zeros(6)
        if np.any(self.joint_limit_avoidance_needed(d_limit=self._joint_limits_hard_dist, dq=dq) != 0.0):
            self.stop()
            rospy.logerr(f"joints are very close to the limits: d := {dq}")
            raise JointLimitException(f"joints very close to joint limits. stop execution")
        rospy.logwarn_throttle(0.2, f"joint(s) {[i + 1 for i in np.where(dp_critical != 0)[0]]} are/is critical.")
        idx = np.where(dp_critical != 0.0)
        q_dot_correction = np.zeros(6)
        q_dot_correction[idx] = -1 * np.sign(dp_critical[idx]) * np.exp(
            -1 * (self._joint_limits_soft_dist[idx] - np.abs(dp_critical[idx]) / (
                (self._joint_limits_soft_dist[idx] - self._joint_limits_hard_dist[idx]))) ** 2
        )
        return gain * q_dot_correction

    def cartesian_correction_velocity(self, gain=1.0):
        return self.Jac() @ self.joint_limit_repelling_velocity_term(gain)

    def legal_joint_config(self, q, soft_limits=True):
        if all([soft_limits, np.all(q > self.joint_limits[:, 0] + self._joint_limits_soft_dist),
                np.all(q < self.joint_limits[:, 1] - self._joint_limits_soft_dist)]):
            return True
        if all([not soft_limits,
                np.all(q > self.joint_limits[:, 0] + self._joint_limits_hard_dist),
                np.all(q < self.joint_limits[:, 1] - self._joint_limits_hard_dist)]):
            return True
        return False

    @property
    def tool_changer_open(self):
        return self._tool_controller.tool_changer_open
