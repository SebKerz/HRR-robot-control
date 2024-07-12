#!/usr/bin/env python3
"""
HRR Cobot Interface
----------------------

This file wraps the different ROS-interfaces provided into a single class,
while also allowing to instantiate subclasses as needed, in detail, theses sub-handles are given as

* ``_robot``: see :py:class:`~hrr_controllers.robot_status_if.hrr_controllers.ComauRobotState` and :py:class:`~hrr_controllers.robot_status_if.hrr_controllers.RobotState`
* ``_ft_handle``: see :py:class:`~hrr_controllers.sensor_handles.hrr_controllers.FTBuffer` and  :py:class:`~hrr_controllers.sensor_handles.hrr_controllers.FTData`
* ``_joint_traj_handle``: see :py:class:`~hrr_controllers.trajectory_handling.hrr_controllers.JointTrajectoryHandler`
* ``_planner_interface``: see :py:class:`~hrr_cobot_robot.motion_planner_if.MotionPlannerIf`
* ``_sns_trk_vel_handle``: see :py:class:`~hrr_controllers.sensor_track_velocity_commander.hrr_controllers.SnsTrkCmd`
* ``_sns_trk_compl_handle``: see :py:class:`~hrr_controllers.sensor_track_compliance_commander.hrr_controllers.SnsTrkComplCmd`
* ``_tool_controller``: see :py:class:`~hrr_cobot_robot.tool_control.pilot_tool_controller.HrrCobotToolControlInterface`


ROS-parameters
^^^^^^^^^^^^^^^^^^^^^^^^^

All ROS-parameters are prepended by a ``{cobot_prefix}``, usually ``/hrr_cobot/``

========================= ================== =======================================================================
Parameter                 default value      notes
========================= ================== =======================================================================
sns_frame                 tcp_controller     name of sensor-tracking tool-frame
base_link_name            base_link          name of robot base link
ee_link_name              tcp_controller     name of robot ee/flange link
ft_link_name              jr3msr             name of F/T-sensor link name
tool                      None               name of current tool
v_max                     0.02               translation velocity limit in [m/s]
omega_max                 0.1                rotation velocity limit in [rad/s]
tool_name                 - (but optional)   name of current robot-tool
buffer_ft_data            True               flag to instantiate ``_ft_handle`` as hrr_controllers.FTBuffer or hrr_controllers.FTData
cmd_sns_vel               True               flag to instantiate ``_sns_trk_vel_handle``
cmd_sns_compl             False              flag to instantiate ``_sns_trk_compl_handle``
read_tp5                  True               flag to instantiate ``_robot`` as hrr_controllers.ComauRobotState or hrr_controllers.RobotState
========================= ================== =======================================================================

.. note::

    * the handles ``_joint_traj_handle``,  ``_tool_controller`` and ``_planner_interface`` are instantiated by default
    * the arduino-interface has been migrated the tool_controller handle directly

"""
# inbuilt imports
from typing import Optional, Union, Dict, List

# 3rd party imports
import numpy as np
import quaternion
import spatialmath as sm

# ROS imports
import rospy
from hr_recycler_msgs.msg import ToolType
from std_srvs.srv import TriggerResponse

import hrr_common
import hrr_controllers

# current module import
from hrr_cobot_robot.utils import tool2tool_type, tool_type2str
from hrr_cobot_robot.tool_control import HrrCobotToolControlInterface, ArduinoToolControlInterface
from hrr_cobot_robot.motion_planner_if import MotionPlannerIf

__all__ = ["HrrCobotIf"]


class HrrCobotIf(hrr_common.RosBaseHandle):
    """Python HR-Recycler Cobot manager
    This class handles the Hr-Recycler Cobot combining the individual capabilities and
    wrapping individual functionalities in an easy-to-use API.

    The initialization depends on the flags set handed to :py:class:`~init_ros`.
    """

    def __init__(self,
                 sns_link="tcp_controller",
                 base_link="base_link",
                 ee_link="tcp_controller", ft_link="jr3msr",
                 v_max=0.02, ω_max=0.1, **_):
        self.sns_link = sns_link
        self.v_max = v_max
        self.ω_max = ω_max
        self.base_link = base_link
        self.ee_link = ee_link
        self.ft_link = ft_link
        self._tf = hrr_common.TfHandler(add=False)
        self._joint_limits_soft_dist = np.r_[0.1 * np.ones(4), 0.2, 0.1]
        self._joint_limits_hard_dist = np.r_[0.01 * np.ones(4), 0.13, 0.01]
        self._tcp_frame = ""
        self._cur_tool = ToolType.NONE
        # ROS handles
        self._pub_tool = None  # type: Optional[rospy.Publisher]
        self.E_p_EFT = None  # type: Optional[np.ndarray]
        self.R_FT_E = None  # type: Optional[np.quaternion]
        self._robot = None  # type: Optional[Union[hrr_controllers.ComauRobotState, hrr_controllers.RobotState]]
        self._ft_handle = None  # type: Optional[Union[hrr_controllers.FTBuffer, hrr_controllers.FTData]]
        self._joint_traj_handle = None  # type: Optional[hrr_controllers.JointTrajectoryHandler]
        self._planner_interface = None  # type: Optional[MotionPlannerIf]
        self._sns_trk_vel_handle = None  # type: Optional[hrr_controllers.SnsTrkCmd]
        self._sns_trk_compl_handle = None  # type: Optional[hrr_controllers.SnsTrkComplCmd]
        self._tool_controller = None  # type: Optional[Union[HrrCobotToolControlInterface, ArduinoToolControlInterface]]
        self._name = ""
        self.E_p_EC = np.zeros(3)
        self.R_E_C = np.eye(3)

    @property
    def tool_controller(self) -> Union[HrrCobotToolControlInterface, ArduinoToolControlInterface, None]:
        """get current tool-controller"""
        if self.tool_id <= ToolType.NONE:
            rospy.logwarn("trying to access tool-controller without a current tool")
            return
        if self._tool_controller is None:
            rospy.logerr_once("this current handle does not have a tool-control interface. Check your setup")
        return self._tool_controller

    @property
    def tool(self) -> str:
        if self._tool_controller is not None:
            return self._tool_controller.tool
        return tool_type2str(self._cur_tool)

    @tool.setter
    def tool(self, value) -> None:
        assert isinstance(value, str), f"cannot set tool to type {type(value)}"
        if self._tool_controller is not None:
            self._tool_controller.tool = value
            value = self._tool_controller.tool
        try:
            self._cur_tool = tool2tool_type(value)
        except (TypeError, AssertionError, RuntimeError):
            pass

    @property
    def tool_id(self) -> int:
        return self._cur_tool

    @tool_id.setter
    def tool_id(self, value: int) -> None:
        try:
            tool_type2str(value)
        except ValueError as e:
            rospy.logerr(f"cannot update tool-ID due to {e}")
            return
        self._cur_tool = value
        self._tool_controller.tool = value

    @property
    def T_B_C(self) -> sm.SE3:
        T_B_C = sm.SE3(*self.E_p_EC)
        T_B_C.A[0:3, 0:3] = self.R_E_C
        return T_B_C

    def set_tool(self, E_p_EC, R_E_C=None) -> None:
        DeprecationWarning("this function should not be used anymore. Please use ``set_tool_from_ros_frame``")
        self.E_p_EC = E_p_EC
        if R_E_C is not None:
            self.R_E_C = R_E_C

    def update_ee_tcp_tf(self):
        """Outsourced from :py:meth:`~set_tool_from_ros_frame` to recall easily."""
        E_p_EC, q_EC = self._tf.T_A_B(A=self.ee_link, B=self._tcp_frame)
        self.E_p_EC = E_p_EC
        self.R_E_C = quaternion.as_rotation_matrix(q_EC)

    def set_tool_from_ros_frame(self, name, tool_frame_id) -> None:
        """set tool-frame from ROS using the tf-tree

        Args:
            name(str): name of tool to be parsed to :py:meth:`~tool`-property
            tool_frame_id(str): link-name that is expected to be part of the TF-tree
        """
        self._tcp_frame = tool_frame_id
        self.tool = name
        self.update_tf()

    def init_ros(self, cobot_prefix,
                 use_ft_buffer=True, use_sns_vel=True, use_cobot_state=True,
                 compile_numba=False, **_) -> None:
        self._ft_handle = hrr_controllers.FTBuffer.from_ros(cobot_prefix=cobot_prefix) if use_ft_buffer \
            else hrr_controllers.FTData.from_ros(cobot_prefix=cobot_prefix)
        self._robot = hrr_controllers.ComauRobotState.from_ros(cobot_prefix=cobot_prefix) if use_cobot_state \
            else hrr_controllers.RobotState.from_ros(cobot_prefix=cobot_prefix)
        if hrr_common.get_param(f"{cobot_prefix}cmd_sns_vel", use_sns_vel):
            self._sns_trk_vel_handle = hrr_controllers.SnsTrkCmd.from_ros(cobot_prefix=cobot_prefix)
        if hrr_common.get_param(f"{cobot_prefix}cmd_sns_compl", use_sns_vel):
            self._sns_trk_compl_handle = hrr_controllers.SnsTrkComplCmd.from_ros(cobot_prefix=cobot_prefix)
        self._joint_traj_handle = hrr_controllers.JointTrajectoryHandler.from_ros(cobot_prefix=cobot_prefix)
        self._tool_controller = HrrCobotToolControlInterface.from_ros(cobot_prefix=cobot_prefix)
        self._planner_interface = MotionPlannerIf.from_ros(cobot_prefix=cobot_prefix)
        self._pub_tool = rospy.Publisher(
            f"{cobot_prefix}{hrr_common.get_param(f'{cobot_prefix}set_tool_name_topic', 'set_tool')}",
            ToolType, queue_size=10)
        self.change_tool(hrr_common.get_param(f"{cobot_prefix}tool_name", "nothing"))
        self.initialize_handle(compile_numba=compile_numba)

    def initialize_handle(self, compile_numba=False):
        """
        Initialize major handles and update states

        Args:
            compile_numba(bool, optional): compile current symbolic models
        """
        # call FK and Jac to generate basic numba-functions
        self.FK(self.q)
        self.Jac()
        if compile_numba:
            rospy.loginfo("compile current robot model... ")
            self._robot.get_model()
        else:
            rospy.loginfo(f"some functions are not yet compiled. Expect delays upon first call")
        self.set_EE_FT_transform()
        self.update_tf()
        self.update_ee_tcp_tf()

    @property
    def model(self):
        return self._robot.get_model(False, False)

    @property
    def has_sns_vel(self) -> bool:
        return self._sns_trk_vel_handle is not None

    @property
    def FT(self) -> Union[hrr_controllers.FTData, hrr_controllers.FTBuffer]:
        assert self._ft_handle is not None, "recall init ros with ``use_ft`` set to ``True``"
        return self._ft_handle

    @classmethod
    def _from_ros(cls, cobot_prefix, compile_numba=False, **_):
        out = cls()
        out.sns_link = hrr_common.get_param(f"{cobot_prefix}sns_frame", out.sns_link)
        out.base_link = hrr_common.get_param(f"{cobot_prefix}base_link_name", out.base_link)
        out.ft_link = hrr_common.get_param(f"{cobot_prefix}ft_link_name", out.ft_link)
        out.ee_link = hrr_common.get_param(f"{cobot_prefix}ee_link_name", out.ee_link)
        out.v_max = hrr_common.get_param(f"{cobot_prefix}v_max", out.v_max)
        out.ω_max = hrr_common.get_param(f"{cobot_prefix}omega_max", out.ω_max)
        out.init_ros(cobot_prefix=cobot_prefix, compile_numba=compile_numba,
                     use_ft_buffer=hrr_common.get_param(f"{cobot_prefix}buffer_ft_data", True),
                     use_cobot_state=hrr_common.get_param(f"{cobot_prefix}read_tp5", True)
                     )
        out._joint_limits_soft_dist[:] = hrr_common.get_param(f"{cobot_prefix}soft_joint_limit_distance",
                                                              out._joint_limits_soft_dist)
        out._joint_limits_hard_dist[:] = hrr_common.get_param(f"{cobot_prefix}hard_joint_limit_distance",
                                                              out._joint_limits_hard_dist)
        out.update_ee_tcp_tf()
        return out

    @staticmethod
    def shutdown() -> None:
        rospy.loginfo(
            f"[{rospy.get_name()}] Shutting down Hrr-Cobot Python Handler...")

    def __str__(self) -> str:
        out = "Hrr-Cobot state:"
        if self._ft_handle is not None:
            out += f"\nFT-data:\n{self._ft_handle}\n---"
        if self._robot is not None:
            out += f"\nRobot-data:\n{self._robot}\n---"
        if self.needs_reset:
            out += '\033[01m\033[31m ROBOT needs reset!\033[0m\n---'
        if self.tool is not None and self.tool != "":
            out += f"\ntool set to {self.tool}\n"
            if np.linalg.norm(self.E_p_EC) > 0.0:
                out += f"E_p_EC:={self.E_p_EC}\n"
            if np.any(self.R_E_C != np.eye(3)):
                out += f"R_E_C:={self.R_E_C}"
            out += "\n---"
        return out

    # FT-sensor interfaces
    @property
    def FT_F(self) -> np.ndarray:
        """
        wrapper for :py:meth:`hrr_cobot_robot.ros_interfaces.sensor_handles.hrr_controllers.FTBuffer.wrench_calib`.

        calibrated sensor reading in FT-sensor frame
        Returns zeros, if no sensor has been initialized.


        Returns:
            np.ndarray: calibrated FT-reading.  (6 x 1)
        """
        if self._ft_handle is None:
            return np.zeros(6)
        return self._ft_handle.wrench_calib

    def _calc_B_F_msr(self) -> np.ndarray:
        def _quat_rotate_vec(q1, v):
            return (q1 * np.quaternion(0, *v) * q1.conjugate()).vec

        if self.R_FT_E is None:
            return np.zeros(6)

        return np.r_[_quat_rotate_vec(self.sns_quat, self.R_FT_E.T @ self.FT_F[0:3]),
                     _quat_rotate_vec(self.sns_quat, self.R_FT_E.T @ self.FT_F[3:6])]

    def reset_sensor(self) -> None:
        try:
            self._ft_handle.reset()
        except AttributeError:
            rospy.logerr("cannot reset FT-buffer -> no FT buffer initialized")

    @property
    def B_F_msr(self) -> np.ndarray:
        r"""
        Get Cartesian wrench in body frame of the robot

        .. math::

            {}^{B}{\bf{F}}_{\mathrm{msr}} = {\bf R}^{B}_{FT} {}^{FT}{\bf{F}}_{\mathrm{msr}}

        where :math:`{F}` denotes the Cartesian wrench

        Note:
            * this method only rotates measurements without adjusting for any lever-based torque alternations

        .. warning::

            This is shitty hack, utterly stupid, incorrect and should be removed soon

        Returns:
            np.ndarray: :math:`{}^{B}{\bf{F}}_{\mathrm{msr}}`
        """
        return self._calc_B_F_msr()

    @property
    def B_F_ext(self) -> np.ndarray:
        r"""
        get the external environment interaction force
        """
        return -self.B_F_msr

    # attributes from robot-handle
    @property
    def q(self) -> np.ndarray:
        """
        wrapper for :py:meth:`hrr_cobot_robot.ros_interfaces.robot_if.hrr_controllers.RobotState.q`.
        If inaccessible return zero-vector."""
        try:
            return self._robot.q
        except AttributeError:
            rospy.logerr("cannot get joints from uninitialized handle")
            return np.zeros(6)

    @property
    def q_calib(self) -> np.ndarray:
        try:
            return self._robot.q_calib
        except AttributeError:
            rospy.logerr("cannot get calibration configuration from uninitialized handle")
            return self.q

    @property
    def sns_pos(self) -> np.ndarray:
        """ wrapper for :py:meth:`hrr_cobot_robot.ros_interfaces.robot_if.hrr_controllers.ComauRobotState.pos`.
        If inaccessible return zero-vector.

        Returns:
            np.ndarray: ee-translation (3 x 1)
        """
        try:
            return self._robot.pos
        except AttributeError:
            rospy.logerr(
                "current instance does not have information about sns-translation")
            return np.zeros(3)

    @property
    def sns_quat(self) -> np.quaternion:
        """ wrapper for :py:meth:`hrr_cobot_robot.ros_interfaces.robot_if.hrr_controllers.ComauRobotState.quat`"""
        try:
            return self._robot.quat
        except AttributeError:
            rospy.logerr(
                "current instance does not have information about sns-orientation")
            return np.quaternion(1.0, 0.0, 0.0, 0.0)

    def FK(self, *args, **kwargs) -> sm.SE3:
        """ wrapper for :py:meth:`hrr_cobot_robot.ros_interfaces.robot_if.hrr_controllers.RobotState.FK`"""
        try:
            return self._robot.FK(*args, **kwargs)
        except AttributeError:
            rospy.logerr(
                "current instance does not have access to forward-kinematic")
            return sm.SE3()

    def IK(self, T_E_B, **kwargs) -> List[np.ndarray]:
        """ wrapper for :py:meth:`hrr_cobot_robot.ros_interfaces.robot_if.hrr_controllers.RobotState.IK`

        Args:
            T_E_B(sm.SE3 or np.ndarray): Transformation from base to ee :math:`{}^{E}{\bf T}_{B}`

        Returns:
            List[np.ndarray]: possible IK-solutions
        """
        try:
            return self._robot.IK(T_E_B, **kwargs)
        except AttributeError:
            rospy.logerr(
                "current instance does not have access to inverse-kinematic")
            return []
        except ValueError:
            rospy.logerr(f"could not find a solution for:\n{T_E_B}")
            return []

    def Jac(self, base=0, target=None) -> np.ndarray:
        try:
            return self._robot.J(self.q, base=base, target=target)
        except AttributeError:
            rospy.logerr(
                "current instance does not have access to inverse-kinematic")
            return np.zeros((6, 6))

    @property
    def joint_limits(self) -> np.ndarray:
        try:
            return self._robot.joint_limits
        except AttributeError:
            rospy.logerr(
                "current instance does not have access to joint limits")
            return np.zeros((6, 2))

    @property
    def T_B_E_robot(self) -> sm.SE3:
        """wrapper for :py:meth:`hrr_cobot_robot.ros_interfaces.robot_if.hrr_controllers.ComauRobotState.T_B_E`

        Returns:
            sm.SE3: forward-kinematic output but as read from tf2-ros
        """
        T_B_E = sm.SE3()
        try:
            return self._robot.T_B_E
        except AttributeError:
            rospy.logerr(
                "current instance does not have access to transformation of robot kinematic chain")
        return T_B_E

    @property
    def T_E_C_robot(self) -> sm.SE3:
        T_E_C = sm.SE3(*self.E_p_EC)
        T_E_C.A[:3, :3] = self.R_E_C
        return T_E_C

    @property
    def T_B_C_robot(self) -> sm.SE3:
        return self.T_B_E_robot @ self.T_E_C_robot

    @property
    def kinematic_consistent(self) -> bool:
        """wrapper for :py:meth:`hrr_cobot_robot.ros_interfaces.robot_if.hrr_controllers.ComauRobotState.kinematic_consistent`
        """
        try:
            return self._robot.kinematic_consistent
        except AttributeError:
            rospy.logerr("current instance cannot check kinematic consistency")
            return False

    def set_EE_FT_transform(self) -> None:
        """set_EE_FT_transform set the transformation from end-effector to FT sensor
        """
        self.R_FT_E = quaternion.as_rotation_matrix(self._tf.T_A_B(A=self.ft_link, B=self.ee_link)[1])
        rospy.logdebug(f"set R_FT_E to:\n{self.R_FT_E}")

    # controller handle interfaces
    @property
    def controller_manager(self) -> str or None:
        for ctrl in self.active_controllers.keys():
            if isinstance(getattr(self, ctrl), hrr_common.BaseController):
                return getattr(self, ctrl).controller_manager_ns

    @property
    def active_controllers(self) -> Dict[str, bool]:
        """get list of active controllers as dictionary,
        where th keys denote the member name and the
        value is ``True`` if the controller is active"""

        def get_status(ctrl):
            try:
                return ctrl.active
            except AttributeError:
                return None

        status = dict()
        for x in (("_sns_trk_compl_handle",
                   "_sns_trk_vel_handle",
                   "_joint_traj_handle")):
            s = get_status(getattr(self, x))
            if s is None:
                continue
            status[x] = s
        return status

    def deactivate_controllers(self) -> None:
        """Deactivate all controllers in a safe manner, i.e.
        catch errors from uninitialized controller handles.
        """

        def save_deactivate(ctrl: Optional[hrr_common.BaseController]):
            try:
                ctrl.deactivate()
            except AttributeError:
                pass

        save_deactivate(self._sns_trk_compl_handle)
        save_deactivate(self._sns_trk_vel_handle)
        save_deactivate(self._joint_traj_handle)

    def activate_controller(self, ctrl_name) -> None:
        """Activate a controller based on a human-readable name"""
        if ctrl_name == "sns-velocity":
            try:
                self._sns_trk_vel_handle.activate()
            except AttributeError:
                rospy.logerr(
                    "cannot activate sensor track velocity controller ")
        elif ctrl_name == "sns-compliance":
            try:
                self._sns_trk_compl_handle.activate()
            except AttributeError:
                rospy.logerr(
                    "cannot activate sensor track compliance controller ")
        elif ctrl_name == "joint-trajectory":
            try:
                self._joint_traj_handle.activate()
            except AttributeError:
                rospy.logerr("cannot activate joint trajectory handler ")
        else:
            rospy.logerr(f"unknown controller choice {ctrl_name}")

    def reactivate_controllers(self, prev_state: dict) -> None:
        """reactivate controllers according to previous configuration saved in ``prev_state``.
        """
        for ctrl, active in prev_state.items():
            if active:
                try:
                    getattr(self, ctrl).activate
                except AttributeError:
                    rospy.logerr(
                        f"failed on activating controller handle {ctrl}")

    def _check_for_reset(self):
        if self.needs_reset:
            raise RuntimeError(f"The robot is currently in status: {self._robot.robot_status} => please restart")

    def init_sns_vel(self) -> None:
        """shortcut command to activate sensor-track velocity control

        Raises:
            AssertionError: if initialization fails
        Note:
            due to runtime convenience the RuntimeError is now replaced with a simple ros-error message
        """
        self.deactivate_controllers()
        self.reset_sns_reference_pose()
        self._sns_trk_vel_handle.activate()
        assert self._sns_trk_vel_handle.active, "initialization of sensor-track velocity control failed"
        try:
            self._check_for_reset()
        except RuntimeError as e:
            rospy.logerr(f"cannot activate sensor-track velocity controller: {e}")

    def acknowledge_calibration(self):
        try:
            self._sns_trk_compl_handle.acknowledge_calibration()
        except AttributeError:
            rospy.logerr("cannot accept calibration -> no compliance controller initialized")

    def init_sns_compl(self, calibration_file, F_max=2.0, force=False, wait_time=0.2) -> None:
        """shortcut command to activate sensor-track velocity control

        Raises:
            AssertionError: if initialization fails
        """
        if not self._sns_trk_compl_handle.active:
            self._sns_trk_compl_handle.activate()
        self.deactivate_controllers()
        self.reset_sns_reference_pose()
        try:
            self.run_calibration_routine(file_name=calibration_file)
        except AttributeError:
            rospy.logerr("this module does not hae a calibration routine. Use HrrCobotControl instead")
        rospy.sleep(wait_time)
        if np.linalg.norm(self.B_F_msr) > F_max:
            rospy.logwarn(f"Attention: |F| = {np.linalg.norm(self.B_F_msr)} after calibration. Rerun with ``force``")
            if force:
                self.acknowledge_calibration()
        else:
            self.acknowledge_calibration()
        self.deactivate_controllers()

        assert self._sns_trk_compl_handle.active, "initialization of sensor-track velocity control failed"
        try:
            self._check_for_reset()
        except RuntimeError as e:
            rospy.logerr(f"cannot activate sensor-track velocity controller: {e}")

    def init_joint_trajectory_control(self) -> None:
        """initialize joint trajectory control, i.e. deactivate all active controllers and
        activate joint trajectory controller

        Raises:
            AttributeError: if joint-trajectory-handler is not initialized
        """
        self.deactivate_controllers()
        self._joint_traj_handle.assert_active()

    def execute_joint_trajectory(self, joint_traj, wait_for_feedback=True, q_des=None) -> bool:
        if joint_traj is None and q_des is None:
            rospy.logerr("cannot move to goal -> neither joint trajectory nor desired joint-goal is provided")
            return False
        if self._joint_traj_handle is None:
            rospy.logerr("cannot move to goal  -> no joint trajectory handle is initialized")
            return False
        if joint_traj is None:
            wait_for_feedback = True
            self._joint_traj_handle.linear_joint_cmd(self.q, q_des, wait_for_feedback=wait_for_feedback,
                                                     print_traj=False)
        elif isinstance(joint_traj, np.ndarray):
            return self._joint_traj_handle.command_joint_trajectory(joint_traj, wait_for_feedback=wait_for_feedback)
        else:
            return self._joint_traj_handle.command_joint_trajectory_msgs(joint_traj,
                                                                         wait_for_feedback=wait_for_feedback)
        return True

    def move_to_joint_pose(self, q_des, stochastic=False) -> None:
        """
        Move to a fixed joint pose, refer to
        :py:class:`hrr_cobot_robot.ros_interfaces.cobot_helper.hrr_controllers.JointTrajectoryHandler`
        for insights about the implementation / functionality.

        Args:
            q_des(np.ndarray): desired joint configuration
            stochastic(bool, optional): if True, the sometimes dangerous motion-planner is used.
        Note:
            This function does not check for possible collisions. Use with care!!
        """

        def matlab_exec():
            if self._planner_interface.has_matlab:    
                joint_traj = self._planner_interface.get_joint_trajectory_to_joint_configuration(q_des)
                self.execute_joint_trajectory(joint_traj=joint_traj, wait_for_feedback=True)

        def linear_joint_traj():
            self.execute_joint_trajectory(None, wait_for_feedback=True, q_des=q_des)

        dq = np.linalg.norm((q_des - self.q))
        if dq <= 1e-5:
            rospy.logdebug(f"current configuration close to current ({dq * 1e3 :.2f} [milli-rad]). keep pose")
            return
        cur_controllers = self.active_controllers
        self.init_joint_trajectory_control()
        if stochastic:
            try:
                rospy.loginfo("Trying to move with planner.")
                matlab_exec()
            except: 
                rospy.logerr("Moving with linear joints! Planner died.")
        rospy.loginfo("Moving linearly if planner didn't move. Take care!")
        linear_joint_traj()
        rospy.logdebug("re-/deactivate controllers")
        self._joint_traj_handle.deactivate()
        self.reactivate_controllers(cur_controllers)

    def stochastic_move_to_pose(self, T_B_E_des, **_) -> None:
        """Use Motion planner interface to generate a joint-trajectory that drives the robot from
        current pose -> somewhere -> to T_B_E_des,
        where the path from A -> somewhere is collision-safe,
        while you have to cross your fingers for the last step.

        good-luck!

        Args:
             T_B_E_des(np.ndarray or sm.SE3): goal-pose
        """
        cur_controllers = self.active_controllers
        self.init_joint_trajectory_control()
        joint_traj = None

        #use default IK to get multiple possible q_des
        qs_des = self.IK(T_B_E_des)

        lower_joint_limits = [-2.9, -1.1, -2.6, -3.4, -1.8, -3.14]
        upper_joint_limits = [2.9, 1.9, 1, 3.4, 1.7, 3.14]

        violating_indices = []
        for k,elem in enumerate(qs_des):
            if any((elem-lower_joint_limits)<=0) or any((elem-upper_joint_limits)>=0):
                violating_indices.append(k)
        qs_des = np.delete(qs_des, violating_indices, axis=0)

        #Get absolute distance to all q_des from current q
        qs_diff = [np.abs(elem-self.q) for elem in qs_des]
        #Find nearest q_des (sorted by joints)
        best_q_diff = qs_diff[0]
        best_idx = 0
        for k, q_diff in enumerate(qs_diff):
            if k==0:
                continue
            if q_diff[0] < best_q_diff[0]:
                best_q_diff = q_diff
                best_idx = k
            elif q_diff[0] == best_q_diff[0]:
                if q_diff[1] < best_q_diff[1]:
                    best_q_diff = q_diff
                    best_idx = k
                elif q_diff[1] == best_q_diff[1]:
                    if q_diff[2] < best_q_diff[2]:
                        best_q_diff = q_diff
                        best_idx = k
                    elif q_diff[2] == best_q_diff[2]:
                        if q_diff[3] < best_q_diff[3]:
                            best_q_diff = q_diff
                            best_idx = k
                        elif q_diff[3] == best_q_diff[3]:
                            if q_diff[4] < best_q_diff[4]:
                                best_q_diff = q_diff
                                best_idx = k
                            elif q_diff[4] == best_q_diff[4]:
                                if q_diff[5] < best_q_diff[5]:
                                    best_q_diff = q_diff
                                    best_idx = k
        q_des = qs_des[best_idx]
            
        if self._planner_interface.has_matlab:    
            joint_traj = self._planner_interface.get_joint_trajectory_to_joint_configuration(q_des)
            if len(joint_traj)==0:
                rospy.loginfo("Could not find joint trajectory with planner that adheres to joint limits")
                return
        self.execute_joint_trajectory(joint_traj=joint_traj, wait_for_feedback=True)
        rospy.loginfo("re-/deactivate controllers")
        self._joint_traj_handle.deactivate()
        self.reactivate_controllers(cur_controllers)

    def update_tf(self) -> None:
        """update robot state,i.e.: update ee-pose, update F/T offset and send current TCP if not static"""
        try:
            self._robot.update_tf()
        except AttributeError:
            pass
        try:
            #self.FT.read_params() # :(
            self._ft_handle.update_load(
                quaternion.as_rotation_matrix(self.sns_quat.conjugate()) @ self.R_FT_E.T
            )
        except AttributeError:
            pass

    @property
    def needs_reset(self) -> bool:
        """check robot state vs. controller state."""
        if self._robot.robot_status == "canceling":
            for t in range(2000):
                if self._robot.robot_status != "canceling":
                    break
                rospy.sleep(1e-3)
        try:
            if self._sns_trk_vel_handle.active or self._sns_trk_compl_handle.active:
                return self._robot.robot_status not in ("moving", "ready")
        except AttributeError:
            pass
        try:
            if self._joint_traj_handle.active:
                return self._robot.robot_status == "terminate"
        except AttributeError:
            pass
        return False

    def reset_sns_reference_pose(self) -> None:
        """
        Reset Sensor-tracking reference pose by commanding the robot to move to the current
        joint pose
        """
        assert self._joint_traj_handle is not None, "need to instantiate joint trajectory handle first"
        assert self._robot is not None, "need to instantiate robot handle first"
        assert not np.all(
            self.q == 0.0), "current joint position must not be 0!"
        self._robot.update_tf()
        self.move_to_joint_pose(self.q)

    def reset_sns_service(self, _) -> TriggerResponse:
        """
        Reset Sensor-tracking reference pose as internal deviations are limited
        in absolute magnitude.
        Reset current pose to internal relative pose to rest sensor track deviation distance
        """
        msg = ""
        if self._joint_traj_handle is not None:
            msg = "cannot reset sensor-track reference pose without joint trajectory handle"
        if not self.kinematic_consistent:
            msg = "robot kinematics are inconsistent -> stop sensor-track reset"
        if len(msg) > 1:
            rospy.logerr(msg)
            return TriggerResponse(False, msg)
        self.reset_sns_reference_pose()
        return TriggerResponse(True, "reset sensor tracking controller reference pose")

    @property
    def can_calibrate(self) -> bool:
        """
        Checks if current robot pose is valid for calibration

        Returns:
            bool: True if current pose is (hopefully) valid for calibration
        """
        return self._robot.can_calibrate

    def move_to_calibration_pose(self, wait_for_feedback=True) -> bool:
        """Move to calibration pose

        Returns:
            bool: True if current pose is sufficiently close to calibration pose
        """
        assert self._robot is not None, "cannot read proper robot data for moving it to calibration pose"
        assert self._joint_traj_handle is not None, "cannot control joints with current cobot handle"
        self.move_to_joint_pose(self._robot.q_calib)
        return np.linalg.norm(self.q - self._robot.q_calib) <= 1e-3

    def publish_rpm(self, rpm=0) -> None:
        """
        Method to publish RPM for shaft grinder. Needs to be turned on using the enable_grinder method after.

        Args:
            rpm (float) : Sets RPM for shaft grinder. Must be in range [3500.0, 25000.0].
                            Values below 3500 are set to 0, values above 25000 are cut to 25000.
                            Defaults to 0 for safety reasons.

        """
        assert tool2tool_type(self.tool) == ToolType.SHAFT_GRINDER, \
            "current tool is not set to shaft-grinder"
        try:
            self._tool_controller.rpm = rpm
        except AttributeError as e:
            rospy.logerr_once(f"cannot set rpm of shaft grinder due to {e}")

    def set_tool_frame(self, tool_type, tool_name=None, robot_urdf_prefix="hrr_cobot.") -> None:
        """Set tool-frame, i.e. set current TCP-frame for given ee-tool.

        Args:
            tool_type(int): tool-id from e.g. ROS-message
            tool_name(str or int, optional): optional tool_name. Defaults to None.
            robot_urdf_prefix(str, optional): URDF-prefix to select current link-name for new TCP. Defaults to ``hrr_cobot.``
        """
        tool_frame_name = self.ee_link
        if tool_type in (ToolType.WSG_50, ToolType.WSG_50_DSA):
            tool_frame_name = f"{robot_urdf_prefix}wsg_50_tcp"
        elif tool_type == ToolType.SHAFT_GRINDER:
            tool_frame_name = f"{robot_urdf_prefix}shaftgrinder_tip"
        elif tool_type == ToolType.SCREW_DRIVER:
            tool_frame_name = f"{robot_urdf_prefix}screwdriver_tip"
        elif tool_type == ToolType.VACUUM_GRIPPER:
            rospy.logwarn_once("vacuum gripper contains two tips. This contradicts default pipeline atm. Use 1")
            tool_frame_name = f"{robot_urdf_prefix}vacuum_tip_2"
        if tool_name is None:
            tool_name = tool_type2str(tool_type)
        self.set_tool_from_ros_frame(name=tool_name, tool_frame_id=tool_frame_name)

    def change_tool(self, tool_name: str, delay=1.0, robot_urdf_prefix="hrr_cobot.") -> bool:
        """Change the current tool based on a new name

        Args:
            tool_name (str): name of new ee-tool.
            delay (float, optional): time delay between publish and tool update in seconds. Defaults to 1.0.
            robot_urdf_prefix (str, optional): prefix for URDF, i.e. for ``tool_frame_id``. Defaults to "".

        Returns:
            bool: True if tool has been successfully set to desired value
        """
        try:
            tool_type = tool2tool_type(tool_name)
        except (AssertionError, ValueError, TypeError) as e:
            rospy.logerr(f"cannot set tool to {tool_name} due to {e}")
            return False
        if tool_name == tool2tool_type(self.tool):
            rospy.loginfo(f"tool {tool_name} is already set.")
            self.set_tool_frame(tool_type, robot_urdf_prefix=robot_urdf_prefix, tool_name=tool_name)
            rospy.sleep(2)
            self.update_ee_tcp_tf()
            return True
        tool_name = tool_type2str(tool_type)
        self._pub_tool.publish(ToolType(type=tool_type))
        rospy.sleep(delay)
        self.set_tool_frame(tool_type, robot_urdf_prefix=robot_urdf_prefix, tool_name=tool_name)
        rospy.sleep(2)
        self.update_ee_tcp_tf()
        self.update()
        self.FT.read_params()
        self.update()
        return self.tool == tool_name

    def _check_for_tool_check(self) -> bool:
        """Check current robot configuration to allow for a tool-change

        Raises:
            RuntimeError: if current control interface is incorrect
        """
        if isinstance(self._tool_controller, ArduinoToolControlInterface):
            raise RuntimeError("Current control interface is Arduino, which has no interface for tool-changer")
        return True

    def open_tool_changer(self, force=False) -> None:
        """pose-aware call to tool-changer opening

        Args:
            force(bool, optional): If True, tool-changer is opened independent of current pose. Defaults to False
        """
        from comau_msgs.srv import SetIO, SetIORequest
        srv = rospy.ServiceProxy("/hrr_cobot/set_digital_io", SetIO)
        srv(SetIORequest(pin=rospy.get_param("/hrr_cobot/toolchanger_dout_pin"), state=True))
        #if force or self._check_for_tool_check():
        #    self._tool_controller.open_tool_changer()
        ##else:
        #    rospy.logerr(f"cannot trigger tool-changer at {self.T_B_E_robot}")

    def close_tool_changer(self, force=False) -> None:
        """pose-aware call to tool-changer closing

        Args:
            force(bool, optional): If True, tool-changer is opened independent of current pose. Defaults to False
        """
        from comau_msgs.srv import SetIO, SetIORequest
        srv = rospy.ServiceProxy("/hrr_cobot/set_digital_io", SetIO)
        srv(SetIORequest(pin=rospy.get_param("/hrr_cobot/toolchanger_dout_pin"), state=False))
        #if force or self._check_for_tool_check():
        #    self._tool_controller.close_tool_changer()
        #else:
        #    rospy.logerr(f"cannot trigger tool-changer at {self.T_B_E_robot}")

    def joint_limit_distance(self):
        d_lb = self.q - self.joint_limits[:, 0]
        d_ub = self.joint_limits[:, 1] - self.q
        d = np.minimum(d_lb, d_ub)
        d[np.where(d == d_lb)] *= -1
        return d

    def joint_limit_avoidance_needed(self, d_limit=None, dq=None):
        if d_limit is None:
            d_limit = self._joint_limits_hard_dist
        d_limit = np.abs(d_limit)
        if dq is None:
            dq = self.joint_limit_distance()
        flags = np.zeros(len(self.q))
        idx = np.where(np.logical_or(np.logical_and(dq < 0, dq > -d_limit), np.logical_and(dq > 0, dq < d_limit)))
        flags[idx] = dq[idx]
        return flags

    def is_reachable(self, T_B_E_des, check_closest_only=True, log=True):
        """
        Check if an EE-pose is reachable for the robot.


        Args:
            T_B_E_des(sm.SE3): EE-pose to be checked
            check_closest_only(bool, optional): check closest joint configuration only
            log(bool, optional): flag to enable ROS-logging, mainly warning / errors

        Returns:
            bool: True, if pose is reachable for robot.
        """
        try:
            q_ik = self._robot.IK(T_B_E_des)
            q_ik = q_ik[np.argsort(np.linalg.norm(q_ik - self.q, axis=1))]
        except (IndexError, ValueError):
            if log:
                rospy.logerr(f'no solution found for pose:\n{T_B_E_des}')
            return False
        lb_check = q_ik >= (self.joint_limits[:, 0] + self._joint_limits_hard_dist)
        ub_check = q_ik <= (self.joint_limits[:, 1] - self._joint_limits_hard_dist)
        valid = np.logical_and(lb_check, ub_check)
        if np.all(valid):
            return True
        res = np.any(np.all(valid, axis=1))
        if not res:
            return False
        if check_closest_only:
            return np.all(valid[0, :])
        jnts = [q_i + 1 for q_i in np.where(np.any(~valid, axis=0))[0]]
        if log:
            rospy.logwarn(f"IK contains invalid configurations for joint(s): {jnts}")
        return res

    def is_tcp_reachable(self, T_B_C_des):
        """check reachability given the goal pose of the tcp of the robot, c.f. :py:meth:`~is_reachable`"""
        return self.is_reachable(T_B_C_des @ self.T_E_C_robot.inv())

    def get_valid_ee_pose(self, B_p_des, B_normal=np.r_[0., 0., 1.], N=200,
                          B_y_test=np.r_[0., 1., 0.]) -> sm.SE3 or None:
        """
        Calculate valid EE-pose via exploiting the rotary symmetry around the normal vector.

        Args:
            B_p_des(np.ndarray): desired tcp location
            B_normal(np.ndarray, optional): normal vector in base frame. Defaults to z-axis
            N(int, optional): number of samples along rotary axis
            B_y_test(np.ndarray): reference x-axis to generate y-axis with

        Returns:
            sm.SE3: desired EE-pose w.r.t. base link.
        """
        R_tmp = sm.base.oa2r(B_y_test, -B_normal)
        y_init = R_tmp[:3, 1]
        for psi in np.linspace(0, 360, N):
            T_B_C_des = hrr_common.calc_goal_pose(normal=B_normal,
                                                  y_axis=sm.SE3.AngleAxis(psi, v=B_normal, unit="deg") * y_init,
                                                  p_location=B_p_des)
            T_B_E_des = T_B_C_des @ self.T_E_C_robot.inv()
            if self.is_reachable(T_B_E_des, log=False):
                return T_B_E_des
        rospy.logerr("could not find a reachable solution")
