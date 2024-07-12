#!/usr/bin/env python3
"""
Robot State Interfaces Handles
------------------------------------

This file wraps simple ROS-interface for joint-controlled robots and 6-DoF COMAU robot.

* :py:class:`~RobotState` subscribes to joint states and optionally adds forward-kinematics handler
* :py:class:`~ComauRobotState` extends the former by additionally subscribing to tf2-ros and TP5 state messages

The dedicated ROS-control interfaces are given as

- JointStateController from ``ros_controllers``
- ``tp5_state_controller.cpp``

Example usage is found in the first tutorial-notebook in the ``hrr_cobot`` package about reading robot data.

Required / Expected ROS-parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``prefix`` usually set to current cobot-namespace, i.e.  "/hrr_cobot/". 
Thus, the table below replace ``prefix`` by ``/hrr_cobot/``

==========================  ======================================  ==================================
attribute                   ROS-parameter                           default
==========================  ======================================  ==================================
``joint_state_topic_name``  ``/hrr_cobot/joint_state_topic_name`` 
``joint_names``             ``/hrr_cobot/joints``                   [``joint_1``, ``joint_2``, .. ]
==========================  ======================================  ==================================

For the :py:class:`~ComauRobotState` handle, the following additional parameters
are read / needed

======================================  ==========================================  ==================================
attribute                               ROS-parameter                               default
======================================  ==========================================  ==================================
only needed to extract TP5 topic name   ``/hrr_cobot/comau_state_controller_name``
``base_frame``                          ``/hrr_cobot/base_link_name``               ``base_link``
``sns_frame``                           ``/hrr_cobot/sns_frame``                    ``tcp_controller``
sets robot pose handle from FK or tf    ``/hrr_cobot/use_tf_for_TBE``                True
======================================  ==========================================  ==================================

"""
# built in modules
import sys
import warnings

# external modules
import numpy as np
import spatialmath as sm
import quaternion
from typing import Optional
from scipy.signal import savgol_filter

# ros imports
import rospy
from comau_msgs.msg import TP5State
from sensor_msgs.msg import JointState

# module imports
from hrr_common.utils import pose_error
from hrr_common.ros_utils.transformations import TfHandler
from hrr_common.ros_utils.templates import RosBaseHandle

__all__ = ["RobotState", "ComauRobotState"]

if float(sys.version[:3]) < 3.6:
    warnings.warn("please update python version (>=3.6) for full functionality. Printing require f-strings")


class RobotState(RosBaseHandle):
    """
    Default ROS Robot State
    subscribes to a ``sensor_msgs/JointState`` ROS message and adds to data instance
    
    Optionally allows to use a savitzky-golay filter on the joints if data is extremly noisy.
    this is only applied if the buffer size is larger than 1.

    Args:
        joint_names(list): list of joint names as string as named in URDF.
        buf_size(int, optional): buffer size for joint messages. Defaults to 1.
    """

    def __init__(self, joint_names=("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"),
                 buf_size=1, **_):
        super().__init__()
        self.joint_names = joint_names
        dof = len(self.joint_names)
        if buf_size > 1:
            self._q = np.zeros((dof, buf_size))
        else:
            self._q = np.zeros(dof)
        self.q_dot = np.zeros(dof)
        self.q_tau = np.zeros(dof)
        self.filter_kwargs = dict(window_length=min(buf_size, 11),
                                  polyorder=2)
        # ROS - handles
        self._q_sub = None
        self._kin_model = None  # type: Optional[object]
        self._kin_model_compiled = False

    def init_ros(self, joint_state_topic_name, *args, **kwargs):
        self._q_sub = rospy.Subscriber(joint_state_topic_name, JointState, self.from_state_msg)

    @classmethod
    def _from_ros(cls, cobot_prefix, **_):
        out = cls()
        out.init_ros(rospy.get_param(f"{cobot_prefix}joint_state_topic_name"))
        out.joint_names = rospy.get_param(f"{cobot_prefix}joint_names", out.joint_names)

        return out

    def from_state_msg(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            try:
                if len(self._q.shape) > 1:
                    self._q = np.roll(self._q, axis=1, shift=1)
                    self._q[self.joint_names.index(name), 0] = msg.position[i]
                else:
                    self._q[self.joint_names.index(name)] = msg.position[i]
                self.q_dot[self.joint_names.index(name)] = msg.velocity[i]
                self.q_tau[self.joint_names.index(name)] = msg.effort[i]
            except (ValueError, KeyError, IndexError):
                pass

    @property
    def q_deg(self):
        """Return joint configuration in degrees"""
        return np.rad2deg(self.q)

    def savgol_filtered_joints(self, **filter_kwargs):
        """filter joints if stored in buffer according to scipy->signal->savgol-filter"""
        if len(self._q.shape) > 1:
            return np.array([savgol_filter(x, **filter_kwargs) for x in self._q])
        return self._q

    @property
    def q(self):
        """ current joint configuration. if joints are stored in buffer, the value is
        returned via ``savitzky-golay filter``, i.e. :py:meth:`~savgol_filtered_joints`"""
        if len(self._q.shape) > 1:
            return self.savgol_filtered_joints(**self.filter_kwargs)[:, 0]
        return self._q.copy()

    def get_model(self, compiled=True, verbose=True):
        if self._kin_model is None:
            from sim_robots import get_racer_kin_model
            self._kin_model = get_racer_kin_model()
        if not self._kin_model_compiled and compiled:
            if verbose:
                rospy.loginfo(f"compiled model within {self._kin_model.compile(timed=True):.2f} [s]")
            else:
                rospy.logdebug(f"compiled model within {self._kin_model.compile(timed=True):.2f} [s]")
            self._kin_model_compiled = True
        return self._kin_model

    def FK(self, q):
        r"""
        Forward Kinematic Handle, forwarding to the kinematic model provided by
        :py:func:`sim_robots.get_racer_kin_model` function

        Args:
            q(np.ndarray): joint vector
        Returns:
            sm.SE3: :math:`{}^{E}{\bf T}_{B}`
        """
        return self.get_model(False, False).forward_kin(q)

    def IK(self, T_E_B, **kwargs):
        r"""
        Inverse Kinematic Handle, forwarding to the kinematic model provided by
        :py:func:`sim_robots.get_racer_kin_model` function

        Args:
            T_E_B(sm.SE3 or np.ndarray): Transformation from base to ee :math:`{}^{E}{\bf T}_{B}`
        Returns:
            np.ndarray: possible joint values
        """
        out = self.get_model(False, False).IK(T_E_B, check=True, **kwargs)
        try:
            _ = out[0]
            return out
        except TypeError:
            return np.array(list(out))
        except IndexError:
            return np.array([])


    def J(self, q, base=0, target=None):
        return self.get_model(False, False).jacobian(q, base=base, target=target)

    @property
    def joint_limits(self):
        return self.get_model(False, False).joint_limits

    def __str__(self):
        return f"q:\t\t{self.q_deg}[°]\nq_dot:\t\t{self.q_dot}[rad/s]\ntau:\t\t{self.q_tau}[Nm]"


class ComauRobotState(RobotState):
    """
    Comau Robot State handle. Extends Robot State handle with additional TF broadcaster handle

    Args:
        tool_frame (str): tool frame ID
        base_frame (str, optional): base frame ID. Defaults to "base_link".
        dof (int, optional): Degrees of Freedom. Defaults to 6.
    """

    def __init__(self, tool_frame="tcp_controller", base_frame="base_link",
                 use_tf_for_TBE=False,
                 joint_names=("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"),
                 **kwargs):
        super(ComauRobotState, self).__init__(joint_names=joint_names, **kwargs)
        self.q_calib = np.r_[0., 0., -np.pi / 2.0, 0., np.pi / 2.0, 0.0]
        self._base_frame = base_frame
        self._sns_frame = tool_frame
        self._quat = np.quaternion(1., 0., 0., 0, )
        self._R = np.zeros((3, 3))
        self._p = np.zeros(3)
        self._tf = TfHandler(add=False)
        self.robot_status = "idle"
        self.tool_msg_frame = ""
        self.tool_pos = np.zeros(3)
        self.tool_rpy = np.zeros(3)
        self._r_sub = None
        self._T_B_E_handle = self.T_B_E_from_tf if use_tf_for_TBE else self.T_B_E_from_fk

    def init_ros(self, joint_state_topic_name, comau_robot_status_topic, *args, **kwargs):
        super(ComauRobotState, self).init_ros(joint_state_topic_name)
        self._r_sub = rospy.Subscriber(comau_robot_status_topic, TP5State, self._tp5_state_msg_cb)

    @classmethod
    def _from_ros(cls, cobot_prefix, **_):
        out = cls()
        out.joint_names = rospy.get_param(f"{cobot_prefix}joint_names", out.joint_names)
        out._sns_frame = rospy.get_param(f"{cobot_prefix}sns_frame", out._sns_frame)
        out._base_frame = rospy.get_param(f"{cobot_prefix}base_link_name", out._base_frame)
        tp5_topic_name = f"{cobot_prefix}{rospy.get_param(f'{cobot_prefix}comau_state_controller_name')}/comau_cartesian_data"
        out.init_ros(rospy.get_param(f"{cobot_prefix}joint_state_topic_name"), tp5_topic_name)
        use_tf_for_TBE = rospy.get_param(f"{cobot_prefix}use_tf_for_TBE", False)
        out._T_B_E_handle = out.T_B_E_from_tf if use_tf_for_TBE else out.T_B_E_from_fk
        out.update_tf()
        return out

    def _tp5_state_msg_cb(self, msg):
        self.robot_status = msg.status.status
        self.tool_msg_frame = msg.tool_pose.frame
        self.tool_pos[0] = msg.tool_pose.x
        self.tool_pos[1] = msg.tool_pose.y
        self.tool_pos[2] = msg.tool_pose.z
        self.tool_rpy[0] = msg.tool_pose.roll
        self.tool_rpy[1] = msg.tool_pose.pitch
        self.tool_rpy[2] = msg.tool_pose.yaw

    def update_tf(self):
        self._p, self._quat = self._tf.T_A_B(A=self._base_frame, B=self._sns_frame, )

    def _assert_tf(self):
        if self._quat is None:
            self.update_tf()
            return self._assert_tf()

    @property
    def pos(self) -> np.ndarray:
        """ current EE-position according to tf2-ros"""
        self._assert_tf()
        return self._p

    @property
    def quat(self) -> np.quaternion:
        """ current EE-orientation according to tf2-ros"""
        self._assert_tf()
        return self._quat

    @property
    def R(self) -> np.ndarray:
        """ current EE-orientation as rotation matrix """
        return quaternion.as_rotation_matrix(self._quat)

    def T_B_E_from_tf(self) -> sm.SE3:
        """ current transformation from EE to Base using the ROS-TF"""
        self.update_tf()
        T = sm.SE3(self._p)
        T.A[:3, :3] = self.R
        return T

    def T_B_E_from_fk(self) -> sm.SE3:
        """current transformation from EE to Base using the FK"""
        return self.FK(self.q)

    @property
    def T_B_E(self) -> sm.SE3:
        """current ee-pose / FK-output
        returns  one of the following:
        
        - :py:meth:`~T_B_E_from_tf`
        - :py:meth:`~T_B_E_from_fk`
        """
        return self._T_B_E_handle()

    @property
    def can_calibrate(self) -> bool:
        """flag if robot can be calibrated, i.e. is sufficiently close (1mm in translation and 0.1 degree)
        to the calibration pose"""
        err = pose_error(self.FK(self.q), self.FK(self.q_calib))
        return np.linalg.norm(err[0:3]) < 1e-3 and np.rad2deg(np.linalg.norm(err[3:6])) < 0.1

    def __str__(self):
        self.update_tf()
        base_str = super().__str__()
        base_str += f"\npos:\t\t{self.pos}[m]"
        base_str += f"\nquat:\t\t{self.quat.w:.5f} <{self.quat.vec}>"
        base_str += f"\ntool-pos:\t{self.tool_pos}[m]"
        base_str += f"\ntool-rpy:\t{np.rad2deg(self.tool_rpy)}[°]"
        base_str += f"\nrobot-status:\t{self.robot_status}"
        return base_str

    @property
    def kinematic_consistent(self):
        """
        Returns True if position of FK(q) and sensor tracking state differ by not more than 1 mm.
        """
        return all((np.linalg.norm(self.q) != 0.0,
                    np.linalg.norm(self.FK(self.q).t - self.pos) < 1e-3))
