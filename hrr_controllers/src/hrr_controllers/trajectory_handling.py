#!/usr/bin/env python
"""
COMAU Trajectory Control Utils
--------------------------------

python-handle for the ``joint_trajectory_controller.cpp`` from comau-controllers package.
This file contains a collection of helpers that are used to generate and send a
(joint / Cartesian) trajectory to the Racer 5 Cobot.

Unless the robot is intended to be controlled via fully defined joint / Cartesian
trajectories, the majority of these functions can be ignored.

Note:
    * The Cartesian trajectory control has been removed from this package. Please refer to the sensor-tracking interfaces at  :py:mod:`hrr_cobot_robot.ros_interfaces.sensor_track_commander`
    * this module contains additional functions which are currently not in use and are thus excluded from the docu. In order to change this, append the function names to the ```__all__``` list.

"""
import numpy as np

import actionlib
from comau_msgs.msg import ExecuteJointTrajectoryAction, ExecuteJointTrajectoryGoal
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from hrr_common.ros_utils.templates import BaseController

__all__ = ["JointTrajectoryHandler"]


def send_trajectory(cl, goal, wait_for_feedback=True):
    """Helper to call trajectory action client.

    Args:
        cl(actionlib.SimpleActionClient): action-client to be called
        goal(object): goal for current action service
        wait_for_feedback(bool): flag to wait for return value. Defaults to False.

    Returns:
        None or object: optional action-client feedback if desired
    """
    cl.send_goal(goal)
    if wait_for_feedback:
        cl.wait_for_result()
        return cl.get_result()


def pt2str(p):
    """`
    print helper for a Joint Trajectory Point

    Args:
        p(JointTrajectoryPoint): single trajectory point / vector in rad

    Returns:
        str: human readable joint vector in degree
    """
    inner = "\t".join(list(map(lambda x: '{:.2f}'.format(x), np.rad2deg(p.positions))))
    return f"[{inner}] [°]"


def print_joint_trajectory_goal(g):
    """
    Print helper for a joint trajectory goal.
    Assumes that ``g`` has a ``trajectory`` member.

    Args:
        g(ExecuteJointTrajectoryGoal): trajectory goal

    Returns:
        str: human-readable string of current joint-goal
    """
    print("\n".join(list(map(pt2str, g.trajectory))))


def generate_joint_traj_points(q_des, q_dot_des=None, q_ddot_des=None, q_tau_des=None, time_resolution=None):
    r"""
    Helper to generate a joint trajectory, given as a list of `JointTrajectoryPoint``

    Args:
        q_des(np.ndarray or list): list of joint waypoints
        q_dot_des(np.ndarray or list or None): optional list of joint velocities
        q_ddot_des(np.ndarray or list or None): optional list of joint accelerations
        q_tau_des(np.ndarray or list or None): optional list of joint torques
        time_resolution(np.ndarray or list or None): optional time samples across trajectory

    Returns:
        jnt_traj(List[]): joint trajectory
    """

    jnt_traj = []
    # if time_resolution is None:
    #     time_delta = 0.005 * np.linspace(1, len(q_des), len(q_des))
    for i, q in enumerate(q_des):
        pt_kwargs = dict(positions=q.tolist())
        if q_dot_des is not None:
            pt_kwargs.update(dict(velocities=q_dot_des[i].tolist()))
        if q_ddot_des is not None:
            pt_kwargs.update(dict(accelerations=q_ddot_des[i].tolist()))
        if q_tau_des is not None:
            pt_kwargs.update(dict(effort=q_tau_des[i].tolist()))
        jnt_traj.append(JointTrajectoryPoint(**pt_kwargs))
    return jnt_traj


def poseStamped2str(p):
    """
    print helper for a Joint Trajectory Point

    Args:
        p(Cart): single trajectory point / vector in rad

    Returns:
        str: human readable joint vector in degree
    """
    rpy = np.rad2deg(np.array((p.roll, p.pitch, p.yaw)))
    return f"p:\t[{p.x},{p.y},{p.z}][m]\trpy:\t{rpy}[°] @ {p.header.frame_id}"


def print_cart_trajectory_goal(g):
    """
    Print helper for a cartesian trajectory goal.
    Assumes that ``g`` has a ``trajectory`` member.

    Args:
        g(ExecuteJointTrajectoryGoal): trajectory goal

    Returns:
        str: human readable string of current joint-goal
    """
    print("\n".join(list(map(poseStamped2str, g.trajectory))))


class JointTrajectoryHandler(BaseController):
    """This class communicates with the ``JointTrajectoryHandler``
    It expects the following ROS-parameters to be set

    * ``~joint_trajectory_action_topic_name`` which defines the action topic name to send joint trajectories to
    * ``~joint_trajectory_handler_name`` which defines the name of the joint-trajectory handler at the controller manager

    If activated, this class allows to load a trajectory
    """

    def __init__(self):
        super().__init__()
        self._q = None
        self._dof = 6
        self._jnt_traj_client = None

    def init_ros(self, action_topic, cobot_prefix, **__):
        super(JointTrajectoryHandler, self).init_ros(cobot_prefix=cobot_prefix, **__)
        self.controller_name = rospy.get_param(f"{cobot_prefix}joint_trajectory_handler_name")
        self._dof = rospy.get_param(f"{cobot_prefix}dof", self._dof)
        self._jnt_traj_client = actionlib.SimpleActionClient(action_topic, ExecuteJointTrajectoryAction)
        self._jnt_traj_client.wait_for_server()

    @classmethod
    def _from_ros(cls, cobot_prefix, **_):
        out = cls()
        out.init_ros(rospy.get_param(f"{cobot_prefix}joint_trajectory_action_topic_name"), cobot_prefix=cobot_prefix,
                     **_)
        return out

    def linear_joint_interpolate(self, q0, q_des, wps=40):
        assert q0.shape == q_des.shape == (self._dof,), "dimension mismatch"
        assert not np.all(q0 == 0.0), "current joint values are all 0, assuming data is missing. Cancel motion"
        jnt_trajectory = np.repeat([np.copy(q0)], 2 * wps, axis=0)
        for i in range(6):
            jnt_trajectory[:, i] = np.linspace(q0[i], q_des[i], 2 * wps)
        return jnt_trajectory

    def linear_joint_cmd(self, q0, q_des, wps=40, wait_for_feedback=True, print_traj=False):
        """
        Simplest joint motion from current joint to joint goal using a linear interpolation

        Args:
            q0(np.ndarray): current joint position
            q_des(np.ndarray): joint goal
            wps(int, optional): numbers of waypoints for linear interpolation. Defaults to 40.
            wait_for_feedback(bool, optional): wait for feedback for joint-trajectory action-client
            print_traj(bool, optional): flag to print trajectory before sending
        """
        try:
            jnt_trajectory = self.linear_joint_interpolate(q0, q_des, wps)
        except AssertionError:
            return
        self.command_joint_trajectory(jnt_trajectory, wait_for_feedback=wait_for_feedback, print_traj=print_traj)

    def np2jointGoal(self, arr, print_traj=False) -> ExecuteJointTrajectoryGoal:
        """
        Convert array of joints to Joint-Trajectory Goal for action server to send

        Args:
            arr(np.ndarray): joint array for :math:`T` trajectory samples (T x d)
            print_traj(optional, bool): print trajectory is True. Defaults to False.

        Returns:
            ExecuteJointTrajectoryGoal: parsed joint trajectory
        """
        assert arr.shape[1] == self._dof, f"dimension mismatch for trajectory of shape {arr.shape} vs. DoF:{self._dof}"
        q_t = ExecuteJointTrajectoryGoal(generate_joint_traj_points(arr))
        if print_traj:
            rospy.loginfo("generated trajectory:\n%s", "\n".join(
                list(map(pt2str, q_t.trajectory))))
        return q_t

    def assert_active(self):
        if not self.active:
            self.activate()
            rospy.logdebug('wait for initialization to be finished')
            rospy.sleep(1.0)
        assert self.active, "cannot command joint trajectory to robot-> controller is inactive"

    def command_joint_trajectory(self, arr, wait_for_feedback=True, print_traj=False):
        """
        Send a trajectory to the robot.

        Warn:
            This function does not check for zeros! so it may raise the robot to straight up positions
            Neither any other collisions nor safety checks are included here.
            So use this function with care!

        Args:
            arr(np.ndarray): joint array for :math:`T` trajectory samples (T x d)
            wait_for_feedback(bool, optional): flag to wait for feedback from the joint trajectory action
            print_traj(bool, optional): flag to print trajectory before sending
        """
        try:
            self.assert_active()
            send_trajectory(self._jnt_traj_client, self.np2jointGoal(arr, print_traj=print_traj),
                            wait_for_feedback=wait_for_feedback)
        except AssertionError as e:
            rospy.logerr(f"could not send joint-trajectory due to: {e}")
            return

    def command_joint_trajectory_msgs(self, traj_msg, wait_for_feedback=True):
        """
        command a trajectory message to the robot

        Args:
            traj_msg(JointTrajectory): desired trajectory as joint trajectory message
            wait_for_feedback:
        """
        try:
            self.assert_active()
            send_trajectory(self._jnt_traj_client, ExecuteJointTrajectoryGoal(trajectory=traj_msg.points),
                            wait_for_feedback=wait_for_feedback)
        except AssertionError as e:
            rospy.logerr(f"could not send joint-trajectory due to: {e}")
            return

    def deactivate(self):
        self._jnt_traj_client.cancel_all_goals()
        super().deactivate()

