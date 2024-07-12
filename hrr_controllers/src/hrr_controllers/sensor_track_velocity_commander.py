#!/usr/bin/env python3
"""
.. _SNS_TRK_VEL_CMD:

Sensor Track Velocity Control Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Python-handle for the ``sns_trk_velocity_controller.cpp``
Allows sending `Cartesian` `velocities` to a COMAU robot.
"""
from socket import TCP_NODELAY
from typing import Optional
import numpy as np

import rospy
from hrr_msgs.msg import SnsTrkTwistCmd

from geometry_msgs.msg import TwistStamped

from hrr_common.ros_utils import np2vec3, np2twist_stamped
from hrr_common.ros_utils.templates import BaseController

__all__ = ["SnsTrkCmd"]


def visualize_cmd(cmd, frame_id="base_link"):
    """
    Generate a visualization message from current command

    Args:
        cmd (TwistStamped): Sensor Tracking Command to be sent to robot
        frame_id(str): reference frame for twist command

    Returns:
        TwistStamped: Visualization message
    """

    def _vector_normalizer(vec):
        norm = np.sqrt((vec.x ** 2 + vec.y ** 2 + vec.z ** 2))
        if norm == 0.0:
            return 1.0
        return norm

    vis_twist = TwistStamped(header=rospy.Header(
        stamp=rospy.Time.now(), frame_id=frame_id))
    normalizer = _vector_normalizer(cmd.twist.linear)
    vis_twist.twist.linear.x = cmd.twist.linear.x / normalizer
    vis_twist.twist.linear.y = cmd.twist.linear.y / normalizer
    vis_twist.twist.linear.z = cmd.twist.linear.z / normalizer
    normalizer = _vector_normalizer(cmd.twist.angular)
    vis_twist.twist.angular.x = cmd.twist.angular.x / normalizer
    vis_twist.twist.angular.y = cmd.twist.angular.y / normalizer
    vis_twist.twist.angular.z = cmd.twist.angular.z / normalizer
    return vis_twist


class SnsTrkCmd(BaseController):
    """
    Basic Sensor tracking handle that is used to generate commands for robot in sns-trk mode
    """

    def __init__(self):
        super().__init__()
        self._center = np.zeros(3)
        self._pub_sns_trk_twist = None  # type: Optional[rospy.Publisher]
        self._pub_sns_frame = None  # type: Optional[rospy.Publisher]
        self._viz_pub = None  # type: Optional[rospy.Publisher]

    def init_ros(self, sns_trk_twist_topic_name="/hrr_cobot/sensor_track_velocity_controller/x_dot_des",
                 gm_topic_name="/hrr_cobot/sensor_track_velocity_controller/tcp_v_cmd",
                 viz_topic="visualize_twist_cmd", *_, cobot_prefix, **__):
        super().init_ros(*_, cobot_prefix=cobot_prefix, **__)
        self.controller_name = rospy.get_param(f"{cobot_prefix}sns_trk_vel_controller_name")
        self._center[0] = rospy.get_param(f"{cobot_prefix}center/x", 0.0)
        self._center[1] = rospy.get_param(f"{cobot_prefix}center/y", 0.0)
        self._center[2] = rospy.get_param(f"{cobot_prefix}center/z", 0.0)
        if sns_trk_twist_topic_name:
            self._pub_sns_trk_twist = rospy.Publisher(sns_trk_twist_topic_name, SnsTrkTwistCmd, queue_size=10, tcp_nodelay = True)
        if gm_topic_name:
            self._pub_sns_frame = rospy.Publisher(gm_topic_name, TwistStamped, queue_size=10, tcp_nodelay = True)
        if viz_topic:
            self._viz_pub = rospy.Publisher(viz_topic, TwistStamped, queue_size=10)

    @classmethod
    def _from_ros(cls, cobot_prefix, **_):
        out = cls()
        out.init_ros(rospy.get_param(f"{cobot_prefix}sns_trk_twist_topic_name", ""),
                     rospy.get_param(f"{cobot_prefix}sns_trk_topic_name"),
                     rospy.get_param(f"{cobot_prefix}visualize_topic_name", ""),
                     cobot_prefix=cobot_prefix)
        return out

    def publish_verbose(self, cmd, viz_cmd):
        """Publish command to robot and visualize command in Rviz

        Args:
            cmd (SnsTrkTwistCmd): command to be published
            viz_cmd(TwistStamped): visualization command to be published
        """
        assert self._pub_sns_trk_twist is not None, "sensor track interface is not properly initialized"
        self._pub_sns_trk_twist.publish(cmd)
        if self._viz_pub is not None:
            self._viz_pub.publish(viz_cmd)

    def publish_in_C(self, C_v_des: np.ndarray, frame_id=""):
        assert self._pub_sns_frame is not None, "sensor track interface is not properly initialized"
        self._pub_sns_frame.publish(np2twist_stamped(C_v_des, frame_id))

    @property
    def zero_cmd(self):
        return SnsTrkTwistCmd(center=np2vec3(self._center))

    def get_twist_cmd(self, v_des, omega_des=None, publish=False):
        """generate twist commands for robot and rviz

        Note:
            ! Deprecated -> use :py:meth:`~update_cmd` instead

        Generates a twist command for sensor-tracking controller and a
        ``geometry_msgs.TwistStamped`` message that amplifies current output to unit vector
        for visualization in rviz.

        Args:
            v_des (np.ndarray): desired velocity vector
            omega_des (np.ndarray, optional): desired rotation velocity vector. Defaults to None.
            publish(np.ndarray, optional): flag to publish data to robot. Defaults to False.

        Returns:
            tuple: sensor-track twist command and visualization message
        """
        if omega_des is None:
            omega_des = np.zeros(3)
        twist_cmd = np2twist_stamped(np.r_[v_des, omega_des], "")
        viz_cmd = visualize_cmd(twist_cmd)
        if publish:
            self.publish_verbose(twist_cmd, viz_cmd)
        return twist_cmd, viz_cmd

    def update_cmd(self, x_dot_des=np.zeros(6), frame=""):
        """
        Send command to robot

        Args:
            x_dot_des (np.ndarray(), optional): desired. Defaults to np.zeros(6).
            frame(str, optional): reference frame. Defaults to None.
        """
        try:
            self._pub_sns_frame.publish(np2twist_stamped(x_dot_des, frame))
        except AttributeError:
            rospy.logerr(
                "cannot forward command to robot as current sensor-track interface isn't properly initialized!")

    def stop(self):
        """Send zero-velocity to the robot and thus set it to halt"""
        self.update_cmd(np.zeros(6))

