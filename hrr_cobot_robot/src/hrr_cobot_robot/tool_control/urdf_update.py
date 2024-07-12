#!/usr/bin/env python3
import os
import rospy
import subprocess

from hr_recycler_msgs.msg import ToolType

import hrr_common
from hrr_cobot_robot.utils import tool2tool_type

__all__ = ["URDFHandles"]


def get_robot_description(robot_description="/hrr_cobot/robot_description"):
    try:
        return rospy.get_param(robot_description)
    except KeyError:
        rospy.logerr(f"could not get ROS-parameter {robot_description}")
        return ""


def set_tool_name_from_msg(msg):
    # type: (ToolType) -> str
    """Get current tool-name from ROS-subscriber and update ROS-parameter"""
    tool_name = "nothing"
    if msg.type == msg.SCREW_DRIVER:
        tool_name = "screwdriver"
    elif msg.type == msg.SHAFT_GRINDER:
        tool_name = "shaftgrinder"
    elif msg.type == msg.VACUUM_GRIPPER:
        tool_name = "vacuum"
    elif msg.type == msg.WSG_50:
        tool_name = "wsg_50"
    elif msg.type == msg.WSG_50_DSA:
        tool_name = "wsg_50_dsa"
    return tool_name


def update_urdf(robot_name, tool_name, cobot_ns, tum_lab, urdf_prefix="hrr_cobot."):
    subprocess.Popen(["roslaunch", "hrr_common", "hrr_cobot.launch",
                      f"tool_name:={tool_name}", f"cobot_ns:={cobot_ns}",
                      f"tum_lab:={tum_lab}", f"robot_prefix:={urdf_prefix}",
                      f"robot_name:={robot_name}", "use_hw:=true", f"__ns:={cobot_ns}"],
                     stdout=subprocess.DEVNULL
                     )


def kill_ros_node(ros_node):
    rospy.logdebug("disable gripper driver")
    subprocess.Popen(["rosnode", "kill", ros_node], stdout=subprocess.DEVNULL)


class URDFHandles:
    p_joint_state_publisher = None
    tool_name = ""

    def __init__(self, tool_name,
                 cobot_ns="/hrr_cobot", robot_name="hrr_cobot",
                 gripper_node_names_parameter_name='/hrr_cobot/gripper_node_names',
                 urdf_prefix="hrr_cobot.", tool_parameter_name="/hrr_cobot/tool_name"):
        self._cobot_ns = hrr_common.fix_ns(cobot_ns)
        self._gripper_node_names_parameter_name = gripper_node_names_parameter_name
        self._robot_name = robot_name
        self._tool_parameter_name = tool_parameter_name
        try:
            self._tum_lab = '129.187' in os.getenv("ROS_IP")
        except TypeError as e:
            rospy.logwarn(f"could not check ROS-IP: {e}")
            self._tum_lab = False
        self._urdf_prefix = urdf_prefix
        self.tool_name = None
        self.update_tool_name(tool_name)

    @staticmethod
    def _log_str(msg):
        return f"URDF-updater->{msg}"

    def _update_urdf(self) -> bool:
        t0 = rospy.get_time()
        update_urdf(robot_name=self._robot_name, cobot_ns=self._cobot_ns, tum_lab=self._tum_lab,
                    tool_name=self.tool_name, urdf_prefix=self._urdf_prefix)
        for t in range(100):
            try:
                rospy.get_param(f"{self._cobot_ns}robot_description")
                if self.valid:
                    rospy.loginfo(self._log_str(
                        f"Updated URDF with tool {self.tool_name} after {rospy.get_time() - t0:.2f} seconds"))
                    return True
            except KeyError:
                pass
            rospy.sleep(0.2)
        rospy.logerr(self._log_str(f"updating URDF failed after {rospy.get_time() - t0:.2f} seconds"))
        return False

    def _state_publisher(self, wait_time=0.5):
        if self.p_joint_state_publisher is not None:
            self.p_joint_state_publisher.kill()
            rospy.loginfo(self._log_str(f"Killed robot state publisher. Wait for {wait_time} before restarting"))
            rospy.sleep(wait_time)

        self.p_joint_state_publisher = subprocess.Popen(["rosrun", "robot_state_publisher", "robot_state_publisher",
                                                         f"robot_description:={self._cobot_ns}robot_description",
                                                         f"__name:={self._robot_name}_state_publisher", f"__ns:=/"
                                                         ],
                                                        start_new_session=True)
        rospy.loginfo(self._log_str(f"Started robot state publisher."))

    def update_tool_name(self, tool_name):
        if tool_name != self.tool_name:
            rospy.loginfo(self._log_str(f"update tool from {self.tool_name} to {tool_name}"))
            self.tool_name = tool_name
            self._update_urdf()
            self._state_publisher()
        else:
            rospy.loginfo(self._log_str(f"current tool is already set to {tool_name}"))
        if not self.valid:
            rospy.logerr(self._log_str("current URDF does not match current tool / robot. please check the setup."))

    def tool_cb(self, msg):
        self.update_tool_name(set_tool_name_from_msg(msg))
        rospy.set_param(self._tool_parameter_name, self.tool_name)

    @property
    def valid(self):
        try:
            description_str = get_robot_description(f"{self._cobot_ns}robot_description")
        except KeyError:
            rospy.logerr_throttle(3.0, self._log_str("could not get robot description parameter"))
            return False
        try:
            assert f"{self._urdf_prefix}base_link" in description_str, "base link missing in URDF"
            assert f"{self._urdf_prefix}ee_link" in description_str, "ee link missing in URDF"
            tool_id = tool2tool_type(self.tool_name)
            if tool_id == ToolType.WSG_50_DSA:
                assert f"{self._urdf_prefix}wsg_50_tcp" in description_str, "wsg 50 tcp missing in URDF"
            elif tool_id == ToolType.SHAFT_GRINDER:
                assert f"{self._urdf_prefix}shaftgrinder_tip" in description_str, "shaft grinder tcp misses in URDF"
            elif tool_id == ToolType.SCREW_DRIVER:
                assert f"{self._urdf_prefix}screwdriver_tip" in description_str, "screwdriver tcp missing in URDF"
            elif tool_id == ToolType.VACUUM_GRIPPER:
                assert f"{self._urdf_prefix}vacuum_tip_1" in description_str, "vacuum tip missing in URDF"
        except AssertionError as e:
            rospy.logerr_throttle(3.0, self._log_str(e))
            return False
        return True


if __name__ == "__main__":
    rospy.init_node("test_urdf")
    urdf = URDFHandles("vacuum")
    rospy.loginfo(f"URDF set correctly: {urdf.valid}")
    rospy.spin()
