#!/usr/bin/env python
"""
Tool Change Routine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Documentation about usage in ROS to be added here
"""
import pathlib
from typing import Union

import numpy as np

import actionlib
import rospy

from hr_recycler_msgs.msg import (
    ChangeToolAction, ChangeToolGoal,
    ChangeToolResult, ChangeToolFeedback, CobotState, ToolType
)

import hrr_common
from hrr_cobot_robot.manipulation_skills.skill_base import SkillBase
from hrr_cobot_robot.utils import tool_type2str

__all__ = ["ToolChangeActionServer"]


class ToolChangeActionServer(SkillBase):
    """Calibration Action"""

    def __init__(self):
        super(ToolChangeActionServer, self).__init__(name="tool_changer")
        self.v = np.r_[0.01, 0.02, 0.03, 0.035, 0.04]
        self.tool_change_poses_path = pathlib.Path(__file__).parent.parent.parent.parent / "data" / "tool_changer"
        assert self.tool_change_poses_path.exists(), f"unknown path {self.tool_change_poses_path}"
        self.starting_pose = None

    def init_ros(self, action_srv_name="/hrr_cobot/change_tool"):
        """
        Initialize ROS API -> action service

        Args:
            action_srv_name(str): name of action
        """
        self._as = actionlib.SimpleActionServer(action_srv_name, ChangeToolAction,
                                                execute_cb=self.execute_skill_cb, auto_start=False)
        self._as.start()

    @classmethod
    def _from_ros(cls, cobot=None, cobot_prefix="/hrr_cobot", skill_prefix="~"):
        out = cls()
        out.init_skill_base(cobot_prefix=cobot_prefix, cobot=cobot)
        out.init_ros(action_srv_name=hrr_common.get_param(
            f"{cobot_prefix}change_tool_action_srv_name", "~change_tool"))
        skill_prefix = hrr_common.fix_prefix(skill_prefix)
        out.v = hrr_common.get_param(f"{skill_prefix}v", out.v)
        tmp = pathlib.Path(hrr_common.get_param(
            f"{skill_prefix}tool_change_poses_path", out.tool_change_poses_path))
        if tmp.exists():
            out.tool_change_poses_path = tmp
        return out

    def load_routine_dict(self, tool_id: int) -> Union[dict, None]:
        # process goal input
        if tool_id == ToolType.NONE:
            return None
        elif tool_id == ToolType.WSG_50:
            filename = self.tool_change_poses_path / "pgrip_4.npy"
        elif tool_id == ToolType.WSG_50_DSA:
            filename = self.tool_change_poses_path / "wsg_3_hardcode.npy"
        elif tool_id == ToolType.SHAFT_GRINDER:
            filename = self.tool_change_poses_path / "shaftgrinder_1.npy"
        elif tool_id == ToolType.SCREW_DRIVER:
            filename = self.tool_change_poses_path / "screw_6.npy"
        elif tool_id == ToolType.VACUUM_GRIPPER:
            filename = self.tool_change_poses_path / "vacuum_2_hardcode.npy"
        else:
            return None
        if filename.exists():
            data = np.load(str(filename), allow_pickle=True)
            if isinstance(data, np.ndarray):
                data = data.item()
            assert isinstance(data, dict)
            return data
        else:
            rospy.logwarn(
                f"[tool_change_routine] Desired tool does not exist or filename {filename} "
                f"for poses is wrong. See get_poses_filename.")
            return None

    def move_to_starting_pose(self):
        self.cobot.move_to_joint_pose(self.starting_pose)

    def pickup_routine(self, poses, new_tool_str):
        """
        Picking up a new tool from the rack. There are no safety checks here, these happen in execute_skill_cb.
        """
        cobot = self._get_cobot(None)

        for i in range(len(poses["poses_A"])):
            if poses["close_tc"][i]:
                cobot.goTo(poses["poses_A"][i], v_max=0.01, check_reachable=False)
                rospy.loginfo(f"[tool_change_routine] reached pos {i} for pickup of {new_tool_str}.")
                self.publish_feedback(current_action=f"Reached pos {i}")
                rospy.loginfo(f"[tool_change_routine] Closing tool changer to pick up {new_tool_str}.")
                # Close Tool changer & Change tool id to new tool
                cobot.close_tool_changer(force=True)
                cobot.change_tool(new_tool_str)
                self.publish_feedback(current_action="Closed tool changer.")
                rospy.sleep(0.2)
            else:
                if poses["use_joint_ctrl"][i]:
                    cobot.move_to_joint_pose(poses["poses_q"][i])
                else:
                    cobot.goTo(poses["poses_A"][i], v_max=0.02, check_reachable=False)
                rospy.loginfo(f"[tool_change_routine] reached pos {i} for pickup of {new_tool_str}.")
                self.publish_feedback(current_action=f"Reached pos {i}")


    def dispose_routine(self, poses):
        """
        Putting a tool into the rack. There are no safety checks here, these happen in execute_skill_cb.
        """
        rospy.logdebug(f"[tool_change_routine] initiating dispose routine")
        self.publish_feedback(current_action="dispose tool-> approach pre pose")
        cobot = self._get_cobot(None)
        # # If outside starting_pose is given, move there first
        # if self.starting_pose is not None:
        #     assert self.starting_pose.shape == (len(cobot.q),)
        #     self.cobot.move_to_joint_pose(self.starting_pose

        for i in range(len(poses["poses_A"])):
            if poses["open_tc"][i]:
                # Close Tool changer & Change tool id to new tool
                cobot.goTo(poses["poses_A"][i], v_max=0.01,check_reachable=False)
                self.publish_feedback(current_action=f"Reached pos {i}")
                rospy.sleep(0.1)
                cobot.open_tool_changer()
                rospy.loginfo(f"[tool_change_routine] Opening tool changer to dispose.")
                cobot.change_tool("nothing")
                self.publish_feedback(current_action="Opened tool changer.")
            else:
                if poses["use_joint_ctrl"][i]:
                    cobot.move_to_joint_pose(poses["poses_q"][i])
                else:
                    cobot.goTo(poses["poses_A"][i], v_max=0.03, check_reachable=False)
                rospy.loginfo(f"[tool_change_routine] reached pos {i} for disposal.")
                self.publish_feedback(current_action=f"Reached pos {i}")


    def execute_skill_cb(self, goal: ChangeToolGoal):
        """
        Actual Skill Execution as an action-service

        If a tool is attached and desired tool (goal) is not None, first remove current tool and then get new one.
        If a tool is attached and desired tool (goal) is None, remove the current tool.
        If no tool is attached and desired tool (goal) is not None, get the new tool.
        If no tool is attached and desired tool (goal) is None, tell the user to rethink his life choices.

        Args:
            goal(ChangeToolGoal): Action-service goal

        """
        cobot = self._get_cobot(None)

        new_tool_str = tool_type2str(goal.new_tool.type)
        self.publish_feedback(current_action=f"change tool from {cobot.tool}) to {new_tool_str}).")
        
        if goal.new_tool.type == cobot.tool_id:
            return self.end_skill(
                msg=f"Current tool ({cobot.tool}) is equal to new tool ({new_tool_str}).",
                save_data=False)

        #self.move_to_starting_pose()

        if cobot.tool_id != ToolType.NONE:
            if False:#cobot.tool_changer_open:
                rospy.logwarn(
                    f"[tool_change_routine] cobot tool id says {cobot.tool} attached, "
                    "but tool changer is open. I will set tool_id to None.")
                cobot.tool_id = ToolType.NONE
                #This might be a safety issue: If the tool changer died/is unplugged,
                #the digital pin might indicate that the tool changer is open, even though it is closed. Better to abort?
            else:
                rospy.loginfo(f"cobot tool id says {cobot.tool} is attached, trying to dispose.")
                routine = self.load_routine_dict(cobot.tool_id)
                if routine is not None:
                    self.dispose_routine(routine["dispose"])
                else:
                    #Better to abort if invalid tool id requested
                    return

        #Change these assert ... boys to if ... abort?
        assert cobot.tool_id == ToolType.NONE, f"current tool is {cobot.tool}. Something went wrong during disposal"  
        #assert cobot.tool_changer_open, f"tool-changer is closed, something seems messed up. Exciting!"

        if goal.new_tool.type == ToolType.NONE:
            rospy.loginfo(f"[tool_change_routine] Successfully disposed tool. No new tool wanted. Done.")
            return self.end_skill(msg="Successfully disposed tool.")
        
        self.publish_feedback(current_action=f"pick up {new_tool_str}")
        rospy.logdebug(f"No tool attached, trying to get {new_tool_str}.")
        routine = self.load_routine_dict(goal.new_tool.type)
        assert routine is not None, f"no route found to pick up  {tool_type2str(goal.new_tool.type)}." #Better to abort
        self.pickup_routine(routine["pickup"], new_tool_str)
        rospy.loginfo(f"Successfully changed tool to {tool_type2str(goal.new_tool.type)}.")
        
        return self.end_skill(msg="Tool change routine done.")

    @property
    def result(self) -> ChangeToolResult:
        """Generate CalibrateActionResult from code,
        c.f. :py:meth:`~hrr_cobot_robot.ros_interfaces.sensor_handles.FTData.srv_response`
        Returns:
            CalibrateCobotResult: action-service result
        """
        res = ChangeToolResult()
        # tba: adjust as needed
        return res

    def publish_feedback(self, current_action, cobot_state=CobotState.BUSY, cobot=None):
        cobot = self._get_cobot(cobot)
        fb = ChangeToolFeedback()
        fb.state = cobot.cobot_state_msg(current_action, cobot_state)
        try:
            self._as.publish_feedback(fb)
        except AttributeError:
            rospy.logerr(f"failed to send {fb} for {current_action}")
