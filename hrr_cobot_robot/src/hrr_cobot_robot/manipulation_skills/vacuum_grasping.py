#!/usr/bin/env python
"""
Vacuum Grasping Action server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Read the comments in the file to check if you need to add something.
"""
import numpy as np
import spatialmath as sm

import actionlib
import rospy

from hr_recycler_msgs.msg import (
    VacuumGraspingAction, VacuumGraspingGoal,
    VacuumGraspingResult, VacuumGraspingFeedback, CobotState, ToolType
)
from hrr_cobot_robot.manipulation_skills.skill_base import SkillBase, get_SE3_from_pose_stamped

import hrr_common as cu

__all__ = ["VacuumGraspingActionServer"]


class VacuumGraspingActionServer(SkillBase):
    """Vacuum Grasping Action"""

    def __init__(self):
        super(VacuumGraspingActionServer, self).__init__(name="vacuum_grasping")
        self._feedback = VacuumGraspingFeedback()
        # Hardcoded param, how many meters above the given object center should we go?
        self.safety_dist = 0.15
        # Hardcoded param, how many meters should we go below the given object z (until force is felt)
        self.below_dist = -0.1
        # If True then use calibration pose and position from vision only
        self.hardcoded_orientation = True
        self._f_contact = 15.0
        self._B_surface_normal = np.r_[0., 0., 1.]
        self._E_p_E_tip = np.r_[0, 0, 0.20018]
        self.f_payload_minimum = 2

    def init_ros(self, action_srv_name):
        """
        Initialize ROS API -> action serviceCobotState

        Args:
            action_srv_name(str, optional): name of action
        """
        self._as = actionlib.SimpleActionServer(action_srv_name, VacuumGraspingAction,
                                                execute_cb=self.execute_skill_cb, auto_start=False)
        self._as.start()

    @classmethod
    def _from_ros(cls, cobot=None, cobot_prefix="/hrr_cobot", skill_prefix="~"):
        out = cls()
        out.init_skill_base(cobot_prefix=cobot_prefix, cobot=cobot)
        out.init_ros(action_srv_name=cu.get_param(f"{cobot_prefix}vacuum_pick_place_action_srv_name",
                                                  "~vacuum_disposal"))
        skill_prefix = cu.fix_prefix(skill_prefix)
        out.safety_dist = cu.get_param(f"{skill_prefix}safety_distance", out.safety_dist)
        out.below_dist = cu.get_param(f"{skill_prefix}below_distance", out.below_dist)
        out._B_surface_normal[:] = cu.get_param(f"{skill_prefix}surface_normal", out._B_surface_normal)
        out._E_p_E_tip[:] = cu.get_param(f"{skill_prefix}E_p_tcp", out._E_p_E_tip)
        out._f_contact = cu.get_param(f"{skill_prefix}f_contact", out._f_contact)
        out.hardcoded_orientation = cu.get_param(f"{skill_prefix}hardcoded_orientation", out.hardcoded_orientation)
        return out

    def execute_skill_cb(self, goal: VacuumGraspingGoal):
        """
        Actual Skill Execution as an action-service ()
        Args:

        """
        self.pre_skill_execution(tool_id=ToolType.VACUUM_GRIPPER)

        self.cobot.change_tool("vacuum")  # comment if necessary!!
        if self.cobot.tool_id != ToolType.VACUUM_GRIPPER:
            return self.cancel(msg="According to tool_id: Vacuum Gripper not attached?!")

        object_center = get_SE3_from_pose_stamped(goal.object_center)
        release_center = get_SE3_from_pose_stamped(goal.release_pose)

        if self.hardcoded_orientation:
            if release_center.t[0]==1:
                q_des = np.array([0, 0, -1.5708, 0., 1.5708, 0])  # last was -1.5708
            else:
                q_des = np.array([0, 0, -1.5708, 0., 1.5708, -1.5708])
            R_des = self.cobot.FK(q_des).R
            # R_des = self.cobot.FK(self.cobot.q_calib).R

            grasp_pose = sm.SE3(object_center.t + self._E_p_E_tip)
            grasp_pose.A[0:3, 0:3] = R_des
            release_pose = sm.SE3(release_center.t + self._E_p_E_tip)
            release_pose.A[0:3, 0:3] = R_des
            release_pose = self.cobot.FK(np.deg2rad(np.array([-50.6, 54.6, -74.5, 1.7, 48.4, -30])))

        else:
            grasp_pose_EE = cu.calc_goal_pose(object_center.A[:3,0], object_center.t, y_axis=object_center.A[:3,1])
            grasp_pose = sm.SE3([0,0,self._E_p_E_tip]) @ grasp_pose_EE 
            self._B_surface_normal = object_center.A[:3,0]

            release_pose_EE = cu.calc_goal_pose(release_center.A[:3,0], release_center.t, y_axis=release_center.A[:3,1])
            release_pose = sm.SE3([0,0,self._E_p_E_tip]) @ release_pose_EE 
            #return self.cancel(msg="Thou shalt hard-code")


        pre_pose = sm.SE3([0, 0, self.safety_dist]) @ grasp_pose

        below_pose = sm.SE3([0, 0, self.below_dist]) @ grasp_pose

        # For now ignore orientation and just use position, assuming we come down along z axis and have last link = 0

        # Go to starting pose (not needed if we use task planner in the end)
        self.cobot.move_to_joint_pose(self.cobot.q_calib)

        # Go to pre-pose above object to be grasped
        self.cobot.goTo(pre_pose, v_max=0.05)  # Motion planner!

        # Move down until force is felt
        self.cobot.move_to_pose(below_pose, err_gain=None, v_max=0.02)
        F0 = np.copy(self.cobot.B_F_msr[0:3])
        for t in range(int(100 * self.cobot.hz)):
            if self.cobot.state is None:
                rospy.loginfo(
                    f"Vacuum Skill moved {100 * self.below_dist} cm below given position, found no object, something is wrong!")
                return self.end_skill(msg="Vacuum grasping failed. Could not find any object.")
            elif self.cobot.state == "error":
                return self.end_skill(msg="Vacuum grasping failed. Cobot in state 'error'.")
            if ((self.cobot.B_F_msr[:3] - F0) - self.cobot.FT.noise[:3]).T @ self._B_surface_normal >= self._f_contact:
                break
            self.cobot.update()
        self.cobot.tool_controller.vacuum = True
        # Enable vacuum
        rospy.sleep(0.5)
        
        # Go up again
        self.cobot.goTo(pre_pose, v_max=0.02)

        # Check if force is different. current force should be bigger since object is grasped
        #rospy.sleep(0.1)
        #if ((self.cobot.B_F_msr[:3] - F0) - self.cobot.FT.noise[
        #                                   :3]).T @ self._B_surface_normal > -self.f_payload_minimum:
        # Current force is not much bigger, no object grasped
        #    return self.cancel(msg="No object felt")

        # Go up 10 cm more to be safe
        self.cobot.goTo(sm.SE3([0, 0, 0.1]) @ self.cobot.T_B_E_robot, v_max=0.04)

        # Go above release pose
        self.cobot.goTo(sm.SE3([0, 0, 0.1]) @ release_pose, v_max=0.04)  # Motion Planner!

        # Go to release pose
        self.cobot.goTo(sm.SE3(release_pose), v_max=0.04)

        # Disable vacuum
        self.cobot.tool_controller.vacuum = False

        # Go back to calibration pose
        self.cobot.move_to_joint_pose(self.cobot.q_calib)  # Motion Planner!

        return self.end_skill(msg="Vacuum grasping done.")

    @property
    def result(self) -> VacuumGraspingResult:
        """Generate CalibrateActionResult from code,

        Returns:
            CalibrateCobotResult: action-service result
        """
        res = VacuumGraspingResult()
        self._update_skill_result()
        res.skill_result = self._skill_result
        return res

    def publish_feedback(self, current_action, cobot_state=CobotState.BUSY, cobot=None):
        cobot = self._get_cobot(cobot)
        fb = VacuumGraspingFeedback()
        fb.state = cobot.cobot_state_msg(current_action, cobot_state)
        try:
            self._as.publish_feedback(fb)
        except AttributeError:
            rospy.logerr(f"failed to send {fb} for {current_action}")
