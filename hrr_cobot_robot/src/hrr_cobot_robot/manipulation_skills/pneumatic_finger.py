#!/usr/bin/env python
"""
Vaccum Grasping Action server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Read the comments in the file to check if yiu need to add something.
"""
import numpy as np
import spatialmath as sm

import actionlib
import rospy

from hr_recycler_msgs.msg import (
    FingerGraspAction, FingerGraspGoal,
    FingerGraspResult, FingerGraspFeedback, CobotState, ToolType, SkillResult
)
from hrr_cobot_robot.manipulation_skills.skill_base import SkillBase, get_SE3_from_pose_stamped

# add the files/modules from your notebook that are nit already there

import hrr_common as cu

__all__ = ["PneumaticFingerActionServer"]


class PneumaticFingerActionServer(SkillBase):
    """Simple Finger Grasping Action"""

    def __init__(self):
        super(PneumaticFingerActionServer, self).__init__(name="finger_grasping")
        self._as = None
        self._feedback = FingerGraspFeedback()
        # Hardcoded param, how many meters above the given object center should we go?
        self.safety_distance = 0.03
        # If True then use calibration pose and position from vision only
        self.hardcoded_orientation = True #Stay on True, orientation based on vision not supported
        self.turn_maneuver = True # Tries to turn the gripper so that battery is better grasped. If True, then always force sensitive
        self.force_sensitive = True  # If True go down until force is felt, otherwise go to exact grasp pose from vision
        self.ee2tip = np.r_[0, 0, -0.234]  # From end effector to tool tip (middle of fingers)
        self.ee2sidetip = np.r_[0, 0.05, -0.234]  # From end effector to one fingertip
        self.f_contact = 10  # Contact force (for going down until..)
        self.speed = 0.1 #AMoving around before and after
        self.tum_lab = False
        self.stop_it = False #For cancelling checks

    def init_ros(self, action_srv_name):
        """
        Initialize ROS API -> action serviceCobotState
        Initialize ROS API -> action serviceCobotState

        Args:
            action_srv_name(str, optional): name of action
        """
        self._as = actionlib.SimpleActionServer(action_srv_name, FingerGraspAction,
                                                execute_cb=self.execute_skill_cb, auto_start=False)
        self._as.start()

    @classmethod
    def _from_ros(cls, cobot=None, cobot_prefix="/hrr_cobot", skill_prefix="~"):
        out = cls()
        out.init_skill_base(cobot_prefix=cobot_prefix, cobot=cobot)
        out.init_ros(action_srv_name=cu.get_param(f"{cobot_prefix}finger_grasping_action_srv_name",
                                                  "~finger_grasp"))
        skill_prefix = cu.fix_prefix(skill_prefix)
        out.safety_distance = cu.get_param(f"{skill_prefix}safety_distance", out.safety_distance)

        return out

        # Here starts the routine
        # Go to infront

    def base2tool(self, pose):
        # Transform pose (sm.SE3) from base into vacuum tool tip space
        # Input is given pose, output is the pose of the EE such that gripper is at given pose
        return pose @ sm.SE3(self.ee2tip)

    def move_until_contact(self, goal_pose, force, v_max=0.01):
        completed = False
        if not self.stop_it:
            self.cobot.move_to_pose(goal_pose, v_max=v_max)
            F0 = np.copy(self.cobot.B_F_msr[:3])
            for t in range(int(100 * self.cobot.hz)):
                if not self.action_server_valid:
                    self.cancel(msg="pneumatic grasping pre-empted")
                    self.stop_it = True
                    break
                elif self.cobot.state is None:
                    completed = True
                    rospy.loginfo(f"Movement done, found no contact. Force deviation {self.cobot.B_F_msr[:3] - F0}")
                    break
                elif self.cobot.state == "error":
                    return self.end_skill(msg="Pneumatic grasping failed. Cobot in state 'error'.")
                elif np.linalg.norm((self.cobot.B_F_msr[:3] - F0) - self.cobot.FT.noise[:3]) >= force:
                    rospy.loginfo(f"Found contact. Force deviation {self.cobot.B_F_msr[:3] - F0}")
                    break
                else:
                    self.cobot.update()
        return completed

    def goTo_with_cancel(self, pose, v_max = 0.01):
        #Like goTo but checks if action server was cancelled or timeout reached.
        if not self.stop_it:
            self.cobot.move_to_pose(pose, v_max=v_max)
            for t in range(int(100 * self.cobot.hz)):
                if not self.action_server_valid:
                    self.cancel(msg="pneumatic grasping pre-empted")
                    self.stop_it = True
                    break
                elif self.cobot.state is None:   
                    return True
                elif self.cobot.state == "error":
                    self.end_skill(msg="Vacuum grasping failed. Cobot in state 'error'.")
                    return False
                else:
                    self.cobot.update()

    def execute_skill_cb(self, goal: FingerGraspGoal):
        """
        Actual Skill Execution as an action-service ()
        Args:

        """
        self.stop_it = False
        self.pre_skill_execution()
        self.timeout = goal.timeout
        self.cobot.tool_id = ToolType.WSG_50  # REMOOOVE!!
        if self.cobot.tool_id != ToolType.WSG_50:
            rospy.loginfo("Tool id not set to pneumatic finger gripper (WSG50 id)")
            self.problemEncountered()
            return

        # Get grasp pose and release pose from goal
        object_center = get_SE3_from_pose_stamped(goal.object_center)
        release_center = get_SE3_from_pose_stamped(goal.release_pose)

        if self.hardcoded_orientation:
            if self.tum_lab:
                grasp_pose = self.cobot.FK(np.r_[0., 0., -1.57079, 0., 1.57079, np.deg2rad(-45)])
            else:
                grasp_pose = self.cobot.FK(np.r_[0., 0., -1.57079, 0., 1.57079, 0])
            [grasp_pose.t[0], grasp_pose.t[1], grasp_pose.t[2]] = object_center.t

            release_pose = self.cobot.FK(self.cobot.q_calib)
            [release_pose.t[0], release_pose.t[1], release_pose.t[2]] = release_center.t
        else:
            # Not supported yet
            return
        if goal.release_pose.pose.orientation.x == 0:
            self.turn_maneuver = False
        else:
            self.turn_maneuver = True
        # Transform into tool tip space
        grasp_pose = self.base2tool(grasp_pose)
        release_pose = self.base2tool(release_pose)
        # release_pose = self.cobot.FK(np.deg2rad(np.array([-77.1, 42.6, -84, 0, 57.3, 89.8])))

        # Compute pre_poses (above the actual ones)
        pre_grasp_pose = grasp_pose @ sm.SE3([0, 0, -self.safety_distance])
        pre_release_pose = release_pose @ sm.SE3([0, 0, -self.safety_distance])

        # Go to starting pose
        self.cobot.move_to_joint_pose(self.cobot.q_calib, stochastic=True)

        # if not self.action_server_valid:
        #     return
        self.cobot.change_tool("vacuum")
        self.cobot.tool_controller.vacuum = True
        if not self.action_server_valid:
            return self.cancel(msg="pneumatic grasping pre-empted")
        # Should use task planner in the end
        rospy.loginfo("PneumaticFinger-Skill: Moving to pre_pose with planner")
        
        if self.turn_maneuver:
            rospy.loginfo("PneumaticFinger-Skill: Initiating turn to feel battery")
            self.cobot.tool_controller.vacuum = False #Closed
            self.goTo_with_cancel(sm.SE3([0, -0.10, 0])@pre_grasp_pose, v_max=self.speed)
            # Now move down
            self.move_until_contact(sm.SE3([0, 0, -self.safety_distance - 0.05]) @ self.cobot.T_B_E_robot, self.f_contact)
            # Now we touch the bottom, go up a bit
            self.goTo_with_cancel(sm.SE3([0, 0, 0.005]) @ self.cobot.T_B_E_robot)            

            # Now go along y axis (neg. or pos. depening on what rotation direction was chosen before) until force is felt
            rospy.loginfo(f"Trying to make contact with battery using threshold {0.15*self.f_contact}")
            self.move_until_contact(sm.SE3([0, 0.035, 0]) @ self.cobot.T_B_E_robot, self.f_contact,v_max=0.008)

            #Now we touch the battery, go up and adjust y properly
            self.goTo_with_cancel(sm.SE3([0, 0, 0.05]) @ self.cobot.T_B_E_robot, v_max=self.speed)
            self.goTo_with_cancel(sm.SE3([0, 0.094, 0])@ self.cobot.T_B_E_robot, v_max=self.speed)
            
            rospy.loginfo(f"PneumaticFinger-Skill: Moving down until contact, f_contact = {self.f_contact}N")
            self.cobot.tool_controller.vacuum = True #Open and ready to grasp
            self.move_until_contact(sm.SE3([0, 0, -self.safety_distance - 0.05])@ self.cobot.T_B_E_robot, 1.5*self.f_contact)
        else:
            self.goTo_with_cancel(pre_grasp_pose, v_max=self.speed)
            if self.force_sensitive:
                # Move down until force is felt
                rospy.loginfo(f"PneumaticFinger-Skill: Moving down until contact - 5cm below grasp pose, treshold f_contact = {self.f_contact}N")
                self.move_until_contact(grasp_pose @ sm.SE3([0, 0, 0.05]), 1.5*self.f_contact)
                self.goTo_with_cancel(sm.SE3([0, 0, 0.01]) @ self.cobot.T_B_E_robot)
            else:
                self.goTo_with_cancel(grasp_pose, v_max=0.01)

        # Close gripper
        self.cobot.tool_controller.vacuum = False
        rospy.sleep(0.5)
        
        # Backup 15 cm
        self.goTo_with_cancel(self.cobot.T_B_E_robot @ sm.SE3([0, 0, -0.15]), v_max=self.speed)

        # Go to release pose
        rospy.loginfo("PneumaticFinger-Skill: Going to pre_release_pose with planner")
        #self.cobot.stochastic_move_to_pose(pre_release_pose)
        self.goTo_with_cancel(pre_release_pose, v_max=self.speed)
        rospy.loginfo("PneumaticFinger-Skill: Going to release pose and opening fingers")
        self.goTo_with_cancel(release_pose, v_max=self.speed)
        self.cobot.tool_controller.vacuum = True
        self.goTo_with_cancel(pre_release_pose, v_max=self.speed)
        self.cobot.move_to_joint_pose(self.cobot.q_calib, stochastic = False)
        self.cobot.change_tool("wsg_50")
        self._skill_result.result = SkillResult.FINISHED
        self.end_skill("Pneumatic finger done!")

    @property
    def result(self) -> FingerGraspResult:
        res = FingerGraspResult()
        self._update_skill_result()
        res.skill_result = self._skill_result
        return res

    def problemEncountered(self):
        if not self._skill_result.result == SkillResult.TIMEOUT:
            self._skill_result.result = SkillResult.FAILED
        self._as.set_aborted(self.result)

    def publish_feedback(self, current_action, cobot_state=CobotState.BUSY, cobot=None):
        cobot = self._get_cobot(cobot)
        fb = FingerGraspFeedback()
        fb.state = cobot.cobot_state_msg(current_action, cobot_state)
        try:
            self._as.publish_feedback(fb)
        except AttributeError:
            rospy.logerr(f"failed to send {fb} for {current_action}")
