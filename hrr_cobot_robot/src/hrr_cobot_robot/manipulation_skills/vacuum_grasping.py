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

from std_msgs.msg import Int8
from geometry_msgs.msg import Pose

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
        self.safety_dist = 0.05
        # Hardcoded param, how many meters should we go below the given object z (until force is felt)
        self.below_dist = -0.05
        # If True then use calibration pose and position from vision only
        self.hardcoded_orientation = True # If false use vision data, otherwise see below
        self._f_contact = 15.0  # 15
        self._B_surface_normal = np.r_[0., 0., 1.]
        self._E_p_E_tip = np.r_[0, 0, 0.20018]
        self.grasp_mode = 0  # -1: Left cup, 0: center, 1: Right cup
        self.f_payload_minimum = 2  # ignored
        self.slow_speed = 0.014  # Use for approach and going up again to prepose
        self.fast_speed = 0.09 #  # Use 0.09 or more for real skill, use 0.03 for DD4 DD5
        self.stop_it = False

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
        # out.safety_dist = cu.get_param(f"{skill_prefix}safety_distance", out.safety_dist)
        # out.below_dist = cu.get_param(f"{skill_prefix}below_distance", out.below_dist)
        out._B_surface_normal[:] = cu.get_param(f"{skill_prefix}surface_normal", out._B_surface_normal)
        out._E_p_E_tip[:] = cu.get_param(f"{skill_prefix}E_p_tcp", out._E_p_E_tip)
        # out._f_contact = cu.get_param(f"{skill_prefix}f_contact", out._f_contact)
        # out.hardcoded_orientation = cu.get_param(f"{skill_prefix}hardcoded_orientation", out.hardcoded_orientation)
        return out

    def goTo_with_cancel(self, pose, v_max=0.04):
        # Like goTo but checks if action server was cancelled or timeout reached.
        if not self.stop_it:
            self.cobot.move_to_pose(pose, v_max=v_max)
            for t in range(int(100 * self.cobot.hz)):
                if not self.action_server_valid:
                    self.cancel(msg="vaccum grasping pre-empted")
                    self.stop_it = True
                    break
                elif self.cobot.state is None:
                    return True
                elif self.cobot.state == "error":
                    return self.end_skill(msg="Vacuum grasping failed. Cobot in state 'error'.")
                else:
                    self.cobot.update()

    def execute_skill_cb(self, goal: VacuumGraspingGoal):
        """
        Actual Skill Execution as an action-service ()
        Args:

        """

        self.pre_skill_execution(tool_id=ToolType.VACUUM_GRIPPER)
        self.stop_it = False
        self.cobot.change_tool("vacuum")  # comment if necessary!!
        if self.cobot.tool_id != ToolType.VACUUM_GRIPPER:
            return self.cancel(msg="According to tool_id: Vacuum Gripper not attached?!")

        object_center = get_SE3_from_pose_stamped(goal.object_center)
        release_center = get_SE3_from_pose_stamped(goal.release_pose)

        try:
            deviceType = rospy.wait_for_message('/hrr_cobot/deviceType', Int8, timeout=10).data
            devicePose = rospy.wait_for_message('/hrr_cobot/deviceCenter', Pose, timeout=10)
            print(devicePose)
        except rospy.ROSException as e:
            rospy.loginfo('No message received for device type or pose.')
            rospy.logerr(e)
            return self.end_skill(msg="Vacuum grasping failed. Could not find rostopic for device type and pose.")
        
        safety_dist = self.safety_dist
        if deviceType == 3:
            self._f_contact = 30
            if object_center.t[1] < devicePose.position.y and np.abs(object_center.t[1] - devicePose.position.y) > 0.03:
                # we need to grasp with right cup
                safety_dist = self.safety_dist+0.08
                rospy.loginfo(f'Device type is PC, using right cup (PSU) since goal y {object_center.t[1]} is right of center y {devicePose.position.y}')
                self.grasp_mode = 1
                ee2tip = self._E_p_E_tip + np.r_[0, 0.053, 0]  # use  for ecoreset
            elif object_center.t[2] < 0.15:
                # we need to grasp with left cup
                safety_dist = self.safety_dist+0.03
                rospy.loginfo(f'Device type is PC, using right cup anyway (cooler) even though goal y {object_center.t[1]}  is left of center y {devicePose.position.y}. ') #Changed this to right/left whatever
                self.grasp_mode = -1 #changed from -1
                ee2tip = self._E_p_E_tip + np.r_[0, 0.053, 0]  # use for ecoreset, changed from -0.053
            else:
                rospy.loginfo("Device type is PC but we are trying to lift the metal cover, using middle of gripper.")
                self.grasp_mode = 0  # Use middle of vacuum gripper
                ee2tip = self._E_p_E_tip

        else:
            rospy.loginfo("Not PC so using the middle of the vacuum gripper.")
            self.grasp_mode = 0  # Use middle of vacuum gripper
            ee2tip = self._E_p_E_tip
        
        # if deviceType == 3:
        #     self.hardcoded_orientation = False
        # else:
        #     self.hardcoded_orientation = True
        # Compute poses. self.hardcoded_orientation decides wether we use hardcoded orientation or compute from vision
        # Set desired orientation
        rospy.loginfo(f"received orient x {goal.object_center.pose.orientation.x}")
        if  goal.object_center.pose.orientation.x == 0.123456:  #hacking hardcoded orientation with dimitris
            rospy.loginfo("Must be middle cover!")
            #Middle cover --> Turn 
            q_des = np.array([0, 0, -1.5708, 0., 1.5708, 0])  # old calibration pose
        else:  
            rospy.loginfo("Taking standard orientation")
            q_des = np.array([0, 0, -1.5708, 0., 1.5708, -1.5708])  # old calibration pose with EE turned 90 degrees (better for middle cover)
        R_des = self.cobot.FK(q_des).R  # Desired orientation as SE3

        rospy.loginfo(f"this is the object center: {object_center.t}")
        # Set grasp pose
        grasp_pose = sm.SE3(object_center.t + ee2tip)
        grasp_pose.A[0:3, 0:3] = R_des

        # Set release pose
        release_pose = sm.SE3(release_center.t + ee2tip)
        release_pose.A[0:3, 0:3] = R_des

        # Alternatively hardcode some joint values
        # release_pose = self.cobot.FK(np.deg2rad(np.array([-50.6, 54.6, -74.5, 1.7, 48.4, -30])))

        if not self.hardcoded_orientation:
            #Change this so it can take only the normal vector
            rospy.loginfo(
                "Vacuum Skill overriding hardcoded orientation for pickup based on vision data")
            # poses computed based on vision
            # 1. get EE pose based on surface normal object_center.A[:3,0] (should be close to [0,0,1]) and on y_axis
            self._B_surface_normal = object_center.A[:3, 0].copy()
            rospy.loginfo(self._B_surface_normal)
            rospy.loginfo(object_center.A[:3, 3])
            grasp_pose_EE = cu.calc_goal_pose(self._B_surface_normal, object_center.t, y_axis=object_center.A[:3, 1])
            # 2. adjust to accomodate tool-tip
            #grasp_pose = sm.SE3(ee2tip) @ grasp_pose_EE
            #Shouldnt it be this instead? To go up through direction of turned EE
            if deviceType == 3:
                grasp_pose = sm.SE3(object_center.t + ee2tip)
            else:
                grasp_pose = grasp_pose_EE @ sm.SE3(-ee2tip)

            rospy.loginfo(f"grasp_pose is {grasp_pose}")

        pre_pose = grasp_pose @ sm.SE3([0, 0, -safety_dist])
        rospy.loginfo(pre_pose)

        below_pose = grasp_pose @ sm.SE3([0, 0, -self.below_dist])

        pre_release_pose = release_pose @ sm.SE3([0, 0, -safety_dist])

        # For now ignore orientation and just use position, assuming we come down along z axis and have last link = 0

        # Go to starting pose (not needed if we use task planner in the end)
        if not self.action_server_valid:
            return self.cancel(msg="vaccum grasping pre-empted")
        rospy.loginfo(
            "Vacuum Skill: Going to pre_pose with planner")
        # if self.hardcoded_orientation:
        if np.linalg.norm(self.cobot.q - q_des)<0.01:
            rospy.loginfo("already at standard joints, using goTo")
            self.goTo_with_cancel(pre_pose, v_max=self.fast_speed)
        else:
            rospy.loginfo("not in correct joints, using planner")
            self.cobot.move_to_joint_pose(q_des, stochastic=True)
            self.goTo_with_cancel(pre_pose, v_max=self.fast_speed)
        # else:
        #     rospy.loginfo("not in correct joints, using planner")
        #     self.cobot.move_to_joint_pose(self.cobot.q_calib, stochastic=True)
        #     self.goTo_with_cancel(pre_pose, v_max=self.fast_speed)

        # Move down until force is felt if prepose is reached:
        rospy.loginfo(
            f"Vacuum Skill: Moving down until contact (f_contact = {self._f_contact}N)")
        
        self.cobot.move_to_pose(below_pose, err_gain=None, v_max=self.slow_speed)
        F0 = np.copy(self.cobot.B_F_msr[0:3])
        self.cobot.tool_controller.vacuum = True
        for t in range(int(100 * self.cobot.hz)):
            if not self.action_server_valid:
                self.cobot.tool_controller.vacuum = False
                self.cobot.goTo(self.cobot.FK(q_des),v_max=1.5*self.fast_speed)
                return self.cancel(msg="vaccum grasping pre-empted")
            if self.cobot.state is None:
                self.cobot.tool_controller.vacuum = False
                rospy.loginfo(
                    f"Vacuum Skill moved {100 * self.below_dist} cm below given position, found no object, something is wrong!")
                return self.end_skill(msg="Vacuum grasping failed. Could not find any object.")
            elif self.cobot.state == "error":
                self.cobot.tool_controller.vacuum = False
                return self.end_skill(msg="Vacuum grasping failed. Cobot in state 'error'.")
            if ((self.cobot.B_F_msr[:3] - F0) - self.cobot.FT.noise[:3]).T @ self._B_surface_normal >= self._f_contact:
                rospy.loginfo(
                    f"Vacuum Skill: Made contact, turning on vacuum")
                rospy.sleep(0.5)
                break
            self.cobot.update()

        if not self.action_server_valid:
            return self.cancel(msg="vaccum grasping pre-empted")
        # Go up again
        #Start slow
        self.goTo_with_cancel(self.cobot.T_B_E_robot @ sm.SE3([0, 0, -0.02]), v_max=0.005)
        rospy.loginfo("Vacuum Skill: goTo back to pre_pose")
        if deviceType == 3 and self.grasp_mode == 1:
            rospy.loginfo("Moving slowly and along y because of PSU")
            psu_pre_pose = sm.SE3([0, 0.06, 0]) @ pre_pose
            self.goTo_with_cancel(psu_pre_pose, v_max=0.7*self.fast_speed)
        else:
            self.goTo_with_cancel(pre_pose, v_max=self.fast_speed)  # self.cobot.goTo(pre_pose, v_max=self.slow_speed)
        #Go up a bit more
        self.goTo_with_cancel(sm.SE3([0,0,0.15])@self.cobot.T_B_E_robot,v_max=self.fast_speed)
        #If force is too high
        # if np.abs(self.cobot.T_B_E_robot.t[2] - pre_pose.t[2]) > 0.01:
        #     rospy.loginfo("Vacuum Skill: Not at prepose yet, something went wrong, will try again in 0.5sec")
        #     rospy.sleep(0.5)
        #     self.goTo_with_cancel(pre_pose, v_max=self.fast_speed)
        
        # Check if force is different. current force should be bigger since object is grasped
        # rospy.sleep(0.1)
        # if ((self.cobot.B_F_msr[:3] - F0) - self.cobot.FT.noise[
        #                                   :3]).T @ self._B_surface_normal > -self.f_payload_minimum:
        # Current force is not much bigger, no object grasped
        #    return self.cancel(msg="No object felt")

        # Go above release pose
        if deviceType == 3 and self.grasp_mode == 1:
            rospy.loginfo("going to pre-release pose slowly because of PSU")
            self.goTo_with_cancel(pre_release_pose, v_max=self.fast_speed)
        else:
            self.goTo_with_cancel(pre_release_pose, v_max=2*self.fast_speed)

        # Go to release pose
        self.goTo_with_cancel(sm.SE3(release_pose), v_max=self.fast_speed)

        # Disable vacuum
        if not self.stop_it:
            rospy.loginfo(
                "Vacuum Skill: Opening vacuum")
            self.cobot.tool_controller.vacuum = False

        # Go back to calibration pose
        self.goTo_with_cancel(pre_release_pose, v_max=1.5 * self.fast_speed)
        rospy.loginfo(
            "Vacuum Skill: Arrived back at pre_release_pose, skill done.")
        if self.hardcoded_orientation:
            rospy.sleep(0.1)
            self.cobot.goTo(self.cobot.FK(q_des),v_max=1*self.fast_speed)
        else:
            rospy.sleep(0.1)
            self.cobot.goTo(self.cobot.FK(self.cobot.q_calib),v_max=1*self.fast_speed)
        self.end_skill("Vacuum grasping finger done!")
        #return self.end_skill(msg="Vacuum grasping done.")

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
