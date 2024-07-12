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
        self.hardcoded_orientation = True
        self.force_sensitive = False  # If True go down until force is felt, otherwise go to exact grasp pose from vision

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
        return pose @ sm.SE3([0, 0, -0.234])

    def execute_skill_cb(self, goal: FingerGraspGoal):
        """
        Actual Skill Execution as an action-service ()
        Args:

        """
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
            grasp_pose = self.cobot.FK(self.cobot.q_calib)
            [grasp_pose.t[0], grasp_pose.t[1], grasp_pose.t[2]] = object_center.t

            release_pose = self.cobot.FK(self.cobot.q_calib)
            [release_pose.t[0], release_pose.t[1], release_pose.t[2]] = release_center.t
        else:
            # Not supported yet
            return

        # Transform into tool tip space
        grasp_pose = self.base2tool(grasp_pose)
        release_pose = self.base2tool(release_pose)
        #release_pose = self.cobot.FK(np.deg2rad(np.array([-77.1, 42.6, -84, 0, 57.3, 89.8])))


        # Compute pre_poses (above the actual ones)
        pre_grasp_pose = grasp_pose @ sm.SE3([0, 0, -self.safety_distance])
        pre_release_pose = release_pose @ sm.SE3([0, 0, -0.1])

        # Go to starting pose
        self.cobot.move_to_joint_pose(self.cobot.q_calib)

        # if not self.action_server_valid:
        #     return
        self.cobot.change_tool("vacuum")
        self.cobot.tool_controller.vacuum = True

        # Should use task planner in the end
        self.cobot.goTo(pre_grasp_pose, v_max=0.02)

        # if not self.action_server_valid:
        #     return

        if self.force_sensitive:
            # Move down until force is felt
            self.cobot.move_to_pose(grasp_pose @ sm.SE3([0, 0, 0.02]), err_gain=None, v_max=0.01)
            force_z = []
            rospy.sleep(0.1)
            T = int(100 * self.cobot.hz)
            for t in range(T):
                force_z.append(self.cobot.FT_F[2])  # Force in z direction
                if self.cobot.state is None:
                    rospy.loginfo(
                        f"Finger Skill moved {100 * self.below_dist} cm below given position, found no object, something is wrong!")
                    self.problemEncountered()
                    return
                elif self.cobot.state == "error":
                    rospy.loginfo("Cobot in error state, aborting pneumatic finger skill.")
                    self.problemEncountered()
                    return
                # if not self.action_server_valid:
                #     return
                if abs(np.mean(force_z) - force_z[-1]) > 1:
                    break
                self.cobot.update()
        else:
            self.cobot.goTo(grasp_pose, v_max=0.01)

        # Close gripper
        self.cobot.tool_controller.vacuum = False

        # Backup 15 cm
        self.cobot.goTo(self.cobot.T_B_E_robot @ sm.SE3([0, 0, -0.15]), v_max=0.03)
        # Check if force is different. current force should be bigger since object is grasped
        rospy.sleep(0.2)

        # if self.cobot.FT_F[2] <= force_z[0] + 0.5:
        #     # Current force is not much bigger, no object grasped
        #     self.problemEncountered()
        #     return

        # if not self.action_server_valid:
        #     return

        # Go to release pose
        self.cobot.goTo(pre_release_pose, v_max=0.02)

        # if not self.action_server_valid:
        #     return
        # Disable finger to drop the object

        self.cobot.goTo(release_pose, v_max=0.02)
        self.cobot.tool_controller.vacuum = True
        self.cobot.goTo(pre_release_pose, v_max=0.02)
        # Go back to calibration pose
        #curj = self.cobot.q_calib.copy()
        #curj[0] = np.deg2rad(-110)
        self.cobot.move_to_joint_pose(self.cobot.q_calib)
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
