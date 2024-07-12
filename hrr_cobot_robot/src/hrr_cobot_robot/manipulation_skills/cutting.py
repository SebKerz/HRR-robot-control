#!/usr/bin/env python
"""
Cutting Skill
^^^^^^^^^^^^^^^^^^

Todo:
    * read device center and type
    * delete and change set param

"""

import time
import pathlib
import actionlib
import hrr_common as hrr_cm
import numpy as np
import rospy
import spatialmath as sm
from geometry_msgs.msg import Vector3, Pose
from hr_recycler_msgs.msg import (
    CuttingAction, CuttingFeedback, CuttingResult, SkillResult, MaterialType,
    CobotState, PlannerGoal, CuttingGoal)
from hr_recycler_msgs.msg import ToolType
from std_msgs.msg import Int8
from tqdm.notebook import trange

from hrr_cobot_robot.hrr_cobot_control import JointLimitWarning, JointLimitException
from hrr_cobot_robot.manipulation_skills.skill_base import SkillBase

__all__ = ["Cutting"]

# global variables

Shaftgrinder_microwave_May2022_A = np.array([[0.54651,  0.04365,  0.83631,  0.50479],
                                             [-0.83737,  0.01488,  0.54643, -0.27124],
                                             [ 0.01141, -0.99894,  0.04468,  0.37027],
                                             [ 0.     ,  0.     ,  0.     ,  1.     ]])
Shaftgrinder_microwave_May2022_q = np.array([0.62354,  0.66676, -1.87005, -1.90294,  1.40214, -2.12749])

Shaftgrinder_tf = np.array([[-0.707107, 0.707107, 0., 0.455498],
                            [0, 0, 1, 0.0482934],
                            [0.707107, 0.707107, 0, 0.653348],
                            [0, 0, 0, 1]])

Shaftgrinder_tf_1 = sm.SE3()
Shaftgrinder_tf_1.A[:4,:4] = [[0.9552, 0.02496, 0.2948, 0],
                              [-0.2956, 0.04037, 0.9545, 0],
                              [0.01192, -0.9989, 0.04594, 0],
                              [0, 0, 0, 1]]

Shaftgrinder_tf_PC_Tower_1 = np.array([[-0.26864, 0.55951, 0.78408, 0.50352],
                                       [0.67257, -0.47376, 0.5685, -0.04761],
                                       [0.68955, 0.68007, -0.24904, 0.26459],
                                       [0., 0., 0., 1.]])

Shaftgrinder_tf_PC_Tower_2 = np.array([[-0.25151, 0.58966, 0.76749, 0.50753],
                                       [0.22861, -0.73435, 0.63911, 0.01337],
                                       [0.94047, 0.3362, 0.04989, 0.22234],
                                       [0., 0., 0., 1.]])

Shaftgrinder_pose = sm.SE3()
Shaftgrinder_pose.A[:4, :4] = np.array([[-0.515, 0.02106, 0.8569, 0.3903],
                                        [-0.8571, 0.0004331, -0.5151, -0.01507],
                                        [-0.01122, -0.9998, 0.01782, 0.5722],
                                        [0, 0, 0, 1]])

Shaftgrinder_pose_q = np.array([-0.08106, -0.23017, -2.33784, -0.93423, -0.8107, 2.32261])


# toEE = np.array([0, -0.156, 0.094])
# toEE = np.array([0, 0, 0])


class Cutting(SkillBase):
    """
    Cutting Skill

    Args:
        SkillBase [class]: the parent class of all skills

    ...

    Attributes:
        cobot [HrrCobotControl] : an interface to the robot
        observer [HrrCobotObserver] : cobot observer
        _status [CobotState] : indicates the status of the robot, (CobotState.IDLE, CobotState.BUSY, CobotState.FINISHED, CobotState.FAILED)
        _name [string] : the name of the skill being executed
        _feedback [CuttingFeedback] : feedback published during the cutting action
        _result [CuttingResult] : result upon completion of the cutting action
        _as [actionlib.SimpleActionServer] : an action server used to execute cutting goals
        _tf  [SE3] : transformation matrix to end effector
        _start_time [time] : start time of the action to check for timeouts
        _deviceType [int8] : to be done! just a dummy type here. device type (MW : 1, EL : 2, PC : 3)
        _yofcenter [Vector3] : to be done! just a dummy type here. y coordinate of the device center.


    """
    _feedback = CuttingFeedback()
    _result = CuttingResult()

    def __init__(self):
        """
        Constructor for Cutting Class. Uses the parent class to initialize.
        """
        super(Cutting, self).__init__(name="cutting", cobot=None, observer=None)
        self._subs = []
        self._toEE = None
        self._start_time = None
        self._tf = None
        self._deviceType = None
        self._yofcenter = None
        self._dev_type_cb = -1
        self._B_dev_pos_cb = np.zeros(3)
        self._B_dev_quat_cb = np.quaternion(1., 0., 0., 0.)

    @property
    def device_type(self) -> int:
        if self._dev_type_cb > 0:
            return self._dev_type_cb
        return hrr_cm.ros_utils.get_param('/device_type')

    @property
    def B_device_center(self):
        return self._B_dev_pos_cb

    def get_y_of_device_center(self) -> float:
        if len(self._subs) > 1:
            return self.B_device_center[1]
        else:
            return rospy.wait_for_message(rospy.get_param('/device_center'), Vector3).y

    def init_ros(self, action_srv_name="~cutting",
                 device_type_topic_name=None,
                 device_pose_topic_name=None):
        """
        Initialize ROS API. Uses parent class to initialize.
        Initializes and starts the Simple Action Server "Cutting".
        Initializes Cutting Feedback and Result

        Args:
            action_srv_name(str, optional):
            device_pose_topic_name(str or None, optional): device pose topic name
            device_type_topic_name(str or None, optional): device type topic name
        """
        # rospy.set_param("/device_type", "/device_type")
        # rospy.set_param("/device_center", '/device_center_topic')
        self._start_time = time.time()
        self._as = actionlib.SimpleActionServer(action_srv_name, CuttingAction, self.execute_skill_cb,
                                                auto_start=False)
        # if device_pose_topic_name:
        #     self._subs.append(rospy.Subscriber(device_pose_topic_name, Pose, self._device_pose_cb, queue_size=10))
        # if device_type_topic_name:
        #     self._subs.append(rospy.Subscriber(device_type_topic_name, Int8, self._device_type_cb, queue_size=10))
        self._as.start()

    @classmethod
    def _from_ros(cls, cobot_prefix, cobot=None):
        """
        Gets ROS parameters for class.
        """
        out = cls()
        out.init_skill_base(cobot_prefix=cobot_prefix, cobot=cobot)
        out.init_ros(hrr_cm.get_param(f"{cobot_prefix}cutting_action_srv_name"),
                     # device_pose_topic_name=hrr_cm.get_param(f"{cobot_prefix}device_pose_topic_name", ""),
                     # device_type_topic_name=hrr_cm.get_param(f"{cobot_prefix}device_type_topic_name", ""),
                     )
        return out

    def _device_pose_cb(self, msg):
        """
        ROS-callback for device pose

        Args:
            msg(Pose):

        Todo:
            this should be a PoseStamped message, but the stochastic planner is doing the same...
        """
        self._B_dev_pos_cb = hrr_cm.vec32np(msg.position)
        self._B_dev_quat_cb = hrr_cm.quat2np(msg.orientation)

    def _device_type_cb(self, msg):
        """
        ROS-callback for device type

        Args:
            msg(Int8):
        """
        self._dev_type_cb = msg.data

    def execute_skill_cb(self, goal: CuttingGoal):
        """
        Actual Skill Execution as an action-service.

        Args:
            goal: Action-service goal. Contains: timeout, normal vectors of point A and B, coordinates of point A and B,
                  material type, material thickness (optional)

        Returns:
              none
        """

        if np.linalg.norm(self.cobot.T_B_E_robot.t - self.cobot.T_B_C_robot.t) < 0.01:
            return
        # self._as.set_aborted(msg="No tooltip transform")
        # self.cobot.tool_controller.tool = 'shaftgrinder'
        self.cobot.F_max = 120
        self._start_time = time.time()

        # self.cobot.tool_controller.rpm = 0

        feedback = CuttingFeedback()
        feedback.state = self.cobot.cobot_state_msg(
            "cutting", status=CobotState.BUSY)
        self._feedback = feedback

        if self.cobot.tool_id == ToolType.SHAFT_GRINDER and self.timeout_check(goal.timeout):

            self.update_feedback_msg(msg="Initiating cutting")

            result = SkillResult().UNKNOWN

            # publish info to the console
            rospy.loginfo(f'[{rospy.get_name()}->{self._name}]: '
                          f'Executing, cutting from {goal.start_location} to {goal.end_location}. '
                          f'Current Robot State: {self._feedback.state}')
            # retrieve the information received in a goal
            surface_normal_start = goal.surface_normal_start
            surface_normal_end = goal.surface_normal_end
            start_location = goal.start_location
            end_location = goal.end_location
            material = goal.material.material_type
            # thickness = 0.005
            # thickness = goal.thickness

            speed = self.determine_mode(material)
            # PrePoseA = hrr_cm.calc_EE_pre_pose(B_normal=hrr_cm.vec32np(surface_normal_start.vector),
            #                                    B_p_location=hrr_cm.vec32np(
            #                                        start_location.vector),
            #                                    T_C_E=self.cobot.T_E_C_robot.inv(),
            #                                    B_y_axis=np.r_[0., 1., 0.],
            #                                    safety_distance=5e-2)
            # PrePoseB = hrr_cm.calc_EE_pre_pose(B_normal=hrr_cm.vec32np(surface_normal_end.vector),
            #                                    B_p_location=hrr_cm.vec32np(
            #                                        end_location.vector),
            #                                    T_C_E=self.cobot.T_E_C_robot.inv(),
            #                                    B_y_axis=np.r_[0., 1., 0.],
            #                                    safety_distance=5e-2)
            # CuttingPoseA = self.calc_EE_cutting_pose(B_normal=hrr_cm.vec32np(surface_normal_start.vector),
            #                                          B_p_location=hrr_cm.vec32np(
            #                                              start_location.vector),
            #                                          T_C_E=self.cobot.T_E_C_robot.inv(),
            #                                          B_y_axis=np.r_[0., 1., 0.],
            #                                          depth=0)
            # CuttingPoseB = self.calc_EE_cutting_pose(B_normal=hrr_cm.vec32np(surface_normal_end.vector),
            #                                          B_p_location=hrr_cm.vec32np(
            #                                              end_location.vector),
            #                                          T_C_E=self.cobot.T_E_C_robot.inv(),
            #                                          B_y_axis=np.r_[0., 1., 0.],
            #                                          depth=0)

            #
            #PrePoseA, PrePoseB, CuttingPoseA, CuttingPoseB = self.updatetf(y=start_location.vector.y, PrePoseA=PrePoseA,
            #                                                                PrePoseB=PrePoseB, CuttingPoseA=CuttingPoseA,
            #                                                                CuttingPoseB=CuttingPoseB)

            # Geh zu shaftgrinder pose
            # self.cobot.move_to_joint_pose(Shaftgrinder_pose_q)
            # self.cobot.update_tf()
            # CuttingPoseA = sm.SE3(hrr_cm.vec32np(start_location.vector) + self._E_p_E_tip)
            # CuttingPoseA.A[0:3, 0:3] = self.R_des

            CuttingPoseA = sm.SE3()
            CuttingPoseA.A[:4, :4] = Shaftgrinder_tf_1.A.copy()
            tooltip_offset_A = hrr_cm.vec32np(start_location.vector) - (Shaftgrinder_tf_1@self.cobot.T_E_C_robot).t #so bzw siehe notebook
            [CuttingPoseA.t[0], CuttingPoseA.t[1], CuttingPoseA.t[2]] = CuttingPoseA.t + tooltip_offset_A

            PrePoseA = sm.SE3([0, 0, 0.05]) @ CuttingPoseA

            CuttingPoseB = sm.SE3() #Aendern!
            CuttingPoseB.A[:4, :4] = Shaftgrinder_tf_1.A.copy()
            tooltip_offset_B = hrr_cm.vec32np(end_location.vector) - (Shaftgrinder_tf_1@self.cobot.T_E_C_robot).t
            [CuttingPoseB.t[0], CuttingPoseB.t[1], CuttingPoseB.t[2]] = CuttingPoseB.t + tooltip_offset_B

            PrePoseB = sm.SE3([0, 0, 0.05]) @ CuttingPoseB

            # translation=T_BC_wish -T_B_C
            # gehe zu momentaner pose +translation

            # Check where we are
            if np.linalg.norm(self.cobot.q - Shaftgrinder_pose_q) < 0.01:
                # We're at shaft grinder pose (which is to be expected after tool change routine)

                path4poses = pathlib.Path(__file__).parent.parent.parent.parent / "data" / "shaftgrinder"
                filename = path4poses / "shaftgrind_help.npy"
                data2 = np.load(str(filename), allow_pickle=True)
                data1 = data2.item()
                poses = data1["from_shaftgrindpose_toEL"]
                for i in range(len(poses["poses_A"])):
                    if poses["use_joint_ctrl"][i]:
                        self.cobot.move_to_joint_pose(poses["poses_q"][i])
                    else:
                        self.cobot.goTo(poses["poses_A"][i], v_max=0.02)
            # Now we can use goTo for configurations from calc_goal_pose

            # Moves through Skill Graph
            # self.cobot.goTo(PrePoseA, v_max=0.02)
            # self.cobot.tool_controller.run_shaftgrinder(goal.timeout, speed)
            # self.cobot.goTo(CuttingPoseA, v_max=0.002)
            # self.cobot.goTo(CuttingPoseB, v_max=0.002)
            # self.cobot.goTo(PrePoseB)

            result = self.skill_graph(result=result, timeout=goal.timeout, use_motion_planner=False, pose=PrePoseA,
                                      v_max=0.2,
                                      rot_max=0.4,
                                      speed="None", msg="Moving to pre-pose")  # vmax = 0.02, rotmax 0.04
            result = self.skill_graph(result=result, timeout=goal.timeout, use_motion_planner=False, pose=CuttingPoseA,
                                      v_max=0.2,
                                      rot_max=0.2, speed=speed,
                                      msg="Moving to first cutting pose, Grinder ON")  # vmax 0.004, rotmax 0.02
            result = self.skill_graph(result=result, timeout=goal.timeout, use_motion_planner=False, pose=CuttingPoseB,
                                      v_max=0.2,
                                      rot_max=0.2, speed=speed,
                                      msg="Moving to second cutting pose, Grinder ON")  # vmax 0.004, rotmax 0.02
            result = self.skill_graph(result=result, timeout=goal.timeout, use_motion_planner=False, pose=PrePoseB,
                                      v_max=0.2,
                                      rot_max=0.2,
                                      speed=speed, msg="Moving to safe pose after cutting, Grinder ON")


        # Creates Feedback and Result
        else:
            rospy.logwarn(
                "tool controller is currently not in shaft grinder mode or timeout")
            result = SkillResult.FAILED

        if result == SkillResult.UNKNOWN and self.timeout_check(goal.timeout):
            self.update_feedback_msg(msg="Finished cutting")
            result = SkillResult.FINISHED
        elif result == SkillResult.UNKNOWN:
            self.update_feedback_msg(msg="Cutting did not fail but was not fully executed due to unknown reasons!")
        elif result == SkillResult.TIMEOUT or not self.timeout_check(goal.timeout):
            self.update_feedback_msg(msg="Cutting failed. Timeout!")
            result = SkillResult.TIMEOUT
        else:
            self.update_feedback_msg(msg="Cutting failed! Wrong tool, pre-empted or force exceeded!")
            result = SkillResult.FAILED

        # self.cobot.tool_controller.rpm = 0
        self.stop_grinder()

        self._result.skill_result = self.create_result(
            time.time() - self._start_time, result=result)
        if result == SkillResult.FINISHED:
            self._as.set_succeeded(result=self._result, text="Cutting succeeded!")
        else:
            msgdict = {SkillResult.TIMEOUT: "Cutting failed. Timeout!",
                       SkillResult.FAILED: "Cutting failed! Wrong tool, pre-empted or force exceeded!",
                       SkillResult.UNKNOWN: "Cutting did not fail but was not fully executed due to unknown reasons!"}
            #     # if self._as is not None:
            #     #     if self._as.current_goal.get_goal() is not None:
            #     self.update_feedback_msg(msg=msgdict[result])
            try:
                self._as.set_aborted(result=self._result, text=msgdict[result])
            except:
                rospy.logerr("no active goal found")

    def skill_graph(self, result, timeout, pose, v_max, rot_max, speed, msg: str, use_motion_planner=False):
        """
        Method to walk through the whole skill on a graph.
        Approaches PrePoseA, then CuttingPoseA (while grinder is on), then CuttingPoseB (grinder on) and
        finishes at PrePoseB (turns grinder off, when last point is reached)

        Args:
            msg: message for feedback of current state
            speed (str) : Sets the rpm for every step in graph indirectly.
            result (int) : Checks and updates the current skillresult.
            timeout (float) : Checks against timeout given in the action goal.
            use_motion_planner(bool) : Determines whether MoveTo Command (False) or Motion Planner (True) will be used. Defaults to True.
            pose (SE3) : Pose of the end point that should be approached, given in endeffector frame.
            v_max (float) : Maximal translational velocity for the robot.
            rot_max (float) : Maximal rotational velocity for the robot.

        Returns:

        """
        if result == SkillResult.UNKNOWN and self.timeout_check(timeout):
            if use_motion_planner:
                pose_qs = self.cobot.IK(pose)
                pose_q = self.find_best_IK(pose_qs)
                return self.cobot.move_to_joint_pose(pose=pose_q)
            else:
                return self.move_and_publish_rpm(pose=pose, result=result, v_max=v_max, rot_max=rot_max, msg=msg,
                                                 timeout=(timeout - (time.time() - self._start_time)), speed=speed)
        elif result == SkillResult.TIMEOUT or not self.timeout_check(timeout):
            return SkillResult.TIMEOUT
        else:
            return SkillResult.FAILED

    def move_and_publish_rpm(self, pose, result, v_max, rot_max, msg: str, timeout=50, speed="None",
                             compensate_joint_limits=False):
        """
        Move the cobot to the desired point, with a specified translational and rotational velocity, with the grinder spinning at a specified number of
        rounds per minute (default zero). Stops after a timeout.

        Args:
                result(int): SkillResult->result of current action
                pose(NumPy.array) : transformation matrix with the new pose
                v_max(float) : maximal linear velocity during the course of the action
                rot_max(float) : maximal rotational velocity during the course of the action
                timeout(int) : number of steps after which the action stops regardless of the results
                speed(str): sets indirectly rpms (integer in range [3500,25000] representing rounds per minute of the shaftgrinder). Defaults to "None" in which case the shaftgrinder doesn't cut.
                compensate_joint_limits(bool, optional):
        Returns:
                result(int):    message indicating the result of the action (finished, failed, timeout or unknown)
        """

        def call_move_to():
            self.cobot.move_to_pose(pose, v_max, rot_max,
                                    rot_precision_threshold=0.001,
                                    pos_precision_threshold=0.001)

        def hack_joint_limits():
            dq = self.cobot.joint_limit_avoidance_needed()
            rospy.logerr(f"encountered joint limit violation on joints {np.where(dq != 0.0)}")
            q_des = self.cobot.q.copy() - dq
            self.cobot.move_to_joint_pose(q_des, stochastic=False)
            rospy.logerr(f"reset joint configuration to {q_des}. Return to motion with decreased precision")
            prec_t, prec_r = self.cobot.transl_precision, self.cobot.rot_precision
            prec_t *= 1.1
            prec_r *= 1.1
            call_move_to()
            self.cobot.transl_precision, self.cobot.rot_precision = prec_t, prec_r

        feedback = CuttingFeedback()
        call_move_to()
        T = int(timeout * self.cobot.hz)

        # self.cobot.compensate_joint_limits = compensate_joint_limits

        if speed != "None":
            try:
                self.cobot.tool_controller.run_shaftgrinder(timeout, speed)
            except NotImplementedError:
                rpm_lookup = {"full_speed": 25000,
                              "slow": 17000,
                              "variable": 20000,
                              "None": 0}
                self.cobot.tool_controller.rpm = rpm_lookup[speed]

        for t in trange(T):

            if self._as.is_preempt_requested():
                rospy.loginfo('Pre-empted')
                self.stop_grinder()
                result = SkillResult.FAILED
                feedback.state = self.cobot.cobot_state_msg(
                    "Pre-empted", status=CobotState.FAILED)
                self._feedback = feedback
                self._as.publish_feedback(self._feedback)
                self._as.set_preempted()
                break

            if self.cobot.state is None:
                rospy.loginfo(f"reached goal pose at step {t + 1} / {T}")
                result = SkillResult.UNKNOWN
                feedback.state = self.cobot.cobot_state_msg(
                    f"reached goal pose for {msg}", status=CobotState.BUSY)
                self._feedback = feedback
                self._as.publish_feedback(self._feedback)
                break

            elif not self.action_sever_valid or self.cobot.state == "FAILURE":
                # self.cobot.tool_controller.tool = "None"
                self.stop_grinder()
                rospy.logerr(f"robot in ERROR state")
                result = SkillResult.FAILED
                feedback.state = self.cobot.cobot_state_msg(
                    f"robot in ERROR state", status=CobotState.FAILED)
                self._feedback = feedback
                self._as.publish_feedback(self._feedback)
                break

            self.cobot.update()
            # try:
            #     self.cobot.update()
            # except (JointLimitWarning, JointLimitException):
            #     hack_joint_limits()

            feedback.state = self.cobot.cobot_state_msg(
                msg, status=CobotState.BUSY)
            self._feedback = feedback
            self._as.publish_feedback(self._feedback)

        if self.cobot.state is None or self._as.is_preempt_requested() or not self.action_sever_valid or self.cobot.state == "FAILURE":
            return result
        else:
            return SkillResult().TIMEOUT

    @staticmethod
    def determine_mode(material):
        """
        Determines the mode for shaftgrinder based on material.
        Args:
            material [MaterialType] : Material from goal depending on the device and material that will be cut.

        Returns:
            speed [float] : sets rounds per minute for shaftgrinder control indirectly.
        """
        materialmsg = MaterialType()
        if material == materialmsg.PLASTIC:
            speed = "variable"
        elif material == materialmsg.CABLE:
            speed = "full_speed"
        elif material == materialmsg.METAL:
            speed = "full_speed"
        else:
            speed = "None"
            rospy.logerr("Error: no valid material given")
        return speed

    def stop_grinder(self):
        self.cobot.tool_controller.run_shaftgrinder(0.05, "slow")
        self.cobot.F_max = 80

    def timeout_check(self, timeout):
        """
        Checks current skill duration against given timeout.
        Args:
            timeout [float] : timeout from given goal.

        Returns: Bool : True if there is time left for the skill. False if duration exceeded timeout time.

        """
        if timeout - (time.time() - self._start_time) > 0:
            return True
        else:
            return False

    @property
    def result(self):
        """
        Getter Method for result.
        """
        return self._result

    def create_result(self, runtime, result):
        """Parse result to ``Skill Result message``

        Args:
            runtime (float): runtime to execute the action
            result (optional, int):

        Returns:
            SkillResult: skill result as publishable ROS-status message
        """
        result_msg = SkillResult()
        result_msg.cobot_state = self._feedback.state
        result_msg.result = result
        result_msg.runtime = runtime

        return result_msg

    def updatetf(self, PrePoseA, PrePoseB, CuttingPoseA, CuttingPoseB):
        """
        Updates the coordinate transformation matrix for the robot/shaftgrinder according to the device and cutting position.
        Args:
            y [float] : y coordinate for Cutting Pose A

        Returns: None

        """
        PrePoseAtf = self.transform_matrix(Shaftgrinder_tf_1, PrePoseA)
        PrePoseBtf = self.transform_matrix(Shaftgrinder_tf_1, PrePoseB)
        CuttingPoseAtf = self.transform_matrix(Shaftgrinder_tf_1, CuttingPoseA)
        CuttingPoseBtf = self.transform_matrix(Shaftgrinder_tf_1, CuttingPoseB)

        # if self.device_type == 3:  # 3 for PC_tower
        #     if (y - self.get_y_of_device_center()) < 0.06:
        #         T1 = np.array([[0.58655, 0.78331, 0.20586],
        #                        [-0.80989, 0.56915, 0.14191],
        #                        [-0.00601, -0.24997, 0.96823]])
        #
        #         PrePoseAtf = self.transform_matrix(T1, PrePoseA)
        #         PrePoseBtf = self.transform_matrix(T1, PrePoseB)
        #         CuttingPoseAtf = self.transform_matrix(T1, CuttingPoseA)
        #         CuttingPoseBtf = self.transform_matrix(T1, CuttingPoseB)
        #
        #     else:
        #         T2 = np.array([[-0.59428, 0.7683, -0.23779],
        #                        [0.68188, 0.6381, 0.35757],
        #                        [0.42646, 0.05035, -0.90311]])
        #         PrePoseAtf = self.transform_matrix(T2, PrePoseA)
        #         PrePoseBtf = self.transform_matrix(T2, PrePoseB)
        #         CuttingPoseAtf = self.transform_matrix(T2, CuttingPoseA)
        #         CuttingPoseBtf = self.transform_matrix(T2, CuttingPoseB)

        # else:
        #     PrePoseAtf = PrePoseA
        #     PrePoseBtf = PrePoseB
        #     CuttingPoseAtf = CuttingPoseA
        #     CuttingPoseBtf = CuttingPoseB

        return PrePoseAtf, PrePoseBtf, CuttingPoseAtf, CuttingPoseBtf

    @staticmethod
    def transform_matrix(tfmatrix, pose):
        posetf = pose
        # print(posetf)
        posetf.A[:3, :3] = tfmatrix @ pose.A[:3, :3]
        # posetf[3]
        return posetf

    def update_feedback_msg(self, msg: str):
        rospy.loginfo(self._log_str(msg))
        cur_state = self._feedback.state.state
        self._feedback.state = self.cobot.cobot_state_msg(msg, None)
        self._feedback.state.state = cur_state
        self.publish_feedback()

    def publish_feedback(self):
        try:
            self._as.publish_feedback(self._feedback)
        except AttributeError:
            rospy.logerr(f"failed to send feedback-message")

    @staticmethod
    def _log_str(msg):
        return f"Cutting -> {msg}"

    @staticmethod
    def calc_EE_cutting_pose(B_normal, B_p_location, T_C_E=None, B_y_axis=np.r_[0., 1., 0.], depth=-5e-2):
        r"""Calculate cutting pose via cutting with the robot into the surface

        The surface is defined by its normal, while the ``safety_distance``
        describes the hovering height that the robot should be steered to

        Args:
            B_normal (np.ndarray): normal vector in base frame :math:`{}^{B}{\bf n} \in \mathbb{R}^{3}`
            B_p_location(np.ndarray): goal-position in base-frame :math:`{}^{B}{\bf p}_{BC} \in \mathbb{R}^{3}`.
            B_y_axis (np.ndarray, optional): reference y-axis to generate pose / right-hand CS in base-frame. Defaults to :math:`{}^{B}{\bf e}_{y}`.
            T_C_E(sm.SE3 or None, optional): homogeneous transformation from end-effector to tool-frame. Defaults to None.
            depth (float, optional): cutting depth. Defaults to -0.05.

        Returns:
            sm.SE3: pre-pose of end-effector wrt to base-frame.

        Raises:
            AssertionError: if safety_distance is negative
        """
        assert depth <= 0.0, "cutting depth is positive"
        return sm.SE3(B_normal * depth) @ hrr_cm.calc_EE_goal_pose(B_normal=B_normal, B_p_location=B_p_location,
                                                                   B_y_axis=B_y_axis, T_C_E=T_C_E)

    def planner_move(self, pose, result=SkillResult()):
        """
        Uses the planner action with obstacle avoidance to move to the Pre-Pose of Point A.

        Todo:

            * test this
            * implement this

        Args:
            pose [SE3] : goal pose

        Returns:
            result(SkillResult): action result
        """

        if self._as.is_preempt_requested():
            rospy.loginfo('Preempted')
            self.cobot.tool_controller.rpm = 0
            self.cobot.tool_controller.tool = "None"
            self._as.set_preempted()
            result.result = SkillResult.FAILED

        elif not self.action_sever_valid or self.cobot.state == "FAILURE":
            self.cobot.tool_controller.rpm = 0
            self.cobot.tool_controller.tool = "None"
            rospy.logerr(f"robot in ERROR state")
            result.result = SkillResult.FAILED

        pre_location_quat = hrr_cm.homog2quat(pose)
        pre_location_pos = hrr_cm.homog2pos(pose)

        # go to start pose
        GoalPose = PlannerGoal()
        GoalPose.goal_pose.header.frame_id = "map"
        GoalPose.goal_pose.header.stamp = rospy.Time.now()
        # pose for the planner

        # position of end_configuration
        GoalPose.goal_pose.pose.position.x = pre_location_pos[0]
        GoalPose.goal_pose.pose.position.y = pre_location_pos[1]
        GoalPose.goal_pose.pose.position.z = pre_location_pos[2]

        # orientation of end_configuration
        GoalPose.goal_pose.pose.orientation.x = pre_location_quat.x
        GoalPose.goal_pose.pose.orientation.y = pre_location_quat.y
        GoalPose.goal_pose.pose.orientation.z = pre_location_quat.z
        GoalPose.goal_pose.pose.orientation.w = pre_location_quat.w

        self._planner_client.send_goal(GoalPose)
        self._planner_client.wait_for_result()  # timeout can be added here

        if np.linalg.norm(self.cobot.sns_pos - self.cobot.T_des.t) < 0.002 and self.cobot.state is None:
            self._result.skill_result.result = SkillResult.UNKNOWN
        else:
            self._result.skill_result.result = SkillResult.FAILED
