#!/usr/bin/env python3
"""
Motion Planner Interface
----------------------------

This small wrapper intends to communicate with Matlab to trigger planning requests
and receive feedback in the form of joint-trajectory messages


ROS-parameters
^^^^^^^^^^^^^^^^^^^^^^^^^

All ROS-parameters are prepended by a ``{cobot_prefix}``, usually ``/hrr_cobot/``

============================================= ================================================================================================================
Parameter                                     notes
============================================= ================================================================================================================
``motion_planner_trajectory_topic``           ROS-topic name that contains the planned trajectory
``motion_planner_joint_goal_topic``           ROS-topic name to publish the desired goal joint configuration
``motion_planner_goal_pose_topic``            ROS-topic name to publish the desired Cartesian goal-pose
``call_joint_trajectory_planner_service``     as the ROS-service name that allows to trigger to plan a trajectory in joint-space
``call_cartesian_trajectory_planner_service`` as the ROS-service name that allows to trigger to plan a trajectory from current pose to a Cartesian goal-pose
============================================= ================================================================================================================
"""
# built in imports
from typing import Optional, Union
import pathlib

# 3rd party imports
import numpy as np
import quaternion
import scipy.io

# ROS imports
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

# module imports
from hrr_common.ros_utils.conversions import np2vec3, np2joint_trajectory_point, np2quat
from hrr_common.ros_utils.helper_handles import get_param
from hrr_common.ros_utils.templates import RosBaseHandle

__all__ = ["MotionPlannerIf"]


class MotionPlannerIf(RosBaseHandle):

    def __init__(self):
        self._sub_joint_trajectory = None  # type: Optional[rospy.Subscriber]
        self._pub_joint_goal = None  # type: Optional[rospy.Publisher]
        self._pub_cartesian_goal = None  # type: Optional[rospy.Publisher]
        self._jnt_trigger_srv = None  # type: Optional[rospy.ServiceProxy]
        self._pose_trigger_srv = None  # type: Optional[rospy.ServiceProxy]
        self._current_joint_traj = None  # type: Optional[JointTrajectory]
        self._joint_traj_from_file = None  # type: Optional[np.ndarray]
        self._start_eng = None
        self._mb_eng = None
        self._mb_working_dir = None

    def init_ros(self, planned_joint_trajectory_name,
                 joint_goal_topic_name, goal_pose_topic_name,
                 trigger_joint_srv_name, trigger_pose_srv_name,
                 **__):
        if planned_joint_trajectory_name:
            self._sub_joint_trajectory = rospy.Subscriber(planned_joint_trajectory_name,
                                                          JointTrajectory, self.traj_cb,
                                                          queue_size=10)
        if joint_goal_topic_name:
            self._pub_joint_goal = rospy.Publisher(joint_goal_topic_name, JointTrajectoryPoint, queue_size=10)
        if goal_pose_topic_name:
            self._pub_cartesian_goal = rospy.Publisher(goal_pose_topic_name, PoseStamped, queue_size=10)
        if trigger_joint_srv_name:
            self._jnt_trigger_srv = rospy.ServiceProxy(trigger_joint_srv_name, Trigger)
        if trigger_pose_srv_name:
            self._pose_trigger_srv = rospy.ServiceProxy(trigger_pose_srv_name, Trigger)

    @property
    def can_plan(self):
        return self._pub_joint_goal is not None and self._jnt_trigger_srv is not None

    @property
    def can_plan_cartesian(self):
        return self._pub_cartesian_goal is not None and self._pose_trigger_srv is not None

    @property
    def has_matlab(self):
        return self._mb_eng is not None

    def init_matlab(self):
        """Initialize connection to matlab instance if possible"""
        try:
            import matlab.engine
        except (ModuleNotFoundError, ImportError) as e:
            rospy.logerr(f"could not find matlab engine. No direct interface to motion planner available")
            return
        rospy.loginfo(f'[{rospy.get_name()}] Initialize MATLAB-Motion planner @ {self._mb_working_dir}')
        try:
            self._start_eng = matlab.engine.start_matlab()
            self._mb_eng = matlab.engine.connect_matlab()
            rospy.loginfo(f'[{rospy.get_name()}] connected to Matlab')
            self._mb_eng.cd(str(self._mb_working_dir), nargout=0)
            self._mb_eng.HrRecyclerDataImport(nargout=0)
            rospy.loginfo(f'[{rospy.get_name()}] MATLAB-Initialization done')
        except Exception as e:
            rospy.logerr(f"failed to initialize Matlab-connection due to: {e}")

    def assert_matlab(self):
        if self._mb_eng is None:
            self.init_matlab()
        assert self._mb_eng is not None, "no connection to matlab planning instance."

    def traj_cb(self, msg):
        """
        receiving callback for message from (Matlab-)motion-planner

        Args:
            msg(JointTrajectory): planned joint trajectory message received from planner

        Returns:
        """
        if self._current_joint_traj is not None:
            rospy.logwarn("replacing current joint trajectory")
        self._current_joint_traj = msg

    def get_trajectory(self):
        """return current trajectory and set to None to prevent sending the same trajectory twice"""
        if self._current_joint_traj:
            return self._current_joint_traj
        self._current_joint_traj = None

    @classmethod
    def _from_ros(cls, cobot_prefix, **kwargs):
        out = cls()
        out.init_ros(
            planned_joint_trajectory_name=rospy.get_param(f"{cobot_prefix}motion_planner_trajectory_topic", ""),
            joint_goal_topic_name=rospy.get_param(f"{cobot_prefix}motion_planner_joint_goal_topic", ""),
            goal_pose_topic_name=rospy.get_param(f"{cobot_prefix}motion_planner_goal_pose_topic", ""),
            trigger_joint_srv_name=rospy.get_param(f"{cobot_prefix}call_joint_trajectory_planner_service", ""),
            trigger_pose_srv_name=rospy.get_param(f"{cobot_prefix}call_cartesian_trajectory_planner_service", "")
        )
        out._mb_working_dir = pathlib.Path(get_param(f"{cobot_prefix}matlab_working_dir",
                                                     '/home/schrottgott/_ros/hr_recycler_ws/src/planner/src'))
        if not out._mb_working_dir.exists():
            rospy.logerr(f'source directory of motion planner {out._mb_working_dir} does not exist')
            return out
        if get_param(f"{cobot_prefix}initialize_matlab", False):
            out.init_matlab()
        return out

    def _return_trajectory(self, res: TriggerResponse) -> JointTrajectory:
        if res.success:
            out = None
            while out is None:
                out = self.get_trajectory()
            return out

    def pose_to_trajectory(self, T_B_E) -> Union[JointTrajectory, None]:
        """
        Generate joint trajectory from matlab-motion planner

        Args:
            T_B_E(np.ndarray): current FK of robot at robot flange. (4 x 4)

        Returns:
            JointTrajectory or None: a JointTrajectory-message that can be forwarded to the robot
        """
        self.assert_matlab()
        p = T_B_E[0:3, 3]
        q = quaternion.from_rotation_matrix(T_B_E[0:3, 0:3])
        scipy.io.savemat(str(self._mb_working_dir / 'arrdata.mat'), mdict={'A': np.r_[p, q.w, q.x, q.y, q.z]})
        rospy.logdebug(f'[{rospy.get_name()}] Calculating SFD ...')
        self._mb_eng.HrRecyclerPerecptioPipeline(nargout=0)
        rospy.logdebug(f'[{rospy.get_name()}] SDF is ready. Run motion planning')
        self._mb_eng.HrRecyclerPlanner(nargout=0)
        rospy.logwarn(f'[{rospy.get_name()}] Planner is done and execution is running ...')
        try:
            jnt_traj = scipy.io.loadmat(str(self._mb_working_dir / 'jnt.mat'))
            return jnt_traj['jointWayPoints']
        except FileNotFoundError:
            rospy.logerr("could not find joint trajectory file")
        except KeyError:
            rospy.logerr("result file does not contain key `jointWayPoints`")

    def get_joint_trajectory_to_joint_configuration(self, q_des: np.ndarray) -> Union[JointTrajectory, None]:
        """
        Generate joint trajectory from matlab-motion planner

        Args:
            q_des(np.ndarray): joint-goal

        Returns:
            JointTrajectory or None: a JointTrajectory-message that can be forwarded to the robot
        """
        self.assert_matlab()
        scipy.io.savemat(str(self._mb_working_dir / 'q_des.mat'), mdict={'q': q_des})
        rospy.logdebug(f'[{rospy.get_name()}] Calculating SFD ...')
        self._mb_eng.HrRecyclerPerecptioPipeline(nargout=0)
        rospy.logdebug(f'[{rospy.get_name()}] SDF is ready. Run motion planning')
        self._mb_eng.HrRecyclerPlanner(nargout=0)
        rospy.logwarn(f'[{rospy.get_name()}] Planner is done and execution is running ...')
        try:
            jnt_traj = scipy.io.loadmat(str(self._mb_working_dir / 'jnt.mat'))
            return jnt_traj['jointWayPoints']
        except FileNotFoundError:
            rospy.logerr("could not find joint trajectory file")
        except KeyError:
            rospy.logerr("result file does not contain key `jointWayPoints`")


    def get_joint_trajectory_to_pose(self, T_B_E: np.ndarray):
        if isinstance(T_B_E, np.ndarray):
            if T_B_E.shape != (4, 4):
                rospy.logerr(f"cannot generate a joint-trajectory to pose of shape {T_B_E.shape}")
                return
        else:
            try:
                T_B_E = T_B_E.A
            except AttributeError:
                rospy.logerr(f"cannot generate a joint-trajectory for goal-pose {T_B_E}")
                return
        return self.pose_to_trajectory(T_B_E)

    def plan_joint_trajectory(self, q_des) -> JointTrajectory:
        assert self._pub_joint_goal is not None, "initialize current handle first"
        self._pub_joint_goal.publish(np2joint_trajectory_point(q_des))
        return self._return_trajectory(self._jnt_trigger_srv(TriggerRequest()))

    def plan_cartesian_trajectory(self, p_des: np.ndarray, quat_des: np.quaternion,
                                  frame_id: str = "") -> JointTrajectory:
        assert self._pub_cartesian_goal is not None, "initialize current handle first"
        msg = PoseStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = frame_id
        msg.pose.position = np2vec3(p_des)
        msg.pose.orientation = np2quat(quat_des)
        self._pub_cartesian_goal.publish(msg)
        return self._return_trajectory(self._pose_trigger_srv(TriggerRequest()))
