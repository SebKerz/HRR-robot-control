#!/usr/bin/env python3
"""
Simple Joint State Publisher
------------------------------------------------

original implementation is python2 based and thus suffers from reading the URDF
https://github.com/vecnatechnologies/JacoROS/blob/master/urdf_tools/joint_state_publisher/joint_state_publisher

This simple python handle circumvents this issue by giving a good old sh** about the URDF
and instead simply plays a man-in-the-middle to publish joint state data from other publishers.

By default, it publishes empty joint states of all missing elements.
"""

from typing import List, Set
import numpy as np

import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

from hrr_common.ros_utils.helper_handles import get_param, fix_prefix, fix_ns

__all__ = ["CompleteJointStatePublisher", "CompleteJointStateSubscriber"]


class CompleteJointStateSubscriber(object):

    def __init__(self, parameter_prefix=None):
        self._subs = []  # type: List[rospy.Subscriber]
        self._srvs = []  # type: List[rospy.ServiceProxy]
        self._joint_names = []  # type: List[str]
        self._topic_names = set()  # type: Set[str]        
        self._q_pos = np.array([], dtype=float)
        self._q_vel = np.array([], dtype=float)
        self._q_eff = np.array([], dtype=float)
        self._t_rec = np.array([], dtype=float)
        self._set_to_zero_time = 0.0
        self._thread_blocker = False
        if parameter_prefix is not None:
            parameter_prefix = fix_prefix(parameter_prefix)
            try:
                self.init(
                    joint_state_topics=get_param(f"{parameter_prefix}source_list"),
                    joint_names=get_param(f"{parameter_prefix}joint_names", self._joint_names),
                    set_to_zero_time=get_param(f"{parameter_prefix}time_to_reset_joints", self._set_to_zero_time))
            except KeyError as e:
                rospy.logerr(
                    f"could not initialize {CompleteJointStateSubscriber.__name__}@{rospy.get_name()} due to {e}")

    @property
    def initialized(self) -> bool:
        """check if current ROS instance is initialized, i.e. if there is at least one subscriber
        
        Returns:
            bool: if instance is properly initialized
        """
        return len(self._subs) > 0

    def init_subs(self, joint_state_topics) -> None:
        """
        Initialize subscribers from list of topic names and add name to topic_names member variable

        Args:
            joint_state_topics(List[str]): list of ROS-topics to subscribe from.
        """
        for joint_topic in set(joint_state_topics) - self._topic_names:
            self._subs.append(rospy.Subscriber(fix_ns(joint_topic), JointState, self._joint_cb))
            self._topic_names.add(joint_topic)
            self._topic_names.add(fix_ns(joint_topic))

    def init_subs_from_parameter(self) -> None:
        """call :py:meth:`~init_subs` from ROS-parameter-server, e.g.
        """
        self.init_subs(get_param("~source_list"), [])

    def _update_subs_srv(self, *_):
        self.init_subs_from_parameter()
        return TriggerResponse(success=True,
                               msg=f"run subscription update from parameter `{rospy.get_name()}/source_list")

    def init(self, joint_state_topics, joint_names=None, set_to_zero_time=0.0, update_service_name=None) -> None:
        """Actual init function, that requires the ROS-dependent arguments to be set

        Args:
            joint_state_topics(List[str]): list of ROS-topics to subscribe from.
            joint_names(List[str], optional): joint names that are later set from other nodes. 
            set_to_zero_time(float, optional): time difference after which a joint value should be reset if no new data is received
            update_service_name(str, optional): name for update service name. Defaults to None.
        """
        self.init_subs(joint_state_topics)
        self._set_to_zero_time = set_to_zero_time
        if isinstance(joint_names, (list, tuple)):
            self._joint_names = joint_names
            self._q_pos.resize((len(joint_names), ))
            self._q_vel.resize((len(joint_names), ))
            self._q_eff.resize((len(joint_names), ))
            self._t_rec.resize((len(joint_names), ))
        if update_service_name:
            self._srvs.append(rospy.ServiceProxy(update_service_name, Trigger))

    def _joint_cb(self, msg):
        """
        Generic call-back for arbitrary joint-state publisher
        In case a new joint-message is spawned / added, the data is 

        Args:
            msg (JointState): joint-state from whatever ROS-node
        """
        for i, name in enumerate(msg.name):
            try:
                q_idx = self._joint_names.index(name)
            except ValueError:
                rospy.logdebug(f"add new joint {name} to list")
                while self._thread_blocker:
                    rospy.sleep(1e-3)
                self._thread_blocker = True
                self._joint_names.append(name)
                self._q_pos.resize((len(self._joint_names), ))
                self._q_vel.resize((len(self._joint_names), ))
                self._q_eff.resize((len(self._joint_names), ))
                self._t_rec.resize((len(self._joint_names), ))
                q_idx = self._joint_names.index(name)
                self._thread_blocker = False
            try:
                self._q_pos[q_idx] = msg.position[i]
                self._t_rec[q_idx] = rospy.get_time()
            except IndexError:
                rospy.logerr(f"index {q_idx} for joint {name} is not in current joint-list")
            try:
                self._q_vel[q_idx] = msg.velocity[i]
            except IndexError:
                pass
            try:
                self._q_eff[q_idx] = msg.effort[i]
            except IndexError:
                pass


    def check_for_reset(self):
        """Check for current entries, which have not been updated for longer and set their values to 0.0"""
        if self._set_to_zero_time > 0:
            reset_idx = np.where(rospy.get_time() - self._t_rec > self._set_to_zero_time)
            self._q_pos[reset_idx] = 0.0
            self._q_vel[reset_idx] = 0.0
            self._q_eff[reset_idx] = 0.0

    @property
    def joint_msg(self):
        """get current data as a ``JointState``-message, ready to be published"""
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.name = self._joint_names
        msg.position = self._q_pos.tolist()
        msg.velocity = self._q_vel.tolist()
        msg.effort = self._q_eff.tolist()
        return msg


class CompleteJointStatePublisher(CompleteJointStateSubscriber):

    def __init__(self, parameter_prefix=None):
        self._r = None  # type: Optional[rospy.Rate()]
        self._publisher = None  # type: Optional[rospy.Publsiher]
        super(CompleteJointStatePublisher, self).__init__(parameter_prefix=parameter_prefix)
        if parameter_prefix is not None:
            parameter_prefix = fix_prefix(parameter_prefix)
            try:
                self.init_publisher(
                    rate=get_param(f"{parameter_prefix}publish_rate"),
                    publisher_topic_name=get_param(f"{parameter_prefix}publisher_topic_name", "/joint_states")
                )
            except KeyError as e:
                rospy.logerr(
                    f"could not initialize {CompleteJointStatePublisher.__name__}@{rospy.get_name()} due to {e}")

    @property
    def initialized(self):
        return super().initialized and self._publisher is not None

    def init_publisher(self, rate, publisher_topic_name) -> None:
        """Actual init function, that requires the ROS-dependent arguments to be set

        Args:
            rate(float): list of ROS-topics to subscribe from.
            joint_names(List[str], optional): joint names that are later set from other nodes. 
            set_zero_time(float, optional): time difference after which a joint value should be reset if no new data is received
        """
        if rate <= 0.0:
            rospy.logerr("rate must be positive")
            return
        self._r = rospy.Rate(rate)
        self._publisher = rospy.Publisher(publisher_topic_name, JointState, queue_size=100)

    @classmethod
    def from_ros(cls, parameter_prefix):
        """Initialize instance from ROS-parameters

        Args:
            parameter_prefix (str): ROS-parameter namespace
        """
        out = cls()
        parameter_prefix = fix_prefix(parameter_prefix)
        try:
            out.init(joint_state_topics=get_param(f"{parameter_prefix}source_list"),

                     joint_names=get_param(f"{parameter_prefix}joint_names", out._joint_names),
                     set_to_zero_time=get_param(f"{parameter_prefix}time_to_reset_joints", out._set_to_zero_time))
        except KeyError as e:
            rospy.logerr(f"could not initialize {CompleteJointStatePublisher.__name__} @{rospy.get_name()} due to {e}")
        return out

    def update(self):
        """Update function step"""
        self.check_for_reset()
        self._publisher.publish(self.joint_msg)
        self._r.sleep()
