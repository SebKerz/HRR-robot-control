#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolRequest

__all__ = ["start_record", "stop_record"]


def start_record(record_srv=None):
    if record_srv is None:
        record_srv = rospy.ServiceProxy("/record", SetBool)
    try:
        res = record_srv(SetBoolRequest(data=True))
    except rospy.ServiceException as e:
        rospy.logerr_once(f"could not start ros-bag recording: {e}")
        return
    if not res.success:
        rospy.logwarn('could not start:', res.message)
        stop_record()
        record_srv(SetBoolRequest(data=True))


def stop_record(record_srv=None):
    if record_srv is None:
        record_srv = rospy.ServiceProxy("/record", SetBool)
    try:
        res = record_srv(SetBoolRequest(data=False))
    except rospy.ServiceException as e:
        rospy.logerr_once(f"could not stop ros-bag recording: {e}")
        return
    if not res.success:
        rospy.logwarn('could not stop:', res.message)
