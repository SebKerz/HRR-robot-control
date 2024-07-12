__all__ = ["cancel_action", "fix_prefix", "get_param", "fix_ns"]

import rospy


def cancel_action(action_client, action_topic):
    try:
        action_client.cancel(action_topic)
    except AttributeError:
        action_client.stop()


def fix_prefix(prefix):
    if prefix == "~":
        return prefix
    if prefix[-1] == "/":
        return prefix
    return f"{prefix}/"


def fix_ns(ns):
    if ns == "~":
        return ns
    if ns[-1] != "/":
        ns = f"{ns}/"
    if ns[0] != "/":
        ns = f"/{ns}"
    return ns


def get_param(param_name, default=None):
    """
    Get ROS-parameter function

    Args:
        param_name(str): ros-parameter name
        default(Any): default value for ROS-parameter lookup

    Returns:
        Any: value for current ROS-parameter name
    """
    if default is None:
        return rospy.get_param(param_name)
    try:
        return rospy.get_param(param_name)
    except KeyError:
        rospy.loginfo(f"[{rospy.get_name()}] "
                      f"could not get ROS-parameter {param_name.replace('~', f'{rospy.get_name()}/')}. "
                      f"Use default {default} instead.")
        return default
