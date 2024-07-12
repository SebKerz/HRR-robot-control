import numpy as np
import quaternion
import rospy
from geometry_msgs.msg import Vector3, Point, Twist, Wrench, TwistStamped, WrenchStamped, Quaternion, Vector3Stamped
from trajectory_msgs.msg import JointTrajectoryPoint

__all__ = ["np2vec3", "np2twist", "np2wrench",
           "np2twist_stamped", "np2wrench_stamped",
           "vec32np", "twist2np", "wrench2np",
           "np2joint_trajectory_point", "joint_trajectory_point2np",
           "quat2np", "quat2rotmat", "np2quat", "rotmat2quat", "np2vector3stamped", "vector3stamped2np"]


def np2joint_trajectory_point(arr):
    msg = JointTrajectoryPoint()
    msg.positions = arr.tolist()
    return msg


def joint_trajectory_point2np(msg):
    # type: (JointTrajectoryPoint) -> np.ndarray
    return np.array(msg.positions)


def np2vec3(arr):
    # type: (np.ndarray) -> Vector3
    """numpy array to geometry message Vector3"""

    vec = Vector3()
    vec.x = arr[0]
    vec.y = arr[1]
    vec.z = arr[2]
    return vec


def np2twist(arr):
    """convert an array to a Twist"""
    assert arr.shape == (6,), f"dimension mismatch for twist and array {arr}"
    msg = Twist()
    msg.linear = np2vec3(arr[0:3])
    msg.angular = np2vec3(arr[3:6])
    return msg


def set_header(msg, frame):
    msg.header.frame_id = frame
    msg.header.stamp = rospy.get_rostime()
    return msg


def np2twist_stamped(arr, frame):
    msg = set_header(TwistStamped(), frame)
    msg.twist = np2twist(arr)
    return msg


def np2vector3stamped(arr, frame):
    msg = set_header(Vector3Stamped(), frame)
    msg.vector = np2vec3(arr)
    return msg


def vector3stamped2np(msg):
    vec32np(msg.vector)


def np2wrench(arr):
    """convert an array to a Wrench"""
    assert arr.shape == (6,), f"dimension mismatch for wrench and array {arr}"
    msg = Wrench()
    msg.force = np2vec3(arr[0:3])
    msg.torque = np2vec3(arr[3:6])
    return msg


def np2wrench_stamped(arr, frame):
    msg = set_header(WrenchStamped(), frame)
    msg.wrench = np2wrench(arr)
    return msg


def vec32np(vec):
    """
    Convert Vector3 to numpy array

    Args:
        vec(Vector3 or Point): geometry message vector3 type

    Returns:
        np.ndarray: numpy array version
    """
    return np.r_[vec.x, vec.y, vec.z]


def twist2np(twist):
    """
    Convert Twist to numpy array

    Args:
        twist(Twist): geometry message vector3 type

    Returns:
        np.ndarray: numpy array version
    """
    return np.r_[vec32np(twist.linear), vec32np(twist.angular)]


def wrench2np(wrench):
    """
    Convert Wrench to numpy array

    Args:
        wrench(Wrench): geometry message vector3 type

    Returns:
        np.ndarray: numpy array version
    """
    return np.r_[vec32np(wrench.force), vec32np(wrench.torque)]


def np2quat(q: np.quaternion) -> Quaternion:
    msg = Quaternion()
    msg.w = q.w
    msg.x = q.x
    msg.y = q.y
    msg.z = q.z
    return msg


def rotmat2quat(R: np.ndarray) -> Quaternion:
    q = quaternion.from_rotation_matrix(R)
    return np2quat(q)


def quat2np(msg: Quaternion) -> np.quaternion:
    return np.quaternion(msg.w, msg.x, msg.y, msg.z)


def quat2rotmat(msg: Quaternion) -> np.ndarray:
    return quaternion.as_rotation_matrix(quat2np(msg))
