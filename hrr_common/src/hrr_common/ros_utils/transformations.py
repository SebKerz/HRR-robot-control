from typing import List, Optional, Union, Tuple
import numpy as np
import quaternion
import spatialmath as sm

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3Stamped, Vector3, PoseStamped

from hrr_common.utils.spatial import quaternion_rotate
from .conversions import vec32np, np2vec3, quat2np

__all__ = ["get_empty_msg_stamped", "get_tf_bare", "TransformBuffer", "TfHandler"]


def get_empty_msg_stamped(reference_frame: str, object_frame: str) -> TransformStamped:
    msg = TransformStamped()
    msg.header.frame_id = reference_frame
    msg.child_frame_id = object_frame
    msg.transform.rotation.w = 1.0
    return msg


def get_tf_bare(listener, target_frame, source_frame, trials=100, ts=1e-3, time=rospy.Time(0)):
    r"""
    Safe tf2-ros lookup function.

    denoting the target-frame as :math:`B` and source-frame as :math:`A`,
    this function uses the ``tf2_ros`` listener to get the transformation
    :math:`{}^{B} {\bf T}_{A}`, i.e. the transformation from an arbitrary
    frame to the desired reference :math:`B`.

    Args:
        listener (tf2_ros.TransformListener): tf2-ros listener object
        target_frame (str): reference frame
        source_frame (str): frame
        trials (int, optional): number of retrials. Defaults to 100.
        ts ([type], optional):  sleep time after an exception is encountered. Defaults to 1e-3.
        time(rosy.Time, optional): ros time. Defaults to 0.

    Returns:
        np.ndarray, np.quaternion: math:`{A}^{p}_{BA}` and :math:`{\bf q}^{A}_{B}`.
    """
    for _ in range(trials):
        try:
            trans_msg = listener.lookup_transform(
                target_frame=target_frame, source_frame=source_frame, time=time)
            t = trans_msg.transform
            return (np.r_[t.translation.x, t.translation.y, t.translation.z],
                    np.quaternion(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(ts)
            continue
    return np.zeros(3), np.quaternion(1., 0., 0., 0.)


class TfHandler:
    """Helper Class for transformation trees / multiple frames in use

    
    Allows to look up transformation according to :py:meth:`~get_tf_bare`
    and broadcast transformations to ROS / TF2 easily.
    """
    buffer = []  # type: List[tf2_ros.Buffer]
    listener = []  # type: List[tf2_ros.TransformListener]

    def __init__(self, add=False):
        try:
            self._update_buffers(add)
        except IndexError:
            self._update_buffers(True)
        self.br = tf2_ros.TransformBroadcaster()

    def _update_buffers(self, add=False):
        if len(self.buffer) > 0 and not add:
            self.buffer[-1] = tf2_ros.Buffer()
        else:
            self.buffer.append(tf2_ros.Buffer())
        if len(self.listener) > 0 and not add:
            self.listener[-1] = tf2_ros.TransformListener(self.buffer[-1])
        else:
            self.listener.append(tf2_ros.TransformListener(self.buffer[-1]))

    def T_A_B(self, A, B, time=rospy.Time(0), i=-1, max_dT=10.0):
        """
        Note:

            In short: **reads alphabetically, but order of rotation is from B -> A**

        get Transformation from B to A, i.e. :math:`{}^{A}{\bf T}_{B}`
        in the form of translation and rotation, i.e.
        :math:`{}^{A}{p}_{AB}` and :math:`{\bf q}^{A}_{B}`.
        
        Args:
            A (str):  frame A (source_frame)
            B (str):  frame B (target_frame)
            time(optional, rospy.Time): optional ros-message time stamp. Defaults to rospy.Time(0) -> get latest transformation.
            i (int, optional): index to current buffer. Defaults to -1.
            max_dT(float,optional): maximum time delta for received data.

        Returns:
            np.ndarray, np.quaternion: math:`{A}^{p}_{ABA}` and :math:`{\bf q}^{A}_{B}`.
        """
        if time.nsecs != 0. and time.secs != 0.:
            if (rospy.get_rostime() - time).to_sec() > max_dT:
                rospy.logerr_once(
                    f"dt {(rospy.get_rostime() - time).to_sec()} is above threshold {max_dT} -> will use current instead")
                time = rospy.Time(0)
        try:
            return get_tf_bare(self.buffer[i], target_frame=A, source_frame=B, time=time)
        except IndexError:
            self._update_buffers(True)
            return get_tf_bare(self.buffer[i], target_frame=A, source_frame=B, time=time)

    def T_A_B_adjoint(self, **kwargs) -> np.ndarray:
        """Get Adjoint from B->A to transform velocity along coordinate frames"""
        p_A_AB, q_A_B = self.T_A_B(**kwargs)
        return sm.base.tr2adjoint(np.block([
            [quaternion.as_rotation_matrix(q_A_B), p_A_AB[:, None]],
            [np.zeros(3), 1.0]
        ]))

    def send_tf(self, tf_msg):
        if tf_msg.child_frame_id != "":
            tf_msg.header.stamp = rospy.get_rostime()
            self.br.sendTransform(tf_msg)

    def A_vector(self, frame_A, frame_B, B_vector, **kwargs):
        """
        Transform a numpy array / vector from current reference frame :math:`B` to :math:`A`.

        Args:
            frame_A: desired reference frame
            frame_B(str): reference frame of current vector
            B_vector(np.ndarray): vector in frame B :math:`{}^{B}{\bf x}

        Returns:
            np.ndarray: :math:`{}^{A}{\bf x}
        """
        assert B_vector.shape == (3,), f"cannot transform vectors of dimension {B_vector.shape} in this function"
        q_A_B = self.T_A_B(frame_A, frame_B, **kwargs)[1]
        return quaternion_rotate(q_A_B, B_vector)

    def A_vectors(self, *B_vectors: np.ndarray, frame_A=None, frame_B=None, **kwargs) -> Tuple[np.ndarray, ...]:
        """transform multiple vectors. Remainder identical to :py:meth:`~A_vector`"""
        assert frame_B is not None and frame_B is not None, "please define reference frames"
        if frame_B == frame_A:
            return B_vectors
        q_A_B = self.T_A_B(frame_A, frame_B, **kwargs)[1]
        return tuple([quaternion_rotate(q_A_B, B_x) for B_x in B_vectors])

    def A_vector_from_msg(self, B_vector_msg, frame_A, frame_B=None):
        """

        Args:
            B_vector_msg(Union[Vector3Stamped, Vector3]:
            frame_A(str): desired reference frame
            frame_B(optional, str): reference frame of message

        Returns:
            np.ndarray: :math:`{}^{A}{\bf x}`
        """
        try:
            frame_B = B_vector_msg.header.frame_id
            try:
                B_vector_msg = B_vector_msg.vector
            except AttributeError:
                B_vector_msg = B_vector_msg.point
        except AttributeError:
            assert frame_B is not None, "received empty reference frame "
            assert isinstance(B_vector_msg, Vector3), f"expected Vector3, received {type(B_vector_msg)}"
        return self.A_vector(frame_A=frame_A, frame_B=frame_B, B_vector=vec32np(B_vector_msg))

    def A_pose(self, msg, frame_A) -> sm.SE3:
        """
        get pose stamped in arbitrary frame

        Args:
            msg(PoseStamped):
            frame_A(str): desired frame for received pose

        Returns:
            sm.SE3: pose w.r.t. frame B as homog. coordinates
        """
        T_B = sm.SE3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        T_B.A[:3, :3] = quaternion.as_rotation_matrix(quat2np(msg.pose.orientation))
        frame_B = msg.header.frame_id
        if frame_B == frame_A or frame_B == "":
            return T_B
        p, q = self.T_A_B(A=frame_A, B=frame_B, time=msg.header.stamp)
        T_A_B = sm.SE3(*p)
        T_A_B.A[:3, :3] = quaternion.as_rotation_matrix(q)
        return T_A_B @ T_B

    def A_velocity(self, frame_A, frame_B, B_velocity, **kwargs):
        """map velocity from frame B into frame A"""
        return self.T_A_B_adjoint(A=frame_A, B=frame_B, **kwargs) @ B_velocity


class TransformBuffer:
    """
    Helper handle to track transformations and evaluate
    """

    def __init__(self, object_frame, ref_frame, buf_size=1000):
        self._cnt = 0
        self._rolls = 0
        self.N = buf_size
        self.pos_buffer = np.zeros((buf_size, 3))
        self.quat_buffer = np.zeros(buf_size, dtype=np.quaternion)
        self._sensor_time = np.zeros(buf_size)
        self.time = np.zeros(buf_size)
        self._ref_frame = ref_frame
        self._child_frame = object_frame
        if self._ref_frame is not None:
            self._tf_buffer = tf2_ros.buffer()
            self._tf_listener = tf2_ros.transformlistener(self._tf_buffer)

    def check_transform(self):
        try:
            trans = self._tf_buffer.lookup_transform(
                self._ref_frame, self._child_frame, rospy.Time(0))
            self.pos_buffer[0, :] = np.r_[trans.transform.translation.x,
                                          trans.transform.translation.y,
                                          trans.transform.translation.z]
            self.quat_buffer[0] = np.quaternion(trans.transform.rotation.w,
                                                trans.transform.rotation.x,
                                                trans.transform.rotation.y,
                                                trans.transform.rotation.z)
            self.time[self._cnt] = rospy.Time(
                trans.header.stamp.secs, trans.header.stamp.nsecs).to_sec()
            self._cnt += 1
            self.pos_buffer = np.roll(self.pos_buffer, 1, axis=0)
            self.quat_buffer = np.roll(self.quat_buffer, 1)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        except (AttributeError, ValueError, TypeError):
            return
        if self._cnt >= self.N:
            self._cnt = 0
            self._rolls += 1

    def reset(self):
        self.pos_buffer.fill(0.0)
        self.quat_buffer.fill(0.0)
        self._cnt = 0
        self._rolls = 0

    @property
    def data_idxs(self):
        return np.where(self.quat_buffer > 0)[0]
