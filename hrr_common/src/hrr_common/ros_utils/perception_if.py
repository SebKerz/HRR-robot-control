"""
Perception handlers and interfaces
----------------------------------------

This file mainly includes _PCL https://pointclouds.org/ interfaces and wrappers that are needed to
subscribe to output provided by the perception related partners within the project.

Please also refer to :ref:`PCLDoc`.

"""
from dataclasses import dataclass
import warnings

import matplotlib.pylab as plt
import numpy as np
import quaternion

import rospy
from geometry_msgs.msg import Quaternion, PoseArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header

__all__ = ["plot_pcl_array", "PclData", "get_pcl_center", "PoseArrayData"]


def ros_cloud_to_list(ros_cloud):
    """ Converts a ROS PointCloud2 message to a list of (x,y,z) or (x,y,z,d) samples
    where d encodes point cloud information, e.g. RGB

    Args:
        ros_cloud (PointCloud2): ROS PointCloud2 message

    Returns:
        list: all samples of the point cloud stored in a list
    """
    points_list = []
    try:
        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2], data[3]])
    except IndexError:
        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2], 0])
    return points_list


def ros_to_pcl(ros_cloud):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

    Args:
        ros_cloud (PointCloud2): ROS PointCloud2 message

    Returns:
        pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
    """
    import pcl
    pcl_data = pcl.PointCloud_PointXYZ()
    pcl_data.from_list(ros_cloud_to_list(ros_cloud))
    return pcl_data


def ros_to_array(ros_cloud):
    """
    Converts a ROS PointCloud2 message to a point array

    If ros_cloud contains data, e.g. rbg the return type is of shape (D x 4) otherwise (D x 3)

    Args:
        ros_cloud (PointCloud2): ROS PointCloud2 message

    Returns:
        np.ndarray: array of point cloud data
    """
    return np.array(ros_cloud_to_list(ros_cloud))


def plot_pcl_array(data, fig_kwargs=None, ax_kwargs=None):
    """ploting helper to visualize a point cloud data array in python

    Args:
        data (Union[np.ndarray, pcl.PointCloud_PointXYZRGB): point cloud data prefereably as a numpy-array
        fig_kwargs (dict or None, optional): figure kwargs. Defaults to None.
        ax_kwargs (dict or None, optional): axes kwargs. Defaults to None.

    Raises:
        TypeError: if data is neither numpy-array nor PCL RGB pointCloud

    Returns:
        tuple: figure and axis handle of given plot
    """

    if not isinstance(data, np.ndarray):
        import pcl
        if isinstance(data, pcl.PointCloud_PointXYZRGB):
            data = data.to_array()
        else:
            raise TypeError(f"unknown type {type(data)}")
    if fig_kwargs is None:
        fig_kwargs = dict()
    if ax_kwargs is None:
        ax_kwargs = dict(projection='3d')
    else:
        ax_kwargs['projection'] = '3d'
    fig = plt.figure(**fig_kwargs)
    ax = fig.add_subplot(**ax_kwargs)
    if data.shape[1] == 4:
        ax.scatter(data[:, 0], data[:, 1], data[:, 2], c=data[:, 3])
    else:
        ax.scatter(data[:, 0], data[:, 1], data[:, 2])
    return fig, ax


def get_polygon_center(polygon):
    """get_polygon_center [summary]

    Args:
        polygons (Union[list, tuple, np.ndarray]): list of polygon points (D x 3)

    Returns:
        np.ndarray: center point for a given polygon (3 x 1)
    """
    P = np.array(polygon)
    return np.mean(P, axis=0)


def get_polygon_orientation(polygon):
    if len(polygon) == 4:
        return np.zeros(3)
    else:
        raise NotImplementedError


def polygon_normal_to_pose(polygon, normal_vec):
    import spatialmath as sm
    T_des = sm.SE3(*get_polygon_center(polygon))
    T_des.A[:3, :3] = sm.SO3.OA(get_polygon_orientation, -normal_vec)
    return T_des


def get_pcl_center(pcl_array, z_min):
    return np.mean(pcl_array[np.where(pcl_array[:, 2] > z_min)], axis=0)[:3]


class PclData:
    """PCL subscription object

    Todo:
        needs additional filtering
    """

    def __init__(self, topic_name):
        """Perception handle to communicate with PCL data

        Args:
            topic_name (str): Pointcloud2 topic name
        """
        self.received_data = False
        self.data = None
        self._R_static = None
        self._t_static = None
        self._calibrated_static_data = None
        try:
            import pcl
            self._pcl_sub = rospy.Subscriber(topic_name, PointCloud2, self.pcl_cb)
        except (ModuleNotFoundError, ImportError):
            warnings.warn("could not import python-pcl. Please install and restart. No subscriber will be started")
            self._pcl_sub = None

    def pcl_cb(self, msg):
        self.data = ros_to_array(msg)
        self.received_data = self.data.shape[0] > 1

    @property
    def reference_data(self):
        """check if data is available in reference frame"""
        if self._calibrated_static_data is None:
            return self.data
        return self._calibrated_static_data

    def transformed_data(self, R, t):
        r"""Transform the PCL data using a homogeneous transformation
        where :math:`\bf{R}` is the rotation from camera to output frame and :math:`\bf{t}` the dedicated
        translation vector

        Args:
            R(np.ndarray): rotation matrix.    (3 x 3)
            t(np.ndarray): translation vector. (3 x 1)
        Returns:
            np.ndarray: rotated point cloud array (D x 4)
        Warn:
            To spare computation load, the matrix is not checked against the constraints
        """
        X = self.data
        X[:, :3] = np.einsum('ji, ...i', R, X[:, :3])
        X[:, :3] += t
        return X

    def set_static_transform(self, q, p):
        """set static transformation externally

        Args:
            q(np.quaternion or Quaternion or None, optional): rotation as a quaternion. . Defaults to None
            p(np.ndarray or None, optional): translation vector. (3 x 1). Defaults to None
        """
        if isinstance(q, Quaternion):
            q = np.quaternion(q.w, q.x, q.y, q.z)
        self._R_static = quaternion.as_rotation_matrix(q)
        self._p_static = p
        self._calibrated_static_data = self.transformed_data(self._R_static, p)

    def get_convex_hull(self, R=None, t=None):
        """Get convex hull for the current point cloud data as an overapproximation

        Args:
            R (np.ndarray, optional): rotation matrix. Defaults to None.
            t (np.ndarray, optional): translation vector. Defaults to None.

        Returns:
            ConvexHull: convex hull around given points
        """
        try:
            from scipy.spatial import ConvexHull
            if R is None and t is None:
                return ConvexHull(self.data[:, :3])
            if R is None:
                R = np.eye(3)
            if t is None:
                t = np.zeros(3)
            return ConvexHull(self.transformed_data(R, t))
        except (ImportError, ModuleNotFoundError):
            warnings.warn('failed to import ConvexHull from scipy. Functionality limited')
            return

    def get_calibrated_msg(self, filter_func=None):
        """
        Returns calibrated static scene as a PointCloud
        for all samples that pass filter func.
        If ``filter_func`` is set to ``None``, all samples are returned

        Args:
            filter_func(callable, optional): masking function. Defaults to None

        Returns:
            PointCloud2: filtered / transformed point cloud message
        """
        h = Header()
        h.stamp = rospy.get_rostime()
        h.frame_id = "world"
        if self._R_static is None or self._p_static is None:
            D = self._calibrated_static_data[:, :3]
        else:
            D = self.transformed_data(self._R_static, self._p_static)[:, :3]
        if filter_func is not None:
            D = D[filter_func(D)]
        return pc2.create_cloud_xyz32(h, D)

    def set_static_scene(self, ref_frame, cam_frame):
        """ Generate static scene / content and save it to member attribute

        Expects a reference and a camera coordinate system name that can be used
        for tf2 transformation lookup

        Args:
            ref_frame (str): reference frame, e.g. 'world'
            cam_frame (str): camera frame, e.g. 'camera_depth_optical_frame'
        Returns:
            np.ndarray: static PCL data in reference frame
        """
        if not self.received_data:
            rospy.sleep(1.0)
            if not self.received_data:
                warnings.warn('did not receive any PCL data yet')
            return None
        try:
            import tf2_ros
            tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(tf_buffer)
            try:
                trans_msg = tf_buffer.lookup_transform(ref_frame, cam_frame, rospy.Time(0))
                t = trans_msg.transform
                p = np.r_[t.translation.x, t.translation.y, t.translation.z],
                q = np.quaternion(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr(f'failed to lookup transformation from {cam_frame} to {ref_frame}')
                return
            self._calibrated_static_data = self.transformed_data(quaternion.as_rotation_matrix(q), p)
            return self._calibrated_static_data
        except (ImportError, ModuleNotFoundError):
            warnings.warn('failed to import tf2_ros. Functionality limited')
            return


@dataclass(eq=True, frozen=True)
class Pose:
    p: np.ndarray
    q: np.quaternion
    frame: str

    @classmethod
    def from_msg(cls, msg, frame):
        return cls(p=np.r_[msg.position.x, msg.position.y, msg.position.z],
                   q=np.quaternion(msg.orientation.w, msg.orientation.x,
                                   msg.orientation.y, msg.orientation.z),
                   frame=frame)

    @property
    def R(self):
        return quaternion.as_rotation_matrix(self.q)

    @property
    def SE3(self):
        try:
            import spatialmath as sm
            T = sm.SE3(*self.p)
            T.A[:3, :3] = self.R
            return T
        except (ModuleNotFoundError, ImportError):
            rospy.logerr("spatialmath-python not found. Return nothing")


class PoseArrayData:

    def __init__(self, pose_array_topic_name: str or None = None):
        self._points = []  # type: list[Pose]
        self._subs = None  # type: rospy.Publisher or None
        if pose_array_topic_name:
            self.init_ros(pose_array_topic_name)

    def cb(self, msg: PoseArray) -> None:
        if len(self._points) > len(msg.poses):
            self._points = self._points[:len(msg.poses)]
        for i, pose in enumerate(msg.poses):
            new_pose = Pose.from_msg(pose, msg.header.frame_id)
            try:
                self._points[i] = new_pose
            except IndexError:
                self._points.append(new_pose)

    def init_ros(self, pose_array_topic_name: str) -> None:
        self._subs = rospy.Subscriber(pose_array_topic_name, PoseArray, self.cb, queue_size=100)

    @property
    def positions(self) -> np.ndarray:
        return np.c_[[x.p for x in self._points]]

    @property
    def quaternions(self) -> np.ndarray:
        return np.c_[[x.q for x in self._points]]

    @property
    def R_batch(self) -> np.ndarray:
        return np.c_[[x.R for x in self._points]]