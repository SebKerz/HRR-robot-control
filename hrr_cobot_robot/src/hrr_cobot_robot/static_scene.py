"""
This class is the simplest and sort of most stupidiest database replacement
we came up with in order to get the cobidiot running
"""
import re
import pathlib
import spatialmath as sm

import numpy as np


__all__ = ["PoseDataBase"]


class PoseDataBase:

    def __init__(self, ref_frame="base_link") -> None:
        self._frame_id = ref_frame
        self._tool_wrack_center = np.zeros((2, 3))
        self._tool_ee_poses = np.empty((0, 4, 4))
        self._collision_objects = dict()        

    @property
    def nb_tools(self) -> int:
        """helper to get number of tools"""
        return self._tool_ee_poses.shape[0]

    @property
    def nb_collision_objects(self) -> int:
        """helper to get number of tools"""
        return len(self._collision_objects)

    @staticmethod
    def load_files_helper(path, file_pattern):
        r"""Load numpy data (recordings) from file,
        e.g.

        .. code-block:: python
            
            static_scene = PoseDataBase()
            static_scene.load_tool_poses('./rack_positions/', r".*pickup_A.*")


        Args:
            path (str or pathlib.Path): directory to load data from
            file_pattern (str):  regex-conform filtering string used in ``re.compile``        
        Returns:
            np.ndarray: files that match the pattern in directory ``path`` concatenated to single array
        """
        path = pathlib.Path(path)        
        np_data = []
        file_pattern = re.compile(file_pattern)
        for f in path.glob("**/*.npy"):
            if f.is_file() and bool(file_pattern.match(f.name)):
                np_data.append(np.load(f))
        if np_data:
            return np.array(np_data)

    def load_tool_poses(self, path=None, file_pattern=r".*pickup_A.*"):
        r"""Load tool pose recordings from file,
        e.g.

        .. code-block:: python
            
            static_scene = PoseDataBase()
            static_scene.load_tool_poses('./rack_positions/', r".*pickup_A.*")


        Args:
            path (str or pathlib.Path): directory to load data from
            file_pattern (str, optional):  regex-conform filtering string used in ``re.compile``. Defaults to r".*pickup_A.*"
        """
        if path is None:
            path = (pathlib.Path(__file__).parent.parent.parent) / "data" / "tool_changer"
        path = pathlib.Path(path)
        assert path.exists(), f"unknown path {path}"
        self._tool_ee_poses = self.load_files_helper(path, file_pattern)

    def legal_tool_changer_pose(self, T_B_E, min_distance=1e-2, min_degree_err=0.5) -> bool:
        """Check if current pose is sufficiently close to any of the known
        tool-changer positions.
        By default returns False (no tool-changer positions given)

        Args:
            T_B_E (sm.SE3): current FK / ee-pose of the robot
            min_distance (float, optional): acceptable distance in translation. Defaults to 1e-2.
            min_degree_err (float, optional): acceptable rotation deviation in degree. Defaults to 0.5.

        Returns:
            bool: True if pose is sufficiently close to saved any known tool-changer poses
        """
        T_err = np.einsum('ik, nkj -> nij', T_B_E.inv().A, self._tool_ee_poses)
        distance = np.linalg.norm(T_err[:, :3, 3], axis=1)
        T_err = T_err[np.where(distance < min_distance)]
        for T in T_err:
            if np.linalg.norm(sm.SO3(T[:3, :3]).rpy("deg")) < min_degree_err:
                return True
        return False
        
    def legal_tool_changer_q(self, cobot, q, **kwargs)-> bool:
        """more or less the identical functionality of :py:meth:`~legal_tool_changer_pose`
        but receives joints and ``cobot`` instance.
        
        Args:
            cobot(hrr_cobot_robot.hrr_cobot_handle.HrrCobotIf): current cobot handle
            q(np.ndarray): joint configuration
            **kwargs(dict): see :py:meth:`~legal_tool_changer_pose`

        Returns:
            bool: True if joint-pose is sufficiently close to saved any known tool-changer poses
        """
        from hrr_cobot_robot.hrr_cobot_handle import HrrCobotIf
        if isinstance(cobot, HrrCobotIf):
            return self.legal_tool_changer_pose(cobot.FK(q), **kwargs)
        return False

    @staticmethod
    def get_generic_waypoints(T_B_E_goal, B_normal, hover_dist):
        T_g = sm.SE3(T_B_E_goal)
        return [
            sm.SE3(*(hover_dist * B_normal)) @ T_g,
            T_g,
            sm.SE3(*(hover_dist * B_normal)) @ T_g]
        
