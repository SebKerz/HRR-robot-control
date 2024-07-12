#!/usr/bin/env python3
"""
Skill Base Class
---------------------

"""
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, List

import actionlib
import numpy as np
import rospy
from hr_recycler_msgs.msg import CobotState, SkillResult

import hrr_common
from hrr_cobot_robot.hrr_cobot_control import HrrCobotControl
from hrr_cobot_robot.hrr_cobot_observer import HrrCobotObserver

__all__ = ["SkillBase", "SimpleMp", "get_SE3_from_pose_stamped"]


def get_SE3_from_pose_stamped(msg):
    p = hrr_common.vec32np(msg.pose.position)
    q = hrr_common.quat2np(msg.pose.orientation)
    return hrr_common.posquat2homog(p, q)

def dummy_func():
    return False


@dataclass(frozen=True)
class SimpleMp:
    """Simplified Manipulation Primitive (MP) that allows to process individual sub-actions to the robot

    .. note::

        the class is frozen, i.e. the arguments cannot be changed after instantiation.

    Attributes:
        name(str): name of manipulation-primitive
        f_init(callable): function that is called to initialize MP, i.e. set control-commands
        args(tuple): arguments to be parsed to `f_init`
        kwargs(dict): keyword-based arguments to parsed to `f_init`
        contact_as_success(bool): evaluates current MP as successful if environment-contact is detected. Defautls to False
        wrench_bias_hack(bool): uses wrench-bias-hack, i.e. it sets the current force-offset as artificial sensor-bias to diminish calibration issues. Defaults to True.
        T_max(float): timeout for current MP. Defaults to infinity.
    """
    name: str
    f_init: callable
    args: tuple = field(default_factory=tuple)
    kwargs: dict = field(default_factory=dict)
    contact_as_success: bool = False
    wrench_bias_hack: bool = True
    success_func: callable = dummy_func
    T_max: float = np.inf


class SkillBase(hrr_common.RosBaseHandle, ABC):
    _cobot_pseudo_pointer = [None]  # type: List[HrrCobotControl or None]

    def __init__(self, name="", cobot: Optional[HrrCobotControl] = None,
                 observer: Optional[HrrCobotObserver] = None):
        self.cobot = cobot
        self.observer = observer
        self.timeout = np.inf
        self._t0 = rospy.get_time()

        self._skill_result = SkillResult()
        self._name = name
        self._feedback = None
        self._as = None  # type: Optional[actionlib.SimpleActionServer]
        self._tf = hrr_common.TfHandler(add=False)
        super(SkillBase, self).__init__()

    def pre_skill_execution(self, tool_id=None, hardcoded_transformation=True) -> bool:
        """
        Helper function to be called for the start of a skill. Resets timing and resets skill-result.
        """
        self._t0 = rospy.get_time()
        self._skill_result = SkillResult()
        self._skill_result.result = SkillResult.UNKNOWN
        self.cobot.update_tf()
        self.cobot.reset_sensor()
        #if not self.cobot.legal_joint_config(self.cobot.q):
        #    return self.cancel(msg="current robot configuration is invalid. Please maneuver to legal position")
        if tool_id is not None:
            self.assert_tool(tool_id)
        if not hardcoded_transformation:
            if np.all(self.cobot.T_E_C_robot.t == np.zeros(3)):
                rospy.logerr(
                    f"current EE->TCP transformation is incorrect. Wait for 2 seconds and retry model-configuration...")
                rospy.sleep(2.0)
            if np.all(self.cobot.T_E_C_robot.t == np.zeros(3)):
                self.cancel(msg="EE->TCP transformation is incorrect. Cancel execution")
                return False
            self.cobot.update_ee_tcp_tf()
        rospy.loginfo(f'Received new goal for {self._name}-skill.')
        return True

    def _update_skill_result(self):
        self._skill_result.runtime = rospy.get_time() - self._t0
        self._skill_result.cobot_state = self.cobot.cobot_state_msg()

    def assert_tool(self, tool_id, dt=2.0, robot_urdf_prefix="hrr_cobot."):
        if self.cobot.tool_id != tool_id:
            rospy.logwarn(f"change tool to screwdriver assuming URDF-prefix `{robot_urdf_prefix}`")
            self.cobot.change_tool(tool_id, robot_urdf_prefix=robot_urdf_prefix)
            rospy.sleep(dt)
            self.cobot.update_tf()

    @property
    def _cobot_status(self):
        return self.cobot.state

    @property
    def cobot(self) -> HrrCobotControl:
        try:
            return self._cobot_pseudo_pointer[0]
        except IndexError:
            rospy.logerr("cobot handle is not initialized")

    @cobot.setter
    def cobot(self, value):
        if value is None:
            return
        if not isinstance(value, HrrCobotControl):
            rospy.logerr(f"cannot assign skill-cobot handle with {type(value)}")
            return
        if self.cobot is None:
            self._cobot_pseudo_pointer[0] = value

    def _get_cobot(self, cobot: Optional[HrrCobotControl], msg="cobot handle is not initialized") -> HrrCobotControl:
        """getter helper to switch between external and internal cobot handle"""
        if cobot is None:
            assert self.cobot is not None, msg
            return self.cobot
        return cobot

    def reset(self):
        try:
            self.cobot.reset()
        except AttributeError:
            rospy.logerr("tried to reset non-existent cobot-handle")
        self.cobot.state = None
        try:
            self.observer.reset()
        except AttributeError:
            pass
        self.pre_skill_execution()

    def init_skill_base(self, cobot_prefix=None, cobot=None):
        """Initialize Skill-base, i.e.

        * set current skill name as ``_name``
        * set cobot handle from ``cobot`` argument
        * initialize :py:class:`hrr_cobot_robot.hrr_cobot_observer.HrrCobotObserver`-instance to track output.

        Args:
            cobot_prefix(str or None, optional):name-space prefix to instantiate new :py:class:`~hrr_cobot_robot.hrr_cobot_control.HrrCobotControl.` instance.
            cobot(HrrCobotControl or None, optional): cobot handle to be used. Defaults to None
        """
        self.cobot = cobot
        if self.cobot is None:
            rospy.loginfo("did not receive cobot handle. Regenerate new instance")
            self.cobot = HrrCobotControl.from_ros(cobot_prefix=cobot_prefix)
        assert self.cobot.FK(cobot.q) is not None, "needs FK for Cartesian servoing"
        self.observer = HrrCobotObserver(hrr_common.get_param(f"~buffer_size", int(60.0 * 5.0 * cobot.hz)),
                                         *hrr_common.get_param(f"~observe_attributes", ["q", "B_F_msr", "B_err_F"]))

    def drop(self, *args, cobot=None) -> dict:
        """
        Drop current data recordings via observer handle and return data as a dictionary that can be saved
        to data by e.g. pickle or similar.

        Args:
            *args: optional arguments to be saved via :py:meth:`~hrr_cobot_robot.hrr_cobot_observer.HrrCobotObserver.save`
            cobot(HrrCobotControl or None, optional): cobot instance. If None, self.cobot is used
        Returns:
            dict:
        """
        if cobot is None:
            cobot = self.cobot
        try:
            return self.observer.drop(cobot, *args)
        except AttributeError:
            rospy.logerr("cannot get data from observer - there is none...")
            return dict()

    def save_data(self, filename=None, *args):
        """
        Save data collected during current skill. Is called by
        default, if either :py:meth:`~cancel` or :py:meth:`~end_skill`
        are called.

        Additional arguments are forwarded to
        :py:meth`~hrr_cobot_robot.hrr_cobot_observer.HrrCobotObserver.save`
        via :py:meth:`~save`.

        Args:
            filename(Path, optional): file path. Defaults to None.
        Returns:
            Path or None: filename if any data is to be saved.
        """
        if filename is None:
            path = Path.home() / "Documents" / "robot_recordings"
            rospy.logdebug(
                f"no path to file provided. Using default path {path}")
            if not path.exists():
                path.mkdir(parents=True)
                rospy.loginfo(f"created path: {path}")
            filename = path / f"{self._name}_data.npy"
        else:
            filename = Path(filename)
        try:
            file_base_name = filename.stem
            for i in range(1000):
                if not filename.exists():
                    break
                filename = filename.parent / f"{file_base_name}_{i:03d}.npy"
            if not filename.exists():
                rospy.loginfo(f"saving data at {filename}")
                np.save(str(filename), self.drop(*args))
                return filename
        except Exception as e:
            rospy.logerr(
                f"encountered error ->{e}<- while saving collected data")

    def cancel(self, *args, save_data=True, msg=None):
        """put robot to halt and drop current data for debugging

        Args:
            *args: optional arguments to be forwarded to :py:meth:`~save`
            save_data(bool, optional): flag to save data at canceling.
            msg(str, optional): optional error message.
        Returns:
            pathlib.Path or None: output from :py:meth:`~save_data`.
        """
        self._skill_result.result = SkillResult.FAILED
        try:
            self.cobot.emergency_stop()
        except AttributeError:
            rospy.logerr("could not stop cobot -> no handle initialized.")
        msg = f"Error encountered! Cancel {self._name}" if msg is None else msg
        self.empty_error_message(msg)
        if save_data:
            self.save_data(*args)

    def as_active(self) -> bool:
        if self._as is not None:
            return self._as.is_active()
        return False

    def end_skill(self, *args, msg=None, save_data=True):
        """default end skill
        The procedure is given as

        #. stopping current motion via :py:meth:`~hrr_cobot_robot.HrrCobotId.stop`
        #. logging current result message or default succession message
        #. storing results in a file (optional) if :py:meth:`~save_data` returns a filename
        #. if current action-server has an active goal, set this goal to succeeded
        #. (optionally) add the filename of the collected data to the result that is published as the action-server result

        Args:
            *args: optional arguments to be stored in file
            msg(str or None): success-message to be published and printed via ros-out
            save_data(bool, optional): flag to enable data saving.
        """
        #self.cobot.F_max = 80
        self._skill_result.result = SkillResult.FINISHED
        try:
            self.cobot.stop()
        except AttributeError:
            rospy.logerr("could not stop robot -> no handle initialized")
        msg = f"succeeded with skill {self._name}" if msg is None else msg
        rospy.loginfo(msg)
        filename = None
        if save_data:
            filename = self.save_data(*args)
        if self.as_active():
            res = self.result
            if filename is not None and hasattr(res, 'data_file_path'):
                res.data_file_path = str(filename)
            self._as.set_succeeded(res, msg)

    def empty_error_message(self, msg):
        """
        Terminate / Abort-helper that can be used for action-services, e.g. as

        .. code-block::
            python


            msg = ""
            if np.linalg.norm(self._cobot.B_F_msr) > 10.0:
                msg = "cannot start when in |F| > 10.0"
            if self.empty_error_message(msg):
                # parse goal ...
                if goal.a < 0.0:
                    msg = "a must be negative"
                if self.empty_error_message(msg):
                    # execute action ...

        Args:
            msg (str or None): error string for abort

        Returns:
            bool: True if action-service has not been terminated
        """
        if isinstance(msg, str) and len(msg) > 1:
            rospy.logerr(msg)
            if self.as_active():
                if self._skill_result.result != self._skill_result.TIMEOUT:
                    self._skill_result.result = SkillResult.FAILED
                self._as.set_aborted(self.result, msg)
            return False
        return True

    @property
    def action_server_valid(self) -> bool:
        """Function that can be called within an execution loop to check against preemption or timeouts.

        .. note::

            by default there is no timeout as ``self.timeout`` is infinite.

        In case the action-server is preempted or a timeout is encountered, the action is
        directly handled, thus when calling this within an execution loop, it is sufficient to run

        .. code-block:: python

            if not self.action_server_valid:
                return

        to close the action-server execution

        Returns:
            bool: True if current action-server is valid
        """
        if self._as is None:
            return False
        if self._as.is_preempt_requested():
            rospy.logwarn("Preempt Request received.")
            if self.as_active():
                self._skill_result.result = SkillResult.FAILED
                self._as.set_preempted(self.result, f"skill {self._name} Preempted. Should Stop!")
            self.cobot.stop()
            return False
        if (rospy.get_time() - self._t0) > self.timeout:
            self._skill_result.result = SkillResult.TIMEOUT
            self.empty_error_message(msg=f"skill {self._name} exceeded timeout of {self.timeout:.2f} [s]")
            self.cobot.stop()
            return False
        return True

    @abstractmethod
    def execute_skill_cb(self, goal):
        """
        General function to process action-service

        Args:
            goal ([type]): [description]
        """

    @property
    def done(self) -> bool:
        """return True if current skill-status is set to FINISHED"""
        return self._cobot_status == CobotState.FINISHED

    @property
    def error(self):
        """return True if current skill-status is set to FAILED"""
        return self._cobot_status == CobotState.FAILED

    @property
    def failure(self):
        """return True if current skill-status is set to FAILED"""
        return self._skill_result.result in (SkillResult.FAILED, SkillResult.TIMEOUT)

    def set_done(self, *args, msg=None, save_data=True) -> None:
        """helper setter to assure that whenever

         .. code-block::
            python

            self.set_done()

        is set, the robot is stopped and the action-service is set to a successful state.

        Similar to the setter of :py:meth:`~error`

        Args:
            *args: optional arguments for :py:meth:`~save`
            msg(str, optional): logging / result message
            save_data(bool, optional): if True, the data is of the observer is saved to file.
        """
        if not self._cobot_status == CobotState.FINISHED:
            self._skill_result.result = CobotState.FINISHED
            self.end_skill(*args, msg=msg, save_data=save_data)

    def raise_error(self, *args, msg=None, save_data=True):
        """helper setter assuring that whenever

        .. code-block::
            python

            self.raise_error()

        is set, the current implementation triggers the emergency break and cancels the action-service thread

        Similar to the setter of :py:meth:`~done`

        Args:
            *args: optional arguments for :py:meth:`~save`
            msg(str, optional): logging / result message
            save_data(bool, optional): if True, the data is of the observer is saved to file.
        """
        if not self._cobot_status == CobotState.FAILED:
            self._skill_result.result = SkillResult.FAILED
            if msg:
                rospy.logerr(msg)
            self.cancel(*args, msg=msg, save_data=save_data)

    @property
    def result(self):
        """Implement this in the actual skill to return the outcome"""
        return None

    def check_pose(self, T_B_C_des):
        """
        Check pose to be reachable without randomness

        Args:
            T_B_C_des(sm.SE3 or np.ndarray):

        Returns:
            bool: True if pose is legal, i.e. reachable for the robot without violating the joint constraints
        """
        try:
            if os.getenv("HRR_QUICK_HACK") == '1':
                return True
        except KeyError:
            pass
        return self.cobot.is_tcp_reachable(T_B_C_des)

    def find_best_IK(IKs):
        """
        Finds best IK in an array of given IK
        Args:
            IKs: np.array of given IK
        Returns (np.array):  best IK

        """

        norm = []
        for i in range(len(IKs)):
            norm.append(np.linalg.norm(IKs[i]))

        bestIK = IKs[np.argmin(norm)]
        return bestIK



