"""
Tool controller interface
===========================

This interface opens easy to use ROS-API to set / command the
"custom" tools of the HR-Recycler project.

"""
from abc import ABC, abstractmethod
from typing import Dict, List, Union

import rospy

from hr_recycler_msgs.msg import ToolType

from hrr_common.ros_utils.templates import RosBaseHandle

from hrr_cobot_robot.utils import tool_type2str, tool2tool_type

__all__ = ["ToolControllerBase"]


class ToolControllerBase(RosBaseHandle, ABC):
    """
    This class tries to unify the commands / integration steps between TUM-arduino hacks and robot control
    Thus,

    .. code-block:: python

        A.tool = "shaftgrinder"
        A.enable_shaftgrinder()

        A.tool = "screwdriver"
        A.enable_screwdriver()

    Once the tool is set, the individual commands can be set, e.g.

    .. code-block:: python

        # shaft grinder
        A.rpm = 5000

        # screwdriver
        A.screwdriver_program = 1
        A.screwdriver_start()

    For the vacuum gripper, the process can be shortened as enabling the control mode is
    equal to enabling the pneumatic valve. Thus,

    .. code-block:: python

        A.vacuum = 1
        A.vacuum = True

    enables the pneumatic valve, whereas ```A.vacuum = False``` disables it again.
    """

    def __init__(self):
        self._cur_tool = ToolType.NONE
        self._driver_status = -10
        self._srvs = dict()  # type: Dict[str, rospy.ServiceProxy]
        self._pubs = dict()  # type: Dict[str, rospy.Publisher]
        self._subs = []  # type: List[rospy.Subscriber]
        self._rpm = 0
        self._screwdriver_prog = 0
        self._screwdriver_motor = 0
        self._screwdriver_button = 0
        self._vacuum_on = False
        self._max_rpm = 25000
        self._min_rpm = 3500
        self._max_prog_num = 8
        self._min_prog_num = 1

    @property
    def tool(self) -> str:
        """Tool mode in human readable form, according to :py:meth:`~tool_type2str`

        As the setter calls the :py:meth:`tool2tool_type` function, you may set the tool
        from string

        .. code-block:: python

            A = ArduinoControlInterface.from_ros()
            A.tool = "shaftgrinder"

        or from integers

        .. code-block:: python

            from hrr_msgs.msg import ToolType

            A = ArduinoControlInterface.from_ros()
            A.tool = ToolType.SHAFT_GRINDER

        Returns:
            str: current tool, e.g. "screwdriver", "shaftgrinder", etc.
        """
        return tool_type2str(self._cur_tool)

    @tool.setter
    def tool(self, value) -> None:
        self._cur_tool = tool2tool_type(value)
        self.send_mode()

    @abstractmethod
    def reset(self):
        """Call reset service if dedicated ROS-service is available"""

    def send_mode(self):
        pass

    @property
    def vacuum(self) -> bool:
        """Method to turn the vacuum on the vacuum gripper via the arduino on.
        When set, the current tool is set to vacuum gripper and the command is forwarded
        to the Arduino controller if HIGH / True.

        Args:
            value (bool): flag that represents state of digital I/O on Arduino board
        """
        if self._cur_tool in (ToolType.VACUUM_GRIPPER, ToolType.WSG_50):
            return self._vacuum_on
        return False

    @vacuum.setter
    def vacuum(self, value) -> None:
        if self._cur_tool in (ToolType.VACUUM_GRIPPER, ToolType.WSG_50):
            self._vacuum_on = bool(value)
            self._update_vacuum_valve()
        else:
            rospy.logerr(f"cannot alter vacuum valve with tool being set to {self.tool}")

    @property
    def screwdriver_program(self) -> Union[int, None]:
        """Screwdriver property to ease ROS-API.
        Using this setter ensures that the value is within min/max ranges,
        sets the internal screwdriver program value, which is returned in the
        getter-function, and then sends the program to the Arduino controller
        via ROS.

        Returns:
            Union[int, None]: current screwdriver program if feasible

        Raises:
            AssertionError: if infeasible program value is provided
        """
        if self._min_prog_num <= self._screwdriver_prog <= self._max_prog_num:
            return self._screwdriver_prog

    @screwdriver_program.setter
    def screwdriver_program(self, value) -> None:
        assert self._min_prog_num <= value <= self._max_prog_num, \
            f"program #{value} is outside of range ({self._min_prog_num}, {self._max_prog_num})"
        self._screwdriver_prog = value
        self._send_sc_prog()

    @property
    def rpm(self):
        """RPM property to ease ROS-API
        Using the setter implemented below, the value is cut to set bounds and then
        send to the Arduino controller via ros directly, i.e.

        .. code-block::
            python

            A = ToolController.from_ros()
            A.tool = "shaftgrinder"
            A.rpm = 5000

        would instantiate this class, set the tool via :py:meth:`~tool2tool_type` function
        and then send the rpm to 5000 and publish it to ROS.
        """
        return self._rpm

    @rpm.setter
    def rpm(self, value):
        if value < self._min_rpm:
            rospy.logwarn(f"minimum value is {self._min_rpm} => set value to 0")
            self._rpm = 0
        elif value > self._max_rpm:
            rospy.logwarn(f"maximum value is {self._max_rpm}. Set value accordingly.")
            self._rpm = self._max_rpm
        else:
            self._rpm = value
        self._send_rpm()

    @abstractmethod
    def _update_vacuum_valve(self):
        """actual call to disable """
        pass

    @abstractmethod
    def _send_rpm(self):
        """send RPM to tool controller"""
        pass

    @abstractmethod
    def _send_sc_prog(self):
        """send RPM to tool controller"""
        pass

    @abstractmethod
    def screwdriver_start(self):
        """Start screwdriver rotation"""

    @abstractmethod
    def screwdriver_stop(self):
        """Stop screwdriver rotation"""
        
    def run_screwdriver(self, dt, prog=None, sleep_time=0.1):
        raise NotImplementedError("this function is not available for current instance")

    def run_shaftgrinder(self, dt, mode):
        raise NotImplementedError

    def open_tool_changer(self):
        raise NotImplementedError("no interface to tool-changer given")

    def close_tool_changer(self):
        raise NotImplementedError("no interface to tool-changer given")

    def stop(self):
        """set current tool into idle mode"""
        if self._cur_tool == ToolType.SHAFT_GRINDER:
            self.rpm = 0
        elif self._cur_tool == ToolType.SHAFT_GRINDER:
            self.screwdriver_stop()
        elif self._cur_tool == ToolType.VACUUM_GRIPPER:
            self.vacuum = False
