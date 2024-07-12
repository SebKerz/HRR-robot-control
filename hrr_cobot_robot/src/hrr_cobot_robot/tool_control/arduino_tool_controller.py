#!/usr/bin/env python3
"""
Arduino robot tools-controller interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Warn:
    this module is deprecated and only maintained for testing / transition

"""
import rospy
from std_msgs.msg import Float32, Int8
from std_srvs.srv import Empty, EmptyRequest
from hr_recycler_msgs.msg import ToolType

from hrr_common.ros_utils.helper_handles import get_param

from hrr_cobot_robot.tool_control._tool_controller_base import ToolControllerBase

__all__ = ["ArduinoToolControlInterface"]


class ArduinoToolControlInterface(ToolControllerBase):
    """This class instance is intended to ease the communication and keep Arduino dependent
    control I/O to a minimum.

    The class can simply be initialized via

    .. code-block:: python

        A = ArduinoControlInterface.from_ros()

    using the available properties, the interface can then be used to indirectly
    communicate with the Arduino-controller.
    """
    def __init__(self):
        super(ArduinoToolControlInterface, self).__init__()

    def send_mode(self):
        """Send the current control mode to the Arduino-controller via ROS.

        Note:
            This method does not allow to set the current tool as a function-argument;
            instead it sends the current value of the ``tool``-property, via :py:meth:`~mode_msg`
            to the Arduino-controller.
        """
        if self._cur_tool == ToolType.VACUUM_GRIPPER:
            return
        try:
            self._pubs["mode"].publish(self.mode_msg)
        except KeyError:
            rospy.logerr(f"cannot update tool controller mod via ROS-publisher. Available publishers: "
                         f"{self._pubs.keys()}")

    def _send_rpm(self):
        if self._cur_tool == ToolType.SHAFT_GRINDER:
            self._pubs["gr_speed"].publish(Float32(self._rpm))
        else:
            rospy.logwarn("tool controller is currently not in shaft grinder mode")

    def _send_sc_prog(self):
        if self._cur_tool == ToolType.SCREW_DRIVER:
            self._pubs["sc_prog"].publish(Int8(self._screwdriver_prog))
        else:
            rospy.logwarn("tool controller is currently not in screwdriver mode")

    def _send_sc_motor_command(self):
        if self._cur_tool == ToolType.SCREW_DRIVER:
            self._pubs["sc_motor"].publish(Int8(self._screwdriver_motor))
        else:
            rospy.logwarn("tool controller is currently not in screwdriver mode")

    def _send_sc_button(self):
        if self._cur_tool == ToolType.SCREW_DRIVER:
            self._pubs["sc_button"].publish(Int8(self._screwdriver_button))
        else:
            rospy.logwarn("tool controller is currently not in screwdriver mode")

    def enable_grinder(self):
        """Method to set the control mode of the Arduino driver to shaft grinder."""
        self.tool = ToolType.SHAFT_GRINDER
        self.send_mode()

    def _update_vacuum_valve(self):
        """Method to set the control mode of the Arduino driver to vacuum gripper.
        Note:
            This only forwards the command to the Arduino, if the vacuum flag is on
        """
        self.tool = ToolType.VACUUM_GRIPPER
        if self._vacuum_on:
            self.send_mode()
        else:
            try:
                self._pubs["mode"].publish(ToolType.NONE)
            except KeyError:
                rospy.logerr(f"cannot disable vacuum gripper due to missing ROS-publisher. Available publishers: "
                             f"{self._pubs.keys()}")

    def enable_screwdriver(self):
        """Method to set the control mode of the Arduino driver to screwdriver."""
        self.tool = ToolType.SCREW_DRIVER
        self.send_mode()

    def disable(self):
        """Method to disable current tool and set-values"""
        self.rpm = 0
        if self._cur_tool == ToolType.SCREW_DRIVER:
            self._screwdriver_prog = 0
            self._send_sc_prog()
        self._vacuum_on = False
        self.tool = None
        self.send_mode()

    def screwdriver_ok(self):
        """Method to trigger the screwdriver OK button"""
        self._screwdriver_button = 1
        self._send_sc_button()

    def screwdriver_reset(self):
        """Method to trigger the screwdriver RST button"""
        self._screwdriver_button = 2
        self._send_sc_button()

    def screwdriver_esc(self):
        """Method to trigger the screwdriver ESC button"""
        self._screwdriver_button = 4
        self._send_sc_button()

    def screwdriver_stop(self):
        """Method to stop the screwdriver"""
        self._screwdriver_motor = 0
        self._send_sc_motor_command()

    def screwdriver_start(self):
        """Method to initialize a screwdriver program"""
        self._screwdriver_motor = 1
        self._send_sc_motor_command()

    def screwdriver_reverse(self):
        """Method to initialize a screwdriver program in reversed direction"""
        self._screwdriver_motor = -1
        self._send_sc_motor_command()

    def update(self):
        """(re-)send the current control mode to the Arduino to prevent an emergency off-switch

        ToDo:
            add update for screwdriver control
        """
        if self._cur_tool in (ToolType.NONE, ToolType.WSG_50, ToolType.WSG_50_DSA):
            return
        if self._cur_tool == ToolType.SHAFT_GRINDER:
            self._send_rpm()
        self.send_mode()

    def init_ros(self, arduino_ros_name, cobot_prefix, **__):
        """
        Initialize ROS-API for the Arduino Controller.
        No parameters are collected automatically.

        Args:
            arduino_ros_name ([type]): [description]
            cobot_prefix (str, optional): [description]. Defaults to "~".
        """
        self._srvs["reset"] = rospy.ServiceProxy(f"{arduino_ros_name}/reset", Empty)
        self._pubs["mode"] = rospy.Publisher(f"{arduino_ros_name}/set_control_mode", Int8, queue_size=100)
        self._pubs["gr_speed"] = rospy.Publisher(f"{arduino_ros_name}/grinder/speed", Float32, queue_size=100)
        self._pubs["sc_motor"] = rospy.Publisher(f"{arduino_ros_name}/screwdriver/motor", Int8, queue_size=100)
        self._pubs["sc_prog"] = rospy.Publisher(f"{arduino_ros_name}/screwdriver/set_program", Int8, queue_size=100)
        self._pubs["sc_button"] = rospy.Publisher(f"{arduino_ros_name}/screwdriver/button", Int8, queue_size=100)
        self._max_rpm = get_param(f"{cobot_prefix}max_rpm", self._max_rpm)
        self._min_rpm = get_param(f"{cobot_prefix}min_rpm", self._min_rpm)

    @classmethod
    def _from_ros(cls, cobot_prefix, **kwargs):
        """
        Method to set all (default) parameters and collect
        ros-params for a fully initialized cobot interface/ROS API.

        Args:
            prefix (str, optional): ros-parameter prefix. Defaults to "~".

        Returns:
            new class instance, with full ROS-API initialized
        """
        out = cls()
        out.init_ros(arduino_ros_name=get_param(f"{cobot_prefix}arduino_node_name"), cobot_prefix=cobot_prefix)
        return out

    def reset(self):
        try:
            self._srvs["reset"](EmptyRequest())
        except KeyError:
            rospy.logerr(f"cannot call reset service from services: {self._srvs.keys()}")

    @property
    def mode_msg(self) -> Int8:
        """
        convert the current tool to the Arduino compatible ROS-message version.

        Returns:
            Int8: current tool-mode as ROS-message compatible for the Arduino controller
        """
        if self._cur_tool in (ToolType.VACUUM_GRIPPER,
                              ToolType.SHAFT_GRINDER,
                              ToolType.SCREW_DRIVER):
            return Int8(self._cur_tool)
        return ToolType.NONE
