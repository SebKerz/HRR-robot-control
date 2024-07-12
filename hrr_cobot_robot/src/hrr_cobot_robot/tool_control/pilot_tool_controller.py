#!/usr/bin/env python3
"""
Hrr Cobot EE-Tools Control Interface
----------------------------------------

Wraps Cobot tool-controllers into single class instance, namely:

* ``_arduino_fallback`` as :py:class:`~hrr_cobot_robot.tool_control.arduino_tool_controller.ArduinoToolControlInterface`

ROS-parameters
^^^^^^^^^^^^^^^^^^^

All ROS-parameters are prepended by a ``{cobot_prefix}``, usually ``/hrr_cobot/``

========================= ================== ========================================================
Parameter                 default value      notes
========================= ================== ========================================================
use_arduino_interface     False              flag to instantiate ``_arduino_fallback`` (c.f. above)
========================= ================== ========================================================
"""
from typing import Optional, Union, Dict

import rospy
from hr_recycler_msgs.msg import ToolType

from hrr_common.ros_utils.helper_handles import get_param
from hrr_controllers.kolver_controller import ScrewdriverControl
from hrr_controllers.shaft_grinder_controller import ShaftgrinderControl
from hrr_controllers.tool_changer_controller import GimaticPinController, GimaticEncoderControl
from hrr_controllers.vacuum_valve_controller import VacuumEncoderControl, VacuumPinController


from hrr_cobot_robot.tool_control._tool_controller_base import ToolControllerBase
from hrr_cobot_robot.tool_control.arduino_tool_controller import ArduinoToolControlInterface


class HrrCobotToolControlInterface(ToolControllerBase):

    def __init__(self):
        super(HrrCobotToolControlInterface, self).__init__()
        self._sc = None  # type: Optional[ScrewdriverControl]
        self._sg = None  # type: Optional[ShaftgrinderControl]
        self._tc = None  # type: Union[GimaticPinController, GimaticPinController, None]
        self._vc = None  # type: Union[VacuumEncoderControl, VacuumPinController, None]
        self._arduino_fallback = None  # type: Optional[ArduinoToolControlInterface]
        self._reset_time = 0.4
        self._default_runtime = 2.0
        self._prev_tool = -1

    def init_ros(self, *_, **__):
        pass

    @classmethod
    def _from_ros(cls, cobot_prefix, **kwargs):
        out = cls()
        out._sc = ScrewdriverControl.from_ros(cobot_prefix=cobot_prefix)
        out._sg = ShaftgrinderControl.from_ros(cobot_prefix=cobot_prefix)
        g_enc = get_param(f"{cobot_prefix}use_tool_changer_encoder", True)
        v_enc = get_param(f"{cobot_prefix}use_vacuum_control_encoder", False)
        out._tc = GimaticEncoderControl.from_ros(cobot_prefix=cobot_prefix) if g_enc else \
            GimaticPinController.from_ros(cobot_prefix=cobot_prefix)
        out._vc = VacuumEncoderControl.from_ros(cobot_prefix=cobot_prefix) if v_enc else \
            VacuumPinController.from_ros(cobot_prefix=cobot_prefix)
        out._reset_time = get_param(fr"{cobot_prefix}controller_reset_sleep_time", out._reset_time)
        out._default_runtime = get_param(fr"{cobot_prefix}screwdriver_default_runtime", out._default_runtime)
        try:
            rospy.get_param(f"{cobot_prefix}arduino_node_name")
            out._arduino_fallback = ArduinoToolControlInterface.from_ros(cobot_prefix=cobot_prefix)
        except KeyError:
            pass
        return out

    def reset(self):
        if self._cur_tool == ToolType.SCREW_DRIVER:
            self._sc.deactivate()
            rospy.sleep(self._reset_time)
            self._sc.activate()
        elif self._cur_tool == ToolType.SHAFT_GRINDER:
            self._sg.deactivate()
            rospy.sleep(self._reset_time)
            self._sg.activate()
        elif self._cur_tool == ToolType.VACUUM_GRIPPER:
            self._vc.deactivate()
            rospy.sleep(self._reset_time)
            self._vc.activate()
        pass

    def _update_vacuum_valve(self):
        if self._cur_tool == ToolType.VACUUM_GRIPPER:
            try:
                self._vc.on = self._vacuum_on
            except AttributeError:
                rospy.logerr("Vacuum Encoder controller not available")

    def _send_rpm(self):
        if self._rpm <= self._min_rpm:
            return
        try:
            # small hack for current integration
            if self._rpm >= self._max_rpm:
                self.run_shaftgrinder(self._sg.timeout, self._sg.fastest )
            else:
                rospy.logerr_once("do not use ``rpm`` with this interface. command the desired mode directly")
                self.run_shaftgrinder(self._sg.timeout, self._sg.slowest)
        except NotImplementedError:
            self._arduino_fallback.rpm = self.rpm

    def _send_sc_prog(self):
        self._sc.program_handle.mode_id = self.screwdriver_program

    def open_tool_changer(self, sleep_time=0.5):
        self._tc.activate()
        rospy.sleep(sleep_time)
        self._tc.mode_name = "open"
        if self._cur_tool in (ToolType.SCREW_DRIVER, ToolType.SHAFT_GRINDER,
                              ToolType.WSG_50_DSA, ToolType.WSG_50,
                              ToolType.VACUUM_GRIPPER):
            self.tool = "nothing"

    def close_tool_changer(self, sleep_time=0.5):
        self._tc.activate()
        rospy.sleep(sleep_time)
        self._tc.mode_name = "close"
        rospy.sleep(sleep_time)
        self._tc.deactivate()

    @property
    def tool_changer_open(self):
        return self._tc.open

    def run_screwdriver(self, dt, prog=None, sleep_time=0.1):
        if self._cur_tool == ToolType.SCREW_DRIVER:
            self._sc.run_program(self.screwdriver_program if prog is None else prog, dt, sleep_time=sleep_time)

    def screwdriver_start(self):
        if self._cur_tool == ToolType.SCREW_DRIVER:
            self._sc.run_program(self.screwdriver_program, self._default_runtime, self._reset_time)

    def screwdriver_stop(self):
        self._sc.program_handle.mode_id = -1

    def send_mode(self):
        if self._cur_tool != self._prev_tool:
            if self._prev_tool == ToolType.SHAFT_GRINDER:
                self._sg.deactivate()
                if self._arduino_fallback:
                    self._arduino_fallback.tool = ToolType.NONE
            elif self._prev_tool == ToolType.SCREW_DRIVER:
                self._sc.deactivate()
            elif self._prev_tool == ToolType.VACUUM_GRIPPER:
                self._vc.deactivate()
            if self._cur_tool == ToolType.SHAFT_GRINDER:
                self._sg.activate()
                if self._arduino_fallback:
                    self._arduino_fallback.tool = ToolType.SHAFT_GRINDER
            elif self._cur_tool == ToolType.SCREW_DRIVER:
                self._sc.activate()
            elif self._cur_tool == ToolType.VACUUM_GRIPPER:
                self._vc.activate()
        self._prev_tool = self._cur_tool

    def run_shaftgrinder(self, dt, mode):
        """Run Shaft-grinder for a predefined runtime.
        
        The mode can be either a legal program id, i.e. program output number of encoder
        or the name as provided from the ROS-parameter configuration.
        
        If both are provided and both are legal, the name is used and the ID is skipped.
        
        .. warning::
            
            the shaft-grinder is only slowly disabled, such that 1-2 seconds need to be added to ``dt``

        Args:
            dt (float): time for shaft-grinder to run
            mode (int or str): shaft-grinder speed-ID or name
        """
        if self._cur_tool == ToolType.SHAFT_GRINDER:
            self._sg.timeout = dt
            if isinstance(mode, str):
                self._sg.mode_name = mode
            elif isinstance(mode, int):
                self._sg.mode_id = mode
            else:
                rospy.logerror(f"trying to set shaft-grinder to mode {mode}, which is of illegal type {type(mode)}")

    @property
    def shaft_grinder_modes(self):
        try:
            return set(self._sg.legal_names.keys())
        except AttributeError:
            return set()

