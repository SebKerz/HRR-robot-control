import numpy as np
import rosparam
import rospy

try:
    from comau_msgs.srv import SetIO, GetComauParams, GetComauParamsRequest, SetIORequest
except ImportError:
    import warnings

    warnings.warn("current version of comau_msgs seems outdated")


    def fake(*_, **__):
        return None


    SetIO = GetComauParams = GetComauParamsRequest = SetIORequest = fake

from hrr_cobot_robot.tool_control._tool_controller_base import ToolControllerBase

__all__ = ["IntegrationHrrCobotToolControlInterface"]

from dataclasses import dataclass
from typing import List


@dataclass(frozen=True)
class ScrewdriverPins:
    prog_pins: np.ndarray = np.array([13, 12, 11], dtype=int)
    prog_E_pin: int = 14

    def sanity_check(self):
        pins = self.prog_pins.tolist() + self.button_pins.tolist() + [self.button_E_pin, self.prog_E_pin]
        assert len(set(pins)) == len(pins)

    def set_program(self, num) -> dict:
        assert 0 <= num <= 8, "program number limited to (0,8)"
        pins = {self.prog_E_pin: True if num == 0 else False}
        for pin, value in zip(self.prog_pins, f"{num - 1:03b}"):
            pins[pin] = True if value != '0' else False
        return pins

    @staticmethod
    def dict_to_req(pin_dict) -> List[SetIORequest]:
        return [SetIORequest(pin=pin, state=value) for pin, value in pin_dict.items()]

    def program_to_ros_msg(self, num) -> List[SetIORequest]:
        return self.dict_to_req(self.set_program(num))

    @staticmethod
    def start_combination(self):
        return {10: False, 9: True, 8: False, 7: True}

    @staticmethod
    def stop_combination(self):
        return {10: False, 9: True, 8: False, 7: False}

    def start_ros_msg(self):
        return self.dict_to_req(self.start_combination())

    def stop_ros_msg(self):
        return self.dict_to_req(self.stop_combination())


class IntegrationHrrCobotToolControlInterface(ToolControllerBase):
    """This class instance is expected to communicate with a
    COMAU robot that is allowing to set the
    """
    _VACUUM_VALVE_PINS = (1, 3)

    def __init__(self):
        super().__init__()
        self._tool_change_pin = 5
        self._screwdriver_pins = ScrewdriverPins()
        self._dout_map = {x: False for x in self._VACUUM_VALVE_PINS}

    def _get_pin_srv(self) -> callable:
        try:
            return self._srvs["set_pin"]
        except KeyError:
            raise RuntimeError("please run ``init_ros`` -> current pin service is empty")

    def _set_pin(self, pin, value):
        assert pin in self._dout_map.keys(), f"cannot set unknown pin {pin}"
        self._get_pin_srv()(SetIORequest(pin=pin, state=value))

    def pin_str(self):
        try:
            from pprint import pformat
            return pformat(self._dout_map)
        except ModuleNotFoundError:
            return f"{self._dout_map}"

    def _update_vacuum_valve(self):
        if self._vacuum_on:
            self._set_pin(self._VACUUM_VALVE_PINS[0], False)
            self._set_pin(self._VACUUM_VALVE_PINS[1], True)
        else:
            self._set_pin(self._VACUUM_VALVE_PINS[0], True)
            self._set_pin(self._VACUUM_VALVE_PINS[1], False)

    def _send_rpm(self):
        pass

    def _send_sc_prog(self):
        pin_srv = self._get_pin_srv()
        list(map(pin_srv, self._screwdriver_pins.program_to_ros_msg(self.screwdriver_program)))

    def screwdriver_start(self):
        """Start screwdriver rotation"""
        pin_srv = self._get_pin_srv()
        list(map(pin_srv, self._screwdriver_pins.start_ros_msg()))

    def screwdriver_stop(self):
        pin_srv = self._get_pin_srv()
        list(map(pin_srv, self._screwdriver_pins.stop_ros_msg()))

    def init_ros(self, set_io_service_name, get_cobot_state_srvice_name, cobot_prefix, **__):
        """
        Initialize ROS-API for the Arduino Controller.
        No parameters are collected automatically.

        Args:
            set_io_service_name(str): name of digital OUT setter service
            get_cobot_state_srvice_name
            prefix (str, optional): [description]. Defaults to "~".
        """
        self._srvs["get_state"] = rospy.ServiceProxy(f"{get_cobot_state_srvice_name}", GetComauParams)
        self._srvs["set_pin"] = rospy.ServiceProxy(f"{set_io_service_name}", SetIO)
        self._tool_change_pin = rospy.get_param(f"{cobot_prefix}tool_changer_pin", self._tool_change_pin)
        if self._tool_change_pin > 0:
            self._dout_map[self._tool_change_pin] = False

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
        out.init_ros(get_cobot_state_srvice_name=rospy.get_param(f"{cobot_prefix}get_comau_parameters_service_name"),
                     set_io_service_name=rospy.get_param(f"{cobot_prefix}set_comau_dout_pin_service_name"), )
        return out

    def update(self):
        try:
            state_res = self._srvs["get_state"](GetComauParamsRequest())
            for dout in state_res.digital_out_states:
                self._dout_map[dout.pin] = dout.state
        except KeyError:
            rospy.logerr("could not get latest ")

    @property
    def tool_changer_open(self):
        self.update()
        return self._dout_map[self._tool_change_pin]

    def open_tool_changer(self):
        if self._tool_change_pin > 0:
            self._set_pin(self._tool_change_pin, True)

    def close_tool_changer(self):
        if self._tool_change_pin > 0:
            self._set_pin(self._tool_change_pin, False)

    def reset(self):
        pass


if __name__ == "__main__":
    rospy.init_node('fuuuu')
    rosparam.set_param("/hrr_cobot/get_comau_parameters_service_name", "/hrr_cobot/get_comau_parameters")
    rosparam.set_param("/hrr_cobot/set_comau_dout_pin_service_name", "/hrr_cobot/set_digital_io")
    test = HrrCobotToolControlInterface.from_ros("/hrr_cobot")
    test.tool = "vacuum"
    rospy.loginfo("enable vacuum for 1 second")
    test.vacuum = True
    rospy.sleep(1.0)
    test.vacuum = False
    test.update()
    rospy.loginfo(test.pin_str())
