#!/usr/bin/env python3
"""
Vacuum Valve Controller
---------------------------------------------------
"""

import rospy
import numpy as np

from hrr_controllers.dout_encoder_controller import EncoderController
from hrr_controllers.dout_controller_handle import DoutController

__all__ = ["VacuumEncoderControl", "VacuumPinController"]


class VacuumEncoderControl(EncoderController):

    def __init__(self):
        super(VacuumEncoderControl, self).__init__(frame_id="vacuum_valve")

    @classmethod
    def _from_ros(cls, cobot_prefix, **__):
        out = cls()
        out.init_ros(controller_name=rospy.get_param(f"{cobot_prefix}vacuum_valve_encoder_controller_name"),
                     cobot_prefix=cobot_prefix, **__)
        return out


class VacuumPinController(DoutController):

    def __init__(self):
        super(VacuumPinController, self).__init__(frame_id="vacuum_valve")

    def read(self):
        for i, digital in enumerate(self.state.pins):
            self._pins[i] = 1 if digital.state else 0

    @property
    def on(self):
        self.read()
        return np.all(self._pins != self._pin_start_values)

    @on.setter
    def on(self, value):
        self.read()
        if value and np.all(self._pins == self._pin_start_values):
            self.invert()
        elif not value and np.all(self._pins != self._pin_start_values):
            self.invert()

    @classmethod
    def _from_ros(cls, cobot_prefix, **__):
        out = cls()
        out.init_ros(controller_name=rospy.get_param(f"{cobot_prefix}vacuum_valve_pin_controller_name"),
                     cobot_prefix=cobot_prefix, **__)
        return out


if __name__ == "__main__":
    rospy.init_node("test_vacuum_controller")
    VP = VacuumPinController.from_ros(cobot_prefix="/hrr_cobot")
    rospy.sleep(1.0)
    VP.activate()
    rospy.loginfo(f"vacuum controller is on {VP.on}")
    VP.invert()
    rospy.sleep(1.0)
    rospy.loginfo(f"vacuum controller is on {VP.on}")
    VP.invert()
    rospy.sleep(1.0)
    rospy.loginfo(f"vacuum controller is on {VP.on}")
    VP.on = True
    rospy.sleep(1.0)
    rospy.loginfo(f"vacuum controller is on {VP.on}")
    VP.deactivate()
