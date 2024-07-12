#!/usr/bin/env python3
"""
Tool Changer Controller Using a GIMATIC EQ20A/B
---------------------------------------------------
"""
import numpy as np
import rospy

from hrr_controllers.dout_encoder_controller import EncoderController
from hrr_controllers.dout_controller_handle import DoutController

__all__ = ["GimaticEncoderControl", "GimaticPinController"]


class GimaticEncoderControl(EncoderController):

    def __init__(self):
        super(GimaticEncoderControl, self).__init__(frame_id="tool_changer")

    @classmethod
    def _from_ros(cls, cobot_prefix, **__):
        out = cls()
        out.init_ros(controller_name=rospy.get_param(f"{cobot_prefix}gimatic_encoder_controller_name"),
                     cobot_prefix=cobot_prefix, **__)
        return out

    @property
    def open(self):
        state = self.state
        return state.enabled and str(self._name2id['open'] - 1) in state.msg


class GimaticPinController(DoutController):

    def __init__(self):
        super(GimaticPinController, self).__init__(frame_id="tool_changer")
        self._open = False

    @classmethod
    def _from_ros(cls, cobot_prefix, **__):
        out = cls()
        out.init_ros(controller_name=rospy.get_param(f"{cobot_prefix}gimatic_pin_controller_name"),
                     cobot_prefix=cobot_prefix, **__)
        return out

    @property
    def mode_name(self):
        if self._open:
            return "open"
        return "close"

    @mode_name.setter
    def mode_name(self, value):
        if value == "open":
            self.invert_start_state()
            return
        elif value == "close":
            self.set_to_start_state()
            return
        raise ValueError(f"can only set to open and close, received {value}")

    @property
    def open(self):
        state = self.state
        return all([ps.state != self._pin_start_values[np.where(self._pins == ps.pin)[0]] for ps in state.pins])


if __name__ == "__main__":
    rospy.init_node("test_grimatic_controller")
    GE = GimaticEncoderControl.from_ros(cobot_prefix="/hrr_cobot")
    GP = GimaticPinController.from_ros(cobot_prefix="/hrr_cobot")
    GE.activate()
    GE.mode = "open"
    rospy.sleep(1.0)
    GE.mode = "close"
    GE.deactivate()
    rospy.sleep(1.0)
    GP.activate()
    GP.invert()
    rospy.sleep(1.0)
    GP.invert()
    GP.deactivate()
