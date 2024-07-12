#!/usr/bin/env python
"""
Shaft Grinder Control
------------------------


"""

import rospy

from hrr_common.ros_utils.helper_handles import get_param

from hrr_controllers.dout_encoder_controller import EncoderController

__all__ = ["ShaftgrinderControl"]


class ShaftgrinderControl(EncoderController):

    def __init__(self):
        super(ShaftgrinderControl, self).__init__(frame_id="shaft_grinder")

    @classmethod
    def _from_ros(cls, cobot_prefix, **__):
        out = cls()
        out.init_ros(controller_name=get_param(f"{cobot_prefix}shaft_grinder_controller_name"),
                     cobot_prefix=cobot_prefix, **__)
        out.timeout = get_param(f"{out.controller_ns}runtime", out.timeout)
        return out

    @property
    def fastest(self):
        return self._id2name[max(self._id2name.keys())]

    @property
    def slowest(self):
        return self._id2name[min(self._id2name.keys())]



if __name__ == "__main__":
    rospy.init_node("test_shaft_grinder")
    ABadChoice4Cutting = ShaftgrinderControl.from_ros(cobot_prefix="/hrr_cobot")
    ABadChoice4Cutting.activate()
    rospy.sleep(1.0)
    ABadChoice4Cutting.mode = "full_speed"
    ABadChoice4Cutting.deactivate()
