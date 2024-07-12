"""
Dout Controller Python-Handle
--------------------------------------

This module contains the python interface for the
dout_controller.cpp controller

Required / Expected ROS-parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Setting the ``prefix`` to the current cobot-namespace, i.e.  "/hrr_cobot/" for brevity,
this controller reads the required parameters from the ROS-controller parameter-servers.
thus, the required parameters narrow down to

.. code-block::python

    controller_ns = fix_prefix(f"{cobot_prefix}/{self.controller_name}")

Thus an inherited class can simply overwrite the ROS-init function

.. code-block::python


    class SimplePinController(DoutController):

        def __init__(self):
            super(SimplePinController, self).__init__(frame_id="digital_stuff")

        @classmethod
        def _from_ros(cls, cobot_prefix, **__):
            out = cls()
            out.init_ros(controller_name=rospy.get_param(f"{cobot_prefix}vacuum_valve_pin_controller_name"), cobot_prefix=cobot_prefix, **__)
            return out

relevant ROS-parameters summarized in short below

* ``get_state_srv_name`` - get state service name of current digital out controller
* ``set_state_srv_name`` - set state service name of current digital out controller
* ``invert_srv_name`` - service name to invert pins of current digital out controller
* ``invert_srv_name`` - service name to invert pins of current digital out controller to start / idle values
* ``pins`` - pins in use for current controller
* ``pin_start_values`` - pin start values

Note:

    ..code-block:: python

        handle.reset()
        handle.deactivate()

    should exhibit the identical behavior. If not, there is an implementation issue
"""
import abc
import numpy as np

import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from comau_msgs.msg import Digital
from hrr_common.ros_utils import fix_prefix
from hrr_msgs.srv import (
    SetPins, SetPinsRequest, SetPinsResponse,
    GetDoutControllerState, GetDoutControllerStateRequest, GetDoutControllerStateResponse
)

from hrr_common.ros_utils.templates import BaseController

__all__ = ["DoutController"]


class DoutController(BaseController, abc.ABC):

    def __init__(self, frame_id: str):
        super().__init__()
        self.timeout = 0.0
        self._get_state_srv = None
        self._invert_srv = None
        self._reset_srv = None
        self._set_srv = None
        self._frame_id = frame_id
        self._pins = np.array([], np.int)
        self._pin_start_values = np.array([], np.int)

    def set_srvs(self, get_srv_name, invert_srv_name, reset_srv_name, set_srv_name):
        self._get_state_srv = rospy.ServiceProxy(get_srv_name, GetDoutControllerState)
        self._invert_srv = rospy.ServiceProxy(invert_srv_name, Trigger)
        self._reset_srv = rospy.ServiceProxy(reset_srv_name, Trigger)
        self._set_srv = rospy.ServiceProxy(set_srv_name, SetPins)

    @property
    def state(self) -> GetDoutControllerStateResponse:
        assert self._get_state_srv is not None, "please initialize ROS-interfaces"
        return self._get_state_srv(GetDoutControllerStateRequest())

    def reset(self):
        assert self._reset_srv is not None, "please initialize ROS-interfaces"
        self._reset_srv(TriggerRequest())

    def invert(self):
        assert self._invert_srv is not None, "please initialize ROS-interfaces"
        self._invert_srv(TriggerRequest())

    def activate(self):
        super(DoutController, self).activate()
        self.reset()

    def deactivate(self):
        self.reset()
        super(DoutController, self).deactivate()

    def set_to_start_state(self):
        req = SetPinsRequest()
        req.state = req.IDLE
        for pin, val in zip(self._pins, self._pin_start_values):
            req.pins_desired.append(Digital(pin=pin, state=val > 0))
        self._set_srv(req)

    def invert_start_state(self):
        req = SetPinsRequest()
        for pin, val in zip(self._pins, self._pin_start_values):
            req.pins_desired.append(Digital(pin=pin, state=val <= 0))
        self._set_srv(req)

    def init_ros(self, cobot_prefix, controller_name, *_, **__):
        super().init_ros(cobot_prefix=cobot_prefix, **__)
        self.controller_name = controller_name
        controller_ns = fix_prefix(f"{cobot_prefix}/{self.controller_name}")
        self.set_srvs(
            get_srv_name=f"{controller_ns}{rospy.get_param(f'{controller_ns}get_state_srv_name')}",
            set_srv_name=f"{controller_ns}{rospy.get_param(f'{controller_ns}set_state_srv_name')}",
            invert_srv_name=f"{controller_ns}{rospy.get_param(f'{controller_ns}invert_srv_name', 'invert')}",
            reset_srv_name=f"{controller_ns}{rospy.get_param(f'{controller_ns}reset_srv_name', 'reset')}",
        )
        pin_start_values: [1, 0]
        pins = rospy.get_param(f"{controller_ns}pins")
        self._pins = np.resize(self._pins, len(pins))
        self._pin_start_values = np.resize(self._pins, len(pins))
        self._pins[:] = pins
        self._pin_start_values[:] = rospy.get_param(f"{controller_ns}pin_start_values")
        self._frame_id = rospy.get_param(f"{controller_ns}frame_id")



