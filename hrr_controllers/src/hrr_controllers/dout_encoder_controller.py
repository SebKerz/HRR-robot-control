"""
Encoder Controller Python-Handle
--------------------------------------

This module contains the python interface for the
dout_encoder_controller.cpp controller

Required / Expected ROS-parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Setting the ``prefix`` to the current cobot-namespace, i.e.  "/hrr_cobot/" for brevity,
this controller reads the required parameters from the ROS-controller parameter-servers.
thus, the required parameters narrow down to

.. code-block::python

    controller_ns = fix_prefix(f"{cobot_prefix}/{self.controller_name}")

Thus an inherited class can simply overwrite the ROS-init function


.. code-block:: python

    class ExampleProgramController(EncoderController):

    def __init__(self):
        super(ExampleProgramController, self).__init__(frame_id="foo")

    @classmethod
    def _from_ros(cls, cobot_prefix, **__):
        out = cls()
        out.init_ros(controller_name=rospy.get_param(f"{cobot_prefix}foo_controller_name"), cobot_prefix=cobot_prefix, **__)
        return out


relevant ROS-parameters summarized in short below

* ``get_state_srv_name`` - get state service name of current digital out controller
* ``set_state_srv_name`` - set state service name of current digital out controller
* ``invert_srv_name`` - service name to invert pins of current digital out controller
* ``invert_srv_name`` - service name to invert pins of current digital out controller to start / idle values
* ``enable_encoder_pin`` - digital encoder enabling pin, where HIGH means switched off
* ``encoder_pins`` - digital encoder pins 
* ``program_names`` - available program names as strings
* ``program_numbers`` - available program names as integers from 1-N 
* ``legal_programs`` - allowed programs. In general identical to ``program_numbers``

.. warning:

    counting for the programs start from 1 not 0

"""
from abc import ABC
import rospy

from hrr_common.ros_utils import fix_prefix
from hrr_msgs.srv import (
    SetEncoderState, SetEncoderStateRequest,
    GetDoutControllerState, GetDoutControllerStateRequest, GetDoutControllerStateResponse
)

from hrr_common.ros_utils.templates import BaseController

__all__ = ["EncoderController"]


class EncoderController(BaseController, ABC):

    def __init__(self, frame_id: str):
        super().__init__()
        self.timeout = 0.0
        self._enc_srv = None
        self._state_srv = None
        self._cur_mode = -1
        self._cur_mode_name = "unknown"
        self._id2name = dict()
        self._name2id = dict()
        self._frame_id = frame_id

    def set_encoder_srvs(self, set_srv_name, get_srv_name):
        self._enc_srv = rospy.ServiceProxy(set_srv_name, SetEncoderState)
        self._state_srv = rospy.ServiceProxy(get_srv_name, GetDoutControllerState)

    @property
    def state(self) -> GetDoutControllerStateResponse:
        assert self._state_srv is not None, "please initialize ROS-interfaces"
        return self._state_srv(GetDoutControllerStateRequest())

    @property
    def mode_id(self):
        return self._cur_mode

    @property
    def mode_name(self):
        return self._cur_mode_name

    @property
    def legal_names(self):
        return self._name2id.keys()

    def _update_mode(self):
        assert self._enc_srv is not None
        if not self._enc_srv(self.cmd_srv_req):
            rospy.logerr("failed to update encoder mode")

    @mode_id.setter
    def mode_id(self, value):
        self._cur_mode = value
        if self._id2name:
            try:
                self._cur_mode_name = self._id2name[self._cur_mode]

            except KeyError:
                rospy.logwarn(f"unknown program name for id {self._cur_mode}")
        self._update_mode()

    @mode_name.setter
    def mode_name(self, value):
        self._cur_mode_name = value
        if self._name2id:
            try:
                self._cur_mode = self._name2id[self._cur_mode_name]
            except KeyError:
                rospy.logwarn(f"unknown program id for {self._cur_mode_name}")
        self._update_mode()

    @property
    def cmd_srv_req(self) -> SetEncoderStateRequest:
        req = SetEncoderStateRequest()
        req.header.stamp = rospy.get_rostime()
        req.header.frame_id = self._frame_id
        req.timeout = self.timeout
        req.new_state = self._cur_mode
        if self._cur_mode_name != "unknown":
            req.state_name = self._cur_mode_name
        return req

    def init_ros(self, controller_name, cobot_prefix, *_, **__):
        super().init_ros(cobot_prefix=cobot_prefix, **__)
        self.controller_name = controller_name
        controller_ns = fix_prefix(f"{cobot_prefix}/{self.controller_name}")
        self.set_encoder_srvs(
            set_srv_name=f"{controller_ns}{rospy.get_param(f'{controller_ns}set_state_srv_name')}",
            get_srv_name=f"{controller_ns}{rospy.get_param(f'{controller_ns}get_state_srv_name')}"
        )
        for m_id, m_name in zip(rospy.get_param(f"{controller_ns}program_numbers"),
                                rospy.get_param(f"{controller_ns}program_names")):
            self._id2name[m_id] = m_name
            self._name2id[m_name] = m_id
        self._frame_id = rospy.get_param(f"{controller_ns}frame_id")
