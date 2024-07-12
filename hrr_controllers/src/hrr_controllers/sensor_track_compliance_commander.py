#!/usr/bin/env python3
"""
.. _SNS_TRK_CMPL_CMD:

Sensor Track Compliance Control Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Python-handle for the ``sns_trk_hybrid_vel_force_controller.cpp``
"""
import numpy as np

import rospy
from std_srvs.srv import SetBool, SetBoolRequest
from hrr_msgs.msg import HybridForceVelocityCmdStamped, HybridForceVelocityControlSelect

from hrr_common.ros_utils import np2twist, np2wrench
from hrr_common.ros_utils.templates import BaseController

__all__ = ["SnsTrkComplCmd"]


class SnsTrkComplCmd(BaseController):
    r"""
    Python client handle to communicate with
    ``hrr_controllers/CompliancePoseController``

    This controller follows:

    .. math::

        \begin{aligned}
            {\bf {}^{B}\dot{x}} =
                & {\bf R}^{B}_{C} {\bf S}_F {{\bf R}^{B}_{C}}^{\top}
                    \left( {}^{B}{\bf F}_{\mathrm{des}} - {}^{B}{\bf F}_{\mathrm{cur}}\right ) + \\
                &  {\bf R}^{B}_{C} {\bf S}_v {}^{C}{\bf \dot{x}_{\mathrm{des}}}
        \end{aligned}

    and accepts the following inputs via ROS:

    * reference frame -> which defines :math:`C` and thus :math:`{\bf R}^{B}_{C}`
    * selection matrices :math:`{\bf S}_F`  and :math:`{\bf S}_v`
    * desired force :math:`{}^{B}{\bf F}_{\mathrm{des}}`
    * desired feed-forward velocity :math:`\dot{x}_{\mathrm{des}}`
    * gains are set via ``dynamic_reconfigure``

    Args:
       cmd_topic_name(str, optional): see :py:meth:`~init_ros`
       control_select_topic_name(str, optional): see :py:meth:`~init_ros`
       gain_conf_topic_name(str, optional):see :py:meth:`~init_ros`

    Note:
        in case :math:`{\bf S}_v = \mathbb{1}`, the controller is identical to :py:class:`~SnsTrkCmd`
    """

    def __init__(self, cmd_topic_name=None,
                 control_select_topic_name=None, gain_conf_topic_name=None) -> None:
        super().__init__()
        from hrr_controllers.utils import ControlSelector
        self.S_F = ControlSelector()
        self.S_v = ControlSelector()
        self.F_des = np.zeros(6)
        self.x_dot_des = np.zeros(6)
        self.K_p_diag = np.zeros(6)
        self.K_i_diag = np.zeros(6)
        self._K_prev = self.K_p_diag.copy()
        self.frame = ""
        self._gain_client = None
        self._full_cmd = None
        self._S_cmd = None
        self._ack_srv = None
        if all([x is not None for x in ((cmd_topic_name, control_select_topic_name,
                                         gain_conf_topic_name))]):
            self.init_ros(cmd_topic_name=cmd_topic_name,
                          control_select_topic_name=control_select_topic_name,
                          gain_conf_topic_name=gain_conf_topic_name)

    def init_ros(self, cmd_topic_name, control_select_topic_name,
                 gain_conf_topic_name, *_, cobot_prefix, ft_ack_srv_name=None, **__) -> None:
        """
        Initialize ROS-API

        * command hybrid control commands
        * set control selection matrices
        * update control gains

        Args:
            cmd_topic_name(str): topic name of hybrid control command publisher
            control_select_topic_name(str): topic name of the control selection publisher
            gain_conf_topic_name(str): name of dynamic_configure server.
            ft_ack_srv_name(str, optional): FT-calibration acknowledge service name
            prefix(str, optional): ROS-param prefix
        """
        import dynamic_reconfigure.client
        super().init_ros(cobot_prefix=cobot_prefix)
        self.controller_name = rospy.get_param(f"{cobot_prefix}sns_trk_compl_controller_name")
        self._gain_client = dynamic_reconfigure.client.Client(gain_conf_topic_name,
                                                              timeout=10, config_callback=self.config_cb)
        self._full_cmd = rospy.Publisher(
            cmd_topic_name, HybridForceVelocityCmdStamped, queue_size=10)
        self._S_cmd = rospy.Publisher(
            control_select_topic_name, HybridForceVelocityControlSelect, queue_size=10)
        if ft_ack_srv_name is not None:
            self._ack_srv = rospy.ServiceProxy(ft_ack_srv_name, SetBool)

    def config_cb(self, config):
        """callback for the configuration -> update internal control gains
        according to actual control gains
        """
        for i, x in enumerate(("x", "y", "z")):
            try:
                self.K_p_diag[i] = config[f"K_P_f{x}"]
                self.K_p_diag[i + 3] = config[f"K_P_t{x}"]
                self.K_i_diag[i] = config[f"K_I_f{x}"]
                self.K_i_diag[i + 3] = config[f"K_I_t{x}"]
            except KeyError as e:
                rospy.logerr(f"failed on updating {e}")
        try:
            rospy.loginfo(
                "Updated force  gains: K_p:=[{K_P_fx}, {K_P_fy}, {K_P_fz}]".format(**config))
            rospy.loginfo(
                "Updated torque gains: K_p:=[{K_P_tx}, {K_P_ty}, {K_P_tz}]".format(**config))
            rospy.logdebug(
                "Updated force integral gains: K_p:=[{K_I_fx}, {K_I_fy}, {K_I_fz}]".format(**config))
            rospy.logdebug(
                "Updated torque integral gains: K_p:=[{K_I_tx}, {K_I_ty}, {K_I_tz}]".format(**config))
        except KeyError as e:
            print(str(e), "\n", config)

    @classmethod
    def _from_ros(cls, cobot_prefix, **_):
        out = cls()
        out.init_ros(rospy.get_param(f"{cobot_prefix}cmd_topic_name"),
                     rospy.get_param(f"{cobot_prefix}control_select_topic_name"),
                     rospy.get_param(f"{cobot_prefix}gain_cfg_name"), cobot_prefix=cobot_prefix,
                     ft_ack_srv_name=rospy.get_param(f"{cobot_prefix}ft_ack_srv_name")
                     )
        return out

    @property
    def S_msg(self):
        """
        Selector matrix as ROS msgs

        Intended usage

        .. code-block:: 
            python

            ctrl = SnsTrkComplCommand.from_ros()
            ctrl.S_vel = np.ones(1)

        sets the values, while

        .. code-block::
            python

            ctrl.update_cmd(full=False)

        sends the selection matrix to the robot (c.f. :py:meth:`~update_cmd`).

        Returns:
            HybridForceVelocityControlSelect: control selection message
        """
        msg = HybridForceVelocityControlSelect()
        msg.S_vel = [s > 0 for s in self.S_v.S_diag]
        msg.S_force = [s > 0 for s in self.S_F.S_diag]
        return msg

    @property
    def ctrl_msg(self):
        """
        Current content as HybridForceVelocityCmdStamped message
        Intended to set class attributes as needed and then send them to
        the robot via

        .. code-block::
            python

            ctrl = SnsTrkComplCommand.from_ros()
            ctrl.F_cmd[2] = 10.0
            ctrl.update_cmd(True)

        see :py:meth:`~update_cmd`.

        Returns:
            HybridForceVelocityCmdStamped: current class attributes as ROS-msg
        """
        msg = HybridForceVelocityCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = self.frame
        msg.cmd.vel_cmd = np2twist(self.x_dot_des)
        msg.cmd.F_cmd = np2wrench(self.F_des)
        msg.cmd.select = self.S_msg
        return msg

    @property
    def _p_gain_config(self):
        return dict(K_P_fx=self.K_p_diag[0],
                    K_P_fy=self.K_p_diag[1],
                    K_P_fz=self.K_p_diag[2],
                    K_P_tx=self.K_p_diag[3],
                    K_P_ty=self.K_p_diag[4],
                    K_P_tz=self.K_p_diag[5])

    @property
    def _i_gain_config(self):
        return dict(K_I_fx=self.K_i_diag[0],
                    K_I_fy=self.K_i_diag[1],
                    K_I_fz=self.K_i_diag[2],
                    K_I_tx=self.K_i_diag[3],
                    K_I_ty=self.K_i_diag[4],
                    K_I_tz=self.K_i_diag[5])

    def update_gains(self, mode=None) -> None:
        """
        Update gains via the ``dynamic-reconfigure`` client.

        Available modes:

        * ``I``: only update integral gains
        * ``full``: update P and I gains
        * default: update P-gains

        Args:
            mode(str,optional): update mode
        """
        assert self._gain_client is not None, "please run `ros_init` on class instance"
        if mode == "I":
            self._gain_client.update_configuration(self._i_gain_config)
        elif mode == "full":
            self._gain_client.update_configuration(
                self._p_gain_config.update(self._i_gain_config))
        else:
            self._gain_client.update_configuration(self._p_gain_config)

    def update_cmd(self, full=True) -> None:
        """Update command message sent to the robot

        #. in case the proportional gains have changed since last call, update P-gains via :py:meth:`~update_gains`.
        #. check mode:

            * ``full`` -> True: send full control command as parsed via :py:meth:`~ctrl_msg`
            * ``full`` -> False: send selection matrix message as parsed via :py:meth:`~S_msg`

        Args:
            full(bool, optional): Flag to send full command message or only selection matrix
        """
        if np.any(self._K_prev != self.K_p_diag):
            self.update_gains("P")
        if full:
            assert self._full_cmd is not None, "please run `ros_init` on class instance"
            self._full_cmd.publish(self.ctrl_msg)
        else:
            assert self._S_cmd is not None, "please run `ros_init` on class instance"
            self._S_cmd.publish(self.S_msg)
        self._K_prev = np.copy(self.K_p_diag)

    def acknowledge_calibration(self):
        if self._ack_srv is not None:
            self._ack_srv(SetBoolRequest(data=True))
        else:
            raise RuntimeWarning("cannot acknowledge calibration -> Service is not initialized")
