"""
Utility function and helpers
---------------------------------

This module contains a list of useful, yet fully standalone functions that cen be used
throughout code / implementation.
Each function is (actually should be) self-explanatory.
If not, please contact the author(s)/maintainer(s).
"""
from typing import Union
import numpy as np

import rospy
from comau_msgs.msg import TP5State
from geometry_msgs.msg import Vector3

from hr_recycler_msgs.msg import ToolType
from hrr_common.ros_utils import activate_controller, check_controllers_active

__all__ = [
    "check_scale",
    "check_gain",
    "check_gain_matrix",
    "load_default_parameters",
    "get_default_params",
    "ros_param_loader",
    "get_cobot_ns",
    "get_controller_manager_name",
    "tool2tool_type",
    "tool_type2str",
]


def check_scale(s, non_zero=False):
    """
    Helper function to check if scaling term ``s`` is within limits.
    If the ``non_zero`` flag is set, the scaling term needs to be positive.

    Args:
        s(float, int):  scaling term
        non_zero(bool, optional): flag to enforce a positive scaling term. Defaults to False.

    Returns:
        bool: True if ``s`` is a valid scaling term.
    """
    if non_zero:
        assert 0 < s <= 1.0, "scaling needs to be in (0, 1]i"
    else:
        assert 0 <= s <= 1.0, "scaling needs to be in [0, 1]"


def check_gain(K, non_zero=False):
    """
    Helper function to check if scalar control gain ``K`` is non-negative
    If the ``non_zero`` flag is set, the term needs to be positive.

    Args:
        K(float, int):  scalar control gain
        non_zero(bool, optional): flag to enforce a positive scaling term. Defaults to False.

    Returns:
        bool: True if ``K`` is a valid control gain.
    """
    if non_zero:
        assert K > 0, "gains need to be positive"
    else:
        assert K >= 0, "negative gains are not allowed"


def check_gain_matrix(K, non_zero=False):
    """
    Matrix version of :py:meth:`~check_gain` checking if ``K``
    positive (semi-) definite

    Args:
        K(np.ndarray):  scalar control gain matrix
        non_zero(bool, optional): flag to enforce a positive scaling term. Defaults to False.

    Returns:
        bool: True if ``K`` is a valid control gain matrix.
    """
    if non_zero:
        assert np.all(
            np.linalg.eigvals(K) > 0
        ), "gain matrix needs to be positive definite"
    else:
        assert np.all(
            np.linalg.eigvals(K) >= 0
        ), "gain matrix needs to be positive semi-definite"


def get_cobot_ns(cobot_ns=None):
    if cobot_ns is None:
        return rospy.get_param("~cobot_ns")
    return cobot_ns


def get_controller_manager_name(cobot_ns):
    try:
        return cobot_ns + "/controller_manager"
    except TypeError:
        return get_cobot_ns() + "/controller_manager"


def get_cobot_status_topic(cobot_ns, state_controller):
    return "{}/{}/comau_cartesian_data".format(cobot_ns, state_controller)


class SimpleStatus:
    def __init__(self, status_topic):
        self.msg = TP5State()
        self._sub = rospy.Subscriber(status_topic, TP5State, self.state_cb)

    @property
    def status(self):
        return self.msg.status.status

    def state_cb(self, msg):
        self.msg = msg


def assert_state_controller(controller_manager_ns, state_controller_name, cobot_ns):
    """checks if the state controller is running at the robot. This needs to be checked for
    a proper controller enabling / setup upon startup

    For further insights about the communication with the controller manager, please refer to
    :py:func:`hrr_common.ros_utils.controller_manager:activate_controller` for the
    activation of the controller and :py:func:`hrr_common.ros_utils.check_controllers_active`
    on how to check the currently active controllers.

    Args:
        controller_manager_ns(str): namespace of the controller manager, e.g. `/hrr_cobot/controller_manager`
        state_controller_name(str): name of the state controller, e.g. `cartesian_robot_state_controller`
        cobot_ns(str): namespace of the robot, e.g. `hrr_cobot`. only needed for info-print
    Returns:
        str: full ROS-resolved name of status-topic, e.g. `/hrr_cobot/cartesian_robot_state_controller/comau_carteisan_data`
    """
    state_running = check_controllers_active(
        controller_manager_ns, state_controller_name
    )
    if not state_running:
        rospy.loginfo(
            "state controller %s not running @ %s", state_controller_name, cobot_ns
        )
        activate_controller(controller_manager_ns, state_controller_name)
        rospy.sleep(1.0)
    return get_cobot_status_topic(get_cobot_ns(), state_controller_name)


def get_default_params(cobot_ns=None):
    """
    Get default ROS-parameters for the ``hrr_cobot`` setup as a dictionary
    This function returns all default ROS-parameters for the current cobot setup.
    It is also used within :py:meth:`~load_default_parameters` to load all these
    parameters in the local namespace of a current ROS node.

    All static, i.e. robot independent parameters are returned as well as the cobot
    dependent ones, in case the actual namespace of the cobot is provided via ``cobot_ns``,
    such as topic-names, controller manager, service names in the private namespace of the current
    ROS-node.

    Note:
        This function allows to set the cobot_ns with arbitrary "/" convention but expects the namespace to
        be global, i.e. the cobot-namespace should start from "/" - the global ROS-namespace root.


    Returns:
        dict: parameters to be loaded to the ROS-parameter server
    """
    DeprecationWarning("do not use this function!")
    ros_params = {
        "sns_frame": f"tcp_controller",
        "base_frame": f"base_link",
        "joint_names": [f"joint_{i}" for i in range(1, 7)],
        "sns_trk_vel_controller_name": "sensor_track_velocity_controller",
        "sns_trk_compl_controller_name": "sensor_track_compliant_controller",
        "joint_trajectory_handler_name": "joint_trajectory_handler",
        "comau_state_controller_name": "comau_robot_state_controller",
        "read_ft": True,
        "cmd_sns_vel": True,
        "cmd_sns_compl": True,
        "read_tp5": True,
        "cmd_joint_handler": True,
        "v_max": 0.01,
        "omega_max": 0.1,
        "sns_link_name": "tcp_controller",
        "base_link_name": "base_link",
        "ft_link_name": "jr3msr",
        "tip_link_name": "tcp_controller",
        "tool": "",
        "ee_tool_translation": [0.0, 0.0, 0.0],
    }
    # recheck if parameters have been alternated
    try:
        for k, v in filter(
            lambda kv: "handler" in kv[0] or "controller" in kv[0], ros_params
        ):
            ros_params[k] = rospy.get_param(k, v)
    except ConnectionRefusedError:
        rospy.logerr(
            "could not establish connection with ROS parameter server. Assuming defaults"
        )
    # cobot dependent ROS-parameters
    if cobot_ns is not None:
        cobot_ns = cobot_ns.replace("/", "")
        cobot_ns = f"/{cobot_ns}"
        ros_params["cobot_ns"] = cobot_ns
        ros_params["gain_cfg_name"] = f"{cobot_ns}"
        ros_params["joint_state_topic_name"] = f"{cobot_ns}/joint_states"
        ros_params["controller_manager_ns"] = f"{cobot_ns}/controller_manager"
        ros_params["ft_sensor_topic_name"] = f"{cobot_ns}/ft_sensor"
        ros_params["unscrew_action_srv_name"] = f"{cobot_ns}/skills/unscrew"
        ros_params["gripper_ns"] = f"{cobot_ns}/gripper"
        ros_params["gripper_alignment_watcher_node"] = f"{cobot_ns}/alignment_watcher"
        ros_params["gripper_alignment_ctrl_loop_hz"] = 50
        ros_params["integral_window_size"] = 50
        ros_params[
            "gripper_alignment_error_topic"
        ] = f"{ros_params['gripper_ns']}/alignment_error"
        ros_params[
            "sns_trk_topic_name"
        ] = f"{cobot_ns}/{ros_params['sns_trk_vel_controller_name']}/tcp_v_cmd"
        ros_params[
            "tp5_topic_name"
        ] = f"{cobot_ns}/{ros_params['comau_state_controller_name']}/comau_cartesian_data"
        ros_params[
            "control_select_topic_name"
        ] = f"{cobot_ns}/{ros_params['sns_trk_compl_controller_name']}/S"
        ros_params[
            "cmd_topic_name"
        ] = f"{cobot_ns}/{ros_params['sns_trk_compl_controller_name']}/hybrid_ctrl_cmd"
        ros_params[
            "ft_ack_srv_name"
        ] = f"{cobot_ns}/{ros_params['sns_trk_compl_controller_name']}/acknowledge_calibration"
        ros_params[
            "joint_trajectory_action_topic_name"
        ] = f"{cobot_ns}/{ros_params['joint_trajectory_handler_name']}/comau_joint_trajectory_handler"

    return ros_params


def ros_param_loader(params_dict, cobot_prefix) -> None:
    """
    ROS-parameter loader helper.
    Expects a dictionary as e.g. provided by :py:meth:`~get_default_params`
    of parameter-name-> value style.

    The prefix defaults to ``~`` which is a private ROS-parameter for the current ROS-node,
    but could be made globally via "" or "/" or chosen arbitrarily.

    Args:
        params_dict(dict):  parameter dictionary
        prefix(str, optional): prefix to define namespace of parameter
    """
    for k, v in params_dict.items():
        try:
            if cobot_prefix in k and k[0: len(cobot_prefix)] == cobot_prefix:
                rospy.set_param(f"{k}", v)
            else:
                raise IndexError
        except (IndexError, TypeError):
            rospy.set_param(f"{cobot_prefix}{k}", v)


def load_default_parameters(cobot_ns="/hrr_cobot"):
    """
    Load the default ROS parameters to the ROS parameter server
    This function expects a ``cobot_ns`` parameter

    Note:
        this function only allows local / private parameters to be set

    Args:
        cobot_ns(str, optional): namespace of the robot, e.g. "hrr-cobot", "/hrr_cobot". Defaults to "/hrr_cobot".
    """
    params = get_default_params(cobot_ns)
    raise RuntimeError("do not use this function!")
    # ros_param_loader(params, cobot_prefix)


def tool2tool_type(tool: Union[int, str, None]) -> int:
    """
    transform tool to ROS and internal based ``ToolType`` representation

    Args:
        tool: tool as string or ID

    Returns:
        tool-ID according to message definition
    Raises:
        RuntimeError: if incorrect string is provided
        AssertionError: if tool-ID is invalid
        TypeError: if incorrect data type is provided
    """
    if tool is None or tool == "" or tool == "nothing":
        return ToolType.NONE
    if isinstance(tool, str):
        if tool.lower() in ("shaft_grinder", "shaft-grinder", "shaftgrinder"):
            return ToolType.SHAFT_GRINDER
        elif tool.lower() in ("screw_driver", "screwdriver", "screw-driver"):
            return ToolType.SCREW_DRIVER
        elif tool.lower() in ("vacuum", "vacuum-gripper", "vacuum_gripper"):
            return ToolType.VACUUM_GRIPPER
        elif tool.lower() in ("wsg_50_dsa", "wsg-50-dsa", "dsa-gripper"):
            return ToolType.WSG_50_DSA
        elif tool.lower() in ("wsg_50", "default_gripper", "gripper"):
            return ToolType.WSG_50
        else:
            raise RuntimeError(f"unknown tool {tool}")
    elif isinstance(tool, int):
        assert tool in (
            ToolType.NONE,
            ToolType.WSG_50,
            ToolType.WSG_50_DSA,
            ToolType.SHAFT_GRINDER,
            ToolType.SCREW_DRIVER,
            ToolType.VACUUM_GRIPPER,
        ), f"unknown tool-ID {tool}"
        return tool
    else:
        raise TypeError(f"cannot assign {type(tool)} to ``tool``")


def tool_type2str(tool_type: int) -> str:
    if tool_type == ToolType.NONE:
        return ""
    elif tool_type == ToolType.WSG_50:
        return "wsg_50"
    elif tool_type == ToolType.WSG_50_DSA:
        return "wsg_50_dsa"
    elif tool_type == ToolType.SCREW_DRIVER:
        return "screwdriver"
    elif tool_type == ToolType.SHAFT_GRINDER:
        return "shaft_grinder"
    elif tool_type == ToolType.VACUUM_GRIPPER:
        return "vacuum"
    raise ValueError(f"Unknown tool ID {tool_type}")
