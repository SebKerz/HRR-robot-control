# /usr/bin/env python3
"""
Controller Manager Interface
---------------------------------

this file contains a collection of helpful commands to communicate with a
`controller manager <http://wiki.ros.org/controller_manager/>`_
to communicate with a robot via `ros_control <http://wiki.ros.org/ros_control/>`_
"""
from functools import partial
import rospy
from controller_manager_msgs.srv import (
    ListControllers, ListControllerTypes,
    LoadController, LoadControllerRequest,
    UnloadController, UnloadControllerRequest,
    SwitchController, SwitchControllerRequest
)

__all__ = ["get_list_controller_manager_handle",
           "get_all_controllers", "print_all_controllers", "all_controller_string",
           "get_known_controller_types",
           "get_controller_names",
           "active_controllers", "inactive_controllers", "check_controllers_active",
           "activate_controller", "deactivate_controller",
           "load_controller", "reload_controller"]

_UNLOADED_STR = "unloaded"


def _parse_srv_name(node_name, srv_name):
    if node_name is None:
        return f"/controller_manager/{srv_name}"
    try:
        if node_name[-1] == "/":
            return f'{node_name}{srv_name}'
        else:
            return f'{node_name}/{srv_name}'
    except IndexError:
        return f"{srv_name}"


class ControllerFilters:

    @staticmethod
    def is_running(x):
        return x.state == "running"

    @staticmethod
    def is_loaded(x):
        return x.state == "initialized"

    @staticmethod
    def is_of_type(x, y):
        return x.type == y


def get_known_controller_types(controller_manger_ns):
    """
    Get all controller types known to the current ROS-environment

    Args:
        controller_manger_ns (string): namespace of controller manager

    Returns:
        list: a list of all controller types. Ignores the base class of the return service
    """
    srv_name = _parse_srv_name(controller_manger_ns, 'list_controller_types')
    rospy.wait_for_service(srv_name)
    list_ctrl_types = rospy.ServiceProxy(srv_name, ListControllerTypes)
    return list_ctrl_types().types


def get_list_controller_manager_handle(controller_manger_ns):
    def py_call(filter_fcn=None):
        x = list_ctrl()
        if filter_fcn is None:
            return x.controller
        return list(filter(filter_fcn, x.controller))

    srv_name = _parse_srv_name(controller_manger_ns, 'list_controllers')
    rospy.wait_for_service(srv_name)
    list_ctrl = rospy.ServiceProxy(srv_name, ListControllers)
    return py_call


def get_controller_names(controller_manger_ns, controller_type=None):
    """
    Get all controllers of a certain type on the controller manager namespace

    Args:
        controller_manger_ns (string): namespace of controller manager
        controller_type (string): controller type as defined in a plugin.xml, i.e. hrr_controllers/CartesianStateController

    Returns:
        list: list of (name, state)-tuples of controller for selected type
    """
    if controller_type is None:
        f_filter = None
    else:
        f_filter = partial(ControllerFilters.is_of_type, y=controller_type)
    ctrl_lister = get_list_controller_manager_handle(controller_manger_ns)
    name_state_list = []
    for x in ctrl_lister(f_filter):
        name_state_list.append((x.name, x.state))
    return name_state_list


def get_all_controllers(controller_manger_ns):
    """
    Get all controllers in ROS. Extends loaded and active controller by all controllers in the parameter server
    may return false values as type is only word in paramterspace used for filtering

    Args:
        controller_manger_ns (string): namespace of controller manager

    Returns:
        list: (name, status) for loaded controllers and (name, type, status) for unloaded controller
    """

    def filter_params_name(ns):
        if 'type' in ns:
            return rospy.get_param(ns) in known_controller_types
        return False

    known_controller_types = get_known_controller_types(controller_manger_ns)
    controllers = get_controller_names(controller_manger_ns)
    loaded_ctrl = [x[0] for x in controllers]
    for par in filter(filter_params_name, rospy.get_param_names()):
        ctrl_name = par.split("/")[-2]
        if ctrl_name not in loaded_ctrl:
            controllers.append((ctrl_name, rospy.get_param(par), _UNLOADED_STR))
    return controllers


def all_controller_string(controller_manager_ns):
    """
    generate human readable string for all controller that are currently loaded, active or unloaded
    as generated from :py:func:`get_all_controllers`

    Args:
        controller_manager_ns (str): namespace of controller manager
    Returns:
        List[str]: list of strings, where each string contains a line / controller information
    """
    str_out = []
    for x in get_all_controllers(controller_manager_ns):
        try:
            ctrl, status = x
            if status == "running":
                str_out.append(f"{ctrl:40}\t=>\t\033[92m running \033[0m")
            else:
                str_out.append(f"{ctrl:40}\t=>\t\033[93m{status}\033[0m")
        except ValueError:
            ctrl, typectrl, _ = x
            str_out.append(f"[unloaded] {ctrl:30}\t[{typectrl}]")
    return str_out


def print_all_controllers(controller_manager_ns):
    """Print wrapper for :py:func:`get_all_controllers`

    Args:
        controller_manager_ns (str): namespace of controller manager
    """
    print("\n".join(all_controller_string(controller_manager_ns)))


def load_controller(controller_manger_ns, controller_name, check=False):
    """load controller by name

    Args:
        controller_manger_ns (string): namespace of controller manager
        controller_name (string): controller name to be loaded. Assumes to be present in controller-manager namespace

    Returns:
        bool: success / return flag from service call
    """
    srv_name = _parse_srv_name(controller_manger_ns, 'load_controller')
    if check:
        if controller_name in active_controllers(controller_manager_ns=controller_manger_ns) or \
          controller_name in inactive_controllers(controller_manager_ns=controller_manger_ns):
            return True
    rospy.wait_for_service(srv_name)
    load_ctrl = rospy.ServiceProxy(srv_name, LoadController)
    res = load_ctrl(LoadControllerRequest(controller_name))
    return res.ok


def reload_controller(controller_manger_ns, controller_name, check=False):
    """reload controller by name, i.e. unload and load

    Args:
        controller_manger_ns (string): namespace of controller manager
        controller_name (string): controller name to be loaded. Assumes to be present in controller-manager namespace

    Returns:
        bool: success / return flag from service call
    """
    if check:
        if not (controller_name in active_controllers(controller_manager_ns=controller_manger_ns) or \
                controller_name in inactive_controllers(controller_manager_ns=controller_manger_ns)):
            return load_controller(controller_manger_ns, controller_name, check=check)
    srv_name = _parse_srv_name(controller_manger_ns, 'unload_controller')
    rospy.wait_for_service(srv_name)
    unload_ctrl = rospy.ServiceProxy(srv_name, UnloadController)
    res = unload_ctrl(UnloadControllerRequest(controller_name))
    if res.ok:
        return load_controller(controller_manger_ns, controller_name, check=check)
    return False


def filter_loaded_controller_by_state(controller_manager_ns, state):
    controllers = []
    for x in get_all_controllers(controller_manager_ns):
        try:
            ctrl, status = x
            if status in state:
                controllers.append(ctrl)
        except ValueError:
            pass
    return controllers


def active_controllers(controller_manager_ns):
    return filter_loaded_controller_by_state(controller_manager_ns, ("running", "active"))


def inactive_controllers(controller_manager_ns):
    return filter_loaded_controller_by_state(controller_manager_ns, ("stopped", "initialized"))


def activate_controller(controller_manger_ns, controller_name):
    """activate controller of a certain type and name

    Args:
        controller_manger_ns (str): namespace of controller manager
        controller_name (str): controller name to be loaded. Assumes to be present in controller-manager namespace

    Returns:
        bool: success / return flag from service call
    """
    srv_name = _parse_srv_name(controller_manger_ns, 'switch_controller')
    rospy.wait_for_service(srv_name)
    controllers = get_all_controllers(controller_manger_ns)
    for x in controllers:
        if x[0] != controller_name:
            continue
        elif x[-1] == "running":
            rospy.loginfo(f'controller {controller_name} is already running. Skipping')
            return False
        if x[-1] == _UNLOADED_STR:
            if not load_controller(controller_manger_ns, controller_name):
                rospy.loginfo(f'controller {controller_name} could not be loaded. Skipping')
                return False
        switch_ctrl = rospy.ServiceProxy(srv_name, SwitchController)
        res = switch_ctrl(SwitchControllerRequest(start_controllers=[controller_name],
                                                  strictness=1))
        return res.ok


def deactivate_controller(controller_manger_ns, controller_name, check_first=False):
    """activate controller of a certain type and name

    Args:
        controller_manger_ns (string): namespace of controller manager
        controller_name (string): controller name to be loaded. Assumes to be present in controller-manager namespace
        check_first(bool): flag to check current controller manager first before calling a switch command

    Returns:
        bool: success / return flag from service call
    """
    call_srv = True
    if check_first:
        controllers = get_all_controllers(controller_manger_ns)
        for x in controllers:
            if x[0] == controller_name and x[-1] != "running":
                rospy.loginfo(f'controller {controller_name} is not running. Skipping')
                call_srv = False
    if call_srv:
        srv_name = _parse_srv_name(controller_manger_ns, 'switch_controller')
        rospy.wait_for_service(srv_name)
        switch_ctrl = rospy.ServiceProxy(srv_name, SwitchController)
        res = switch_ctrl(SwitchControllerRequest(stop_controllers=[controller_name],
                                                  strictness=1, start_asap=True))
        return res.ok
    return False


def check_controllers_active(controller_manager_ns, controller_name):
    cur_ctrls = get_all_controllers(controller_manager_ns)
    for ctrl in cur_ctrls:
        if ctrl[0] == controller_name and ctrl[1] == "running":
            return True
    return False
