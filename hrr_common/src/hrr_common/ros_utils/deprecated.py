""" This file contains deprecated source code that might be of interest but is not in use any longer
"""

import rospy
import numpy as np
import spatialmath as sm
from typing import Tuple, Union
from sim_robots import CartesianHybridForcePoseControl, get_racer_kin_model


from .ros_data_logger import log_helper
from ..utils import get_range, rate_dt_helper, normalize_vec

class HwComplianceController(CartesianHybridForcePoseControl):


    _vel_cmd = 0
    _delta_cmd = 1
    _pos_cmd = 2

    def __init__(self, δt, robot_handle,
                    control_mode='pos', use_rot=True,
                    **kwargs):
        """

        Args:
            δt(float): update time step
            robot_handle(HrrCobotIf): HrrCobot handle
            use_rot(bool, optional): flag to include rotation error / command. Defaults to True.
            kwargs(dict): optional parameter for :py:class:`~CartesianHybridForcePoseControl`.

        """
        self._robot_if = robot_handle
        self._rot_ctrl = use_rot
        self._kin_model = get_racer_kin_model()
        self.control_mode = self.str2int(control_mode)
        self.v_cmd_cur = np.zeros(6)
        super(HwComplianceController, self).__init__(dt=δt, **kwargs)

    def str2int(self, c_mode):
        if isinstance(c_mode, int):
            return c_mode
        if any([x in c_mode.lower() for x in ('pos', 'pose', 'rot')]):
            return self._pos_cmd
        elif any([x in c_mode.lower() for x in ('delta', 'displacement', 'relative')]):
            return self._delta_cmd
        elif any([x in c_mode.lower() for x in ('velocity', 'vel', 'incremental')]):
            return self._vel_cmd
        else:
            raise ValueError(f"unknown control command {c_mode}")

    def command(self, precision_threshold, θ=0.0):
        if self.control_mode == self._vel_cmd:
            θ = max(0., min(θ, 1.0))
            self._robot_if.update_tf()
            ε_pos = (self._admittance_system.T_B_E.t - (
                    self._robot_if.sns_pos + 0.5 * self.v_cmd_cur[:3] * self.dt))
            # v_cmd = θ * self.v_cmd_cur[:3] + (1 - θ) * ε_pos / (20 * self.dt)
            z_vel = np.logical_and(np.abs(ε_pos) < precision_threshold,
                                    np.abs(self._admittance_system._E_x_dot_virtual[:3]) < 1e-4)
            v_virtual = self._robot_if.T_B_E_robot.jacob() @ (0.3 * self._admittance_system._E_x_dot_virtual)
            v_cmd = v_virtual[:3]
            v_cmd[z_vel] = 0.0
            v_cmd[~z_vel] = np.clip(v_cmd[~z_vel], -0.02, 0.02)
            ω_cmd = np.zeros(3)
            if self._rot_ctrl:
                # todo: add rotation twist command here
                pass
            self._robot_if.get_twist_cmd(v_cmd, ω_cmd, publish=True)
            self.v_cmd_cur[:3] = v_cmd
            self.v_cmd_cur[3:] = ω_cmd
        # command delta : self.B_Δ_x
        if self.control_mode == self._delta_cmd:
            if self._rot_ctrl:
                self._robot_if.command_Δu(self.B_Δ_x)
            else:
                self._robot_if.command_Δu(self.B_Δ_x, err_rot_gain=0.0)
            return

        # command pose: Robot_ctrl.virtual_T_B_E
        if self.control_mode == self._pos_cmd:
            if self._rot_ctrl:
                q = sm.UnitQuaternion(sm.base.r2q(self._admittance_system.T_B_E.R))
                self._robot_if.command_pose(self._admittance_system.T_B_E.t, q)
            else:
                self._robot_if.command_pose(self._admittance_system.T_B_E.t)
            return

    def update(self, B_T_des, B_F_des, vel_des=None, impact=np.zeros(6)):
        E_ε_F = np.clip(B_F_des - self.B_F_env, -self._force_err_limit, self._force_err_limit)
        return super().update(B_T_des, E_ε_F, vel_des, impact)

    def reset(self):
        # type: () -> None
        super().reset(self._robot_if.q, self._robot_if.T_B_E_robot)

    @property
    def robot_q(self):
        # type: () -> np.ndarray
        return self._robot_if.q

    @property
    def robot_T_B_E(self):
        # type: () -> sm.SE3
        return self._robot_if.T_B_E_robot

    @property
    def virtual_T_B_E(self):
        return self._admittance_system.T_B_E

    @property
    def virtual_E_x_dot(self):
        return self._admittance_system.E_x_dot

    @property
    def virtual_B_x_dot(self):
        return self.virtual_T_B_E.jacob() @ self._admittance_system.E_x_dot

    def compliant_free_space(self, T_B_E_des, B_F_des=np.zeros(6), N=1000):
        from tqdm import trange
        from .ros_data_logger import init_data_dict
        self._robot_if.reset_bias()
        data = init_data_dict()
        for _ in trange(N):
            contact = self._robot_if.in_contact()
            if contact:
                self.S = (1, 1, 0, 1, 1, 1)
                self.B_F_env = -np.r_[self._robot_if.filtered[:3], np.zeros(3)]
            else:
                self.S = (1, 1, 1, 1, 1, 1)
                self.B_F_env = np.zeros(6)
            log_data(self, self._robot_if.q, self._robot_if.filtered,
                        data, contact, T_B_E_des, B_F_des=B_F_des)
            self.update(T_B_E_des, B_F_des=B_F_des)
            self.command(θ=0., precision_threshold=1e-4)
            rospy.sleep(self.dt)
        return data


def log_vel_errors(cart_ctrl, data, B_v_des=np.zeros(6)):
    """
    Log current ee-velocity error

    Args:
        cart_ctrl(CartesianHybridForcePoseControl): Hybrid Cartesian Force/Pose controller instance
        data(dict): data dictionary that contains trans/rotation error in EE-frame.
        B_v_des(np.ndarray, optional): desired EE-velocity in base frame (B), defaults to array of zeros (6x1)
    """
    log_helper(data, cart_ctrl.E_ε_dot_x(B_v_des), r"\dot{\epsilon}")


def log_pose_errors(cart_ctrl, data, T_B_E_des):
    """
    Log current pose error

    Args:
        cart_ctrl(CartesianHybridForcePoseControl): Hybrid Cartesian Force/Pose controller instance
        data(dict): data dictionary that contains trans/rotation error in EE-frame.
        T_B_E_des(sm.SE3): desired EE-pose in base frame (B)
    """
    log_helper(data, cart_ctrl.E_ε_x(T_B_E_des), r"\epsilon")


def log_errors(cart_ctrl, data, T_B_E_des, B_F_des=np.zeros(6), B_v_des=np.zeros(6)):
    """
    Log current controller errors

    Args:
        cart_ctrl(CartesianHybridForcePoseControl): Hybrid Cartesian Force/Pose controller instance
        data(dict): data dictionary that contains trans/rotation error in EE-frame.
        T_B_E_des(sm.SE3): desired EE-pose in base frame (B)
        B_F_des(np.ndarray, optional): desired EE-velocity in base frame (B), defaults to array of zeros (6x1)
        B_v_des(np.ndarray, optional): desired EE-velocity in base frame (B), defaults to array of zeros (6x1)
    """
    log_pose_errors(cart_ctrl, data, T_B_E_des)
    log_vel_errors(cart_ctrl, data, B_v_des)
    log_helper(data, cart_ctrl.E_Δ_x, r"\Delta")
    log_helper(data, B_F_des - cart_ctrl.B_F_env, r"\epsilon", pos=r"{f}", rot=r"{\tau}")




def log_data(compliance_ctrl, q, f_raw, data, contact, T_B_E_des, B_F_des=np.zeros(6)):
    """log_data 

    Args:
        compliance_ctrl (HwComplianceControl): [description]
        q ([type]): [description]
        f_raw ([type]): [description]
        data ([type]): [description]
        contact ([type]): [description]
        T_B_E_des ([type]): [description]
        B_F_des ([type], optional): [description]. Defaults to np.zeros(6).
    """
    log_errors(compliance_ctrl, data, T_B_E_des, B_F_des=B_F_des)
    data["contacts"].append(contact)
    data["q"].append(q)
    data["f_raw"].append(f_raw)
    data["f_msr"].append(compliance_ctrl.B_F_env)
    data["p_is"].append(compliance_ctrl.robot_T_B_E.t)
    data["dT"].append(compliance_ctrl.B_Δ_x[:3])
    data["v_x"].append(compliance_ctrl.virtual_T_B_E.t)
    data["v_xt"].append(compliance_ctrl.virtual_B_x_dot)


ERROR = -1
SUCCESS = 1
TIMEOUT = -2



# private functions
def log_data(hrr_cobot, data=None, f_e=None):
    if data is not None:
        data['f'].append(np.copy(hrr_cobot.B_F_msr))
        if f_e is not None:
            data['f_e'].append(f_e)



def get_state(hrr_cobot, success_fcn, error_fcn):
    if error_fcn(hrr_cobot.T_B_E_robot, hrr_cobot.B_F_msr):
        return ERROR
    elif success_fcn(hrr_cobot.T_B_E_robot, hrr_cobot.B_F_msr):
        return SUCCESS


def send_cmd(hrr_cobot,state, v_sns, ω_sns=np.zeros(3)):
    if state == SUCCESS or state == ERROR:
        hrr_cobot.get_twist_cmd(np.zeros(3), np.zeros(3), publish=True)
    else:
        hrr_cobot.get_twist_cmd(v_sns, ω_sns, publish=True)


def check_step(cur_state, new_state, sleep_fcn):
    # type: (int, Union[int, None], callable) -> Tuple[Union[int, None], bool]
    if new_state is None:
        sleep_fcn()
        return cur_state, False
    else:
        return new_state, True


def get_force_err(hrr_cobot, force_normal, f_des):
    F_des = force_normal * f_des
    f_err = F_des - hrr_cobot.B_F_ext[:3]
    f_err *= np.abs(force_normal)
    return f_err


def get_torque_err(hrr_cobot, torque_dir, B_τ_des):
    B_M_des = torque_dir * B_τ_des
    err = B_M_des - hrr_cobot.B_F_ext[3:]
    return err * np.abs(torque_dir)


def get_scaled_velocity(hrr_cobot, s, direction):
    return s * hrr_cobot.v_max * direction


def moveStep(hrr_cobot, normal, s, success_fcn, error_fcn, data=None):
    """simple move step with a constant velocity along a normal vector

    Args:
        hrr_cobot (HrrCobotIf): hrr cobot handle
        normal(np.ndarray): normal vector to approach object
        s (float): velocity scaling value
        success_fcn (callable): success function (T_B_E_robot, B_F_msr) -> bool
        error_fcn (callable): error function (T_B_E_robot, B_F_msr) -> bool
        data (dict or None, optional): optional debug dict. Defaults to None.

    Returns:
        int or None: state outcome flag (see above)

    Note:
        this function does not normalize the vector as it is expected to be run in a loop, so use with care!
    """
    v_sns = get_scaled_velocity(hrr_cobot, s, normal)
    hrr_cobot.update_tf()
    log_data(hrr_cobot, data)
    S = get_state(hrr_cobot, success_fcn=success_fcn, error_fcn=error_fcn)
    send_cmd(hrr_cobot, S, v_sns)
    return S


def contact_wrench_step(hrr_cobot, K_f, K_t, normal, f_des, 
                       torque_dir, tau_des,
                       success_fcn, error_fcn, data = None):
    """Contact force step update

    Args:
        hrr_cobot (HrrCobotIf): hrr cobot handle
        K_f (float): proportional gain for force controller
        normal_vec (np.ndarray): normal vector
        f_des (float): desired force magnitude
        data (dict or None, optional): optional debug dict. Defaults to None.
        success_fcn (callable): success function (T_B_E_robot, B_F_msr) -> bool
        error_fcn (callable): error function (T_B_E_robot, B_F_msr) -> bool
    Returns:
        int or None: state transition flag
    """
    f_err = get_force_err(hrr_cobot, normal, f_des)
    t_err = get_torque_err(hrr_cobot, torque_dir, tau_des)
    if data is not None:
        log_data(hrr_cobot, data, f_e=f_err)
    S = get_state(hrr_cobot, success_fcn=success_fcn, error_fcn=error_fcn)
    send_cmd(hrr_cobot, S, K_f * f_err, K_t * t_err)
    return S

def contact_force_step(hrr_cobot, K_f, normal, f_des, success_fcn, error_fcn, data = None):
    """Contact force step update

    Args:
        hrr_cobot (HrrCobotIf): hrr cobot handle
        K_f (float): proportional gain for force controller
        normal_vec (np.ndarray): normal vector
        f_des (float): desired force magnitude
        data (dict or None, optional): optional debug dict. Defaults to None.
        success_fcn (callable): success function (T_B_E_robot, B_F_msr) -> bool
        error_fcn (callable): error function (T_B_E_robot, B_F_msr) -> bool
    Returns:
        int or None: state transition flag
    """
    f_err = get_force_err(hrr_cobot, normal, f_des)
    if data is not None:
        log_data(hrr_cobot, data, f_e=f_err)
    S = get_state(hrr_cobot, success_fcn=success_fcn, error_fcn=error_fcn)


def hybrid_force_pos_step(hrr_cobot, K_f, s, force_normal, velocity_dir,
                          f_des, success_fcn, error_fcn, data = None):
    """Contact force step update

    Args:
        hrr_cobot (HrrCobotIf): hrr cobot handle
        K_f (float): proportional gain for force controller
        force_normal (np.ndarray): normal vector to apply force
        velocity_dir(np.ndarray): velocity steering direction
        f_des (float): desired force magnitude
        success_fcn (callable): success function (T_B_E_robot, B_F_msr) -> bool
        error_fcn (callable): error function (T_B_E_robot, B_F_msr) -> bool
        data (dict or None, optional): optional debug dict. Defaults to None.
    Returns:
        int or None: state transition flag
    """
    v_motion = get_scaled_velocity(hrr_cobot, s, velocity_dir)
    f_err = get_force_err(hrr_cobot, force_normal, f_des)
    if data is not None:
            log_data(hrr_cobot, data, f_e=f_err)
    S = get_state(hrr_cobot, success_fcn=success_fcn, error_fcn=error_fcn)
    send_cmd(hrr_cobot, S, K_f * f_err + v_motion)
    return S




# public functions
def is_success(state):
    if isinstance(state, int):
       return state == SUCCESS
    return 'success' in state


def is_error(state):
    if isinstance(state, int):
       return state == ERROR
    return 'error' in state


def move(hrr_cobot, s, normal, success_fcn=None, error_fcn=None,
         rate = None, dt=None,
         data=None, time_max=20.0):
    r"""
    move object along surface normal vector, where approach direction given as as the surface normal :math:`-\bf{n}`
    and the coordinate system is equal to the sensor-tracking frame, thus ignored here. Using the
    maximum velocity :math:`v_\mathrm{max}` and the scaling term :math:`s`, the sensor-track command is set to

    .. math::

        \dot{\bf{x}} = s v_\mathrm{max} \bf{x}

    Note:
        ! the term `normal` vector is misleading as the current implementation uses the reverted normal vector :math:`=\bf{n}`

    Args:
        normal(np.ndarray): approach direction as a unit vector.
        s(float): scaling value for the maximum velocity
        time_max (float, optional): timeout threshold. Defaults to 20.0.
        dt ([type], optional): temporal increment per step. Defaults to 1e-2.
        v_max (float, optional): maximum velocity. Defaults to 0.01.

    Returns:
        Tuple[int, Union[dict, None]]: status and optional logging dict data
    """
    def default_error(_, f_msr):
        return np.linalg.norm(f_msr) > 100.0

    def default_success(*_):
        if hrr_cobot.in_contact:
            rospy.sleep(1e-1)
            return hrr_cobot.in_contact

    if error_fcn is None:
        error_fcn = default_error
    if success_fcn is None:
        success_fcn = default_success
    dt, loop_sleep = rate_dt_helper(rate, dt)
    assert 0 < s <= 1, "s needs to be in (0, 1]"
    n = normalize_vec(normal)
    N = int(time_max / dt)
    S = TIMEOUT
    for _ in get_range(N):
        S, stop =  check_step(S, moveStep(hrr_cobot, n, s, success_fcn=success_fcn, error_fcn=error_fcn), loop_sleep)
        if stop:
            break
    return S, data


def set_contact_force(hrr_cobot, K_f, force_normal, f_des,
                      rate = None, dt=None,
                      error_fcn=None, success_fcn=None,
                      f_precision=1.0, f_max=60.0,
                      time_max=20.0, data=None, **kwargs):
    """Set contact force over simpile P-controller

    Args:
        K_f (float): gain for normal force P-controller
        data (dict or None, optional): optional debug dict. Defaults to None.
        normal_vec (np.ndarray): normal vector
        f_des (float): desired force magnitude
        r(Union[rospy.Rate, float], optional): rospy rate. Defaults to None
        f_precision (float, optional): precision threshold in N. Defaults to 1.0.
        f_max (float, optional): maximum to throw an error in N. Defaults to 60.0.

    Returns:
        Tuple[int, Union[dict, None]]: status and optional logging dict data
    """
    def default_error(_, f_msr):
        return np.linalg.norm(f_msr) > f_max

    def default_success(_, f_msr):
        return np.linalg.norm(get_force_err(hrr_cobot, force_normal, f_des)) <= f_precision

    if error_fcn is None:
        error_fcn = default_error
    if success_fcn is None:
        success_fcn = default_success
    force_normal = normalize_vec(force_normal)
    dt, loop_sleep = rate_dt_helper(rate, dt)
    N = int(time_max / dt)
    S = TIMEOUT
    for _ in get_range(N):
        S, stop = check_step(S, contact_force_step(hrr_cobot, K_f, normal=force_normal,
                                                   f_des=f_des, data=data,
                                                   error_fcn=error_fcn, success_fcn=success_fcn,
                                                   **kwargs), loop_sleep)
        if stop:
            break
    hrr_cobot.get_twist_cmd(np.zeros(3), np.zeros(3), publish=True)
    return S, data

def set_contact_wrench(hrr_cobot, K_f, K_t, force_normal, f_des,  torque_dir,  tau_des,
                       rate = None, dt=None,
                       error_fcn=None, success_fcn=None,
                       f_precision=1.0, f_max=60.0,
                       time_max=20.0, data=None, **kwargs):
    """Set contact wrench over simpile P-controller.
    More or less 6-DoF version of :py:func:`~set_contact_force`.

    apply Cartesian velocity according to

    TBA

    Args:
        K_f (float): gain for normal force P-controller
        data (dict or None, optional): optional debug dict. Defaults to None.
        normal_vec (np.ndarray): normal vector
        f_des (float): desired force magnitude
        r(Union[rospy.Rate, float], optional): rospy rate. Defaults to None
        f_precision (float, optional): precision threshold in N. Defaults to 1.0.
        f_max (float, optional): maximum to throw an error in N. Defaults to 60.0.

    Returns:
        Tuple[int, Union[dict, None]]: status and optional logging dict data
    """
    def default_error(_, f_msr):
        return np.linalg.norm(f_msr) > f_max

    def default_success(_, f_msr):
        return (np.linalg.norm(get_force_err(hrr_cobot, force_normal, f_des)) <= f_precision and
                np.linalg.norm(get_torque_err(hrr_cobot, force_normal, tau_des)) <= f_precision)


    if error_fcn is None:
        error_fcn = default_error
    if success_fcn is None:
        success_fcn = default_success
    force_normal = normalize_vec(force_normal)
    torque_dir = normalize_vec(torque_dir)
    dt, loop_sleep = rate_dt_helper(rate, dt)
    N = int(time_max / dt)
    S = TIMEOUT
    for _ in get_range(N):
        S, stop = check_step(S, contact_wrench_step(hrr_cobot, K_f=K_f, K_t=K_t, normal=force_normal,
                                                    f_des=f_des, torque_dir=torque_dir, tau_des=tau_des,
                                                    data=data,
                                                    error_fcn=error_fcn, success_fcn=success_fcn,
                                                   **kwargs), loop_sleep)
        if stop:
            break
    hrr_cobot.get_twist_cmd(np.zeros(3), np.zeros(3), publish=True)
    return S, data


def force_velocity_profile(hrr_cobot, K_f, s, force_normal, velocity_dir,
                           f_des, rate = None, dt=None,
                           error_fcn=None, success_fcn=None,
                           f_precision=1.0, f_max=60.0, f_min=2.0,
                           time_max=20.0, data=None, **kwargs):
    """
    Apply a P-controller based super advanced 'force-control' while applying a velocity profile along
    a orthogonal path

    Args:
        K_f (float): gain for normal force P-controller
        data (dict or None, optional): optional debug dict. Defaults to None.
        normal_vec (np.ndarray): normal vector
        f_des (float): desired force magnitude
        r(Union[rospy.Rate, float], optional): rospy rate. Defaults to None
        f_precision (float, optional): precision threshold in N. Defaults to 1.0.
        f_max (float, optional): maximum to throw an error in N. Defaults to 60.0.

    Returns:
        Tuple[int, Union[dict, None]]: status and optional logging dict data
    """
    def default_error(_, f_msr):
        return np.linalg.norm(f_msr[:3]) > f_max or np.linalg.norm(force_normal * f_msr[:3]) < f_min

    def default_success(_, f_msr):
        return np.linalg.norm(get_force_err(hrr_cobot, force_normal, f_des)) <= f_precision

    if error_fcn is None:
        error_fcn = default_error
    if success_fcn is None:
        success_fcn = default_success
    assert 0 < s <= 1, "s needs to be in (0, 1]"
    force_normal = normalize_vec(force_normal)
    velocity_dir = normalize_vec(velocity_dir)
    assert np.dot(velocity_dir, force_normal) == 0.0, "force and velocity profile need to be orthogonal"
    dt, loop_sleep = rate_dt_helper(rate, dt)
    N = int(time_max / dt)
    S = TIMEOUT
    for _ in get_range(N):
        S, stop = check_step(S, hybrid_force_pos_step(hrr_cobot, K_f, s, force_normal=force_normal,
                                                       velocity_dir=velocity_dir, f_des=f_des, data=data,
                                                       error_fcn=error_fcn,success_fcn=success_fcn, **kwargs),
                             loop_sleep)
        if stop:
            break
    hrr_cobot.get_twist_cmd(np.zeros(3), np.zeros(3), publish=True)
    return S, data



