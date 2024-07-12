#!/usr/bin/env python3
"""
Force-Torque Sensor interface and handler
--------------------------------------------

python-handle for the ``ForceTorquePlugin`` or arbitrary
provider from F/T-sensor data readings via a ```geometry_msgs/WrenchStamped``` message.

In detail, this file contains the basic python FT-sensor interfaces
and some simplistic filters:

* moving-average :py:func:`~moving_avg`
* butterworth-filter :py:func:`~butter_filter` from `scipy->signal_butter <https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html>`_
* Savitkzy-Golay-filter :py:func:`FTBuffer.filtered_buf` from `scipy->signal->savgol_filter <https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.savgol_filter.html>`_


Required / Expected ROS-parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``prefix`` usually set to current cobot-namespace, i.e.  "/hrr_cobot/".
Thus, the table below replace ``prefix`` by ``/hrr_cobot/``

==========================  ======================================  ==================================
attribute                   ROS-parameter                           default
==========================  ======================================  ==================================
``topic_name``              ``/hrr_cobot/ft_sensor_ns``             ft_sensor
``joint_names``             ``/hrr_cobot/joints``                   [``joint_1``, ``joint_2``, .. ]
==========================  ======================================  ==================================

For the :py:class:`~ComauRobotState` handle, the following additional parameters
are read / needed

======================================  ==========================================  ==================================
attribute                               ROS-parameter                               default
======================================  ==========================================  ==================================
only needed to extract TP5 topic name   ``/hrr_cobot/comau_state_controller_name``
``base_frame``                          ``/hrr_cobot/base_link_name``               ``base_link``
``sns_frame``                           ``/hrr_cobot/sns_frame``                    ``tcp_controller``
======================================  ==========================================  ==================================


"""
import pickle
from dataclasses import dataclass
import numpy as np
import matplotlib.pylab as plt
from scipy import signal, optimize

import rospy
from geometry_msgs.msg import WrenchStamped

from hrr_common.ros_utils.conversions import np2wrench_stamped, np2wrench
from hrr_common.ros_utils.templates import RosBaseHandle
from hrr_common.ros_utils.helper_handles import fix_prefix, get_param
from hrr_common.utils.file_io import load_pickle, save_pickle

__all__ = ["FTData", "FTBuffer", "free_space_offset_regression", "CalibrationParameters"]


def moving_avg(arr, n):
    """Moving average filter across an array ``arr``

    Args:
        arr (np.ndarray): array to be filtered
        n (int): filter length

    Returns:
        np.ndarray: filtered array
    """
    cumsum = np.cumsum(np.insert(arr, 0, 0))
    return (cumsum[n:] - cumsum[:-n]) / float(n)


def butter_filter(arr, N=4, cutoff_hz=0.06, **filter_kwargs):
    try:
        X = signal.butter(N, cutoff_hz, output='ba', **filter_kwargs)
        a, b = X[:2]
    except TypeError as e:
        rospy.logerr(f"could not get butterworth-filter arguments due to {e}")
        return arr
    return signal.filtfilt(b, a, arr)


@dataclass(eq=True, frozen=True)
class CalibrationParameters:
    bias: np.ndarray = np.zeros(6)
    noise: np.ndarray = np.zeros(6)
    B_grav_vec: np.ndarray = np.zeros(3)
    FT_com: np.ndarray = np.zeros(3)


def free_space_offset_regression(data, R_FT_E, figsize=(8, 10), plot=False, call_plot=False):
    r"""Run regression over batches of collected data

    Using least-square regression for force and torque regression:

    1. use

       .. math::

            {\bf R}^{FT}_{B} \begin{bmatrix}0\\0\\-mg\end{bmatrix} +
                \boldsymbol{F}_\mathrm{bias} = {}^{FT}{\bf F}_{\mathrm{msr}}

       to regress gravity force (limited to :math:`z` component)

       .. math::

           \boldsymbol{R}^{B}_{FT} {}^{FT}\boldsymbol{p}_{FT\rightarrow CoM} \times
               \begin{bmatrix}0\\0\\-mg\end{bmatrix} +  \boldsymbol{\tau}_\mathrm{bias} =
               {}^{FT}{\boldsymbol{\tau}}_{\mathrm{msr}}

       and bias term, s.t. we regress parameter-vector

       .. math::

           \begin{aligned}
                \begin{bmatrix} -mg \\
                    {}^{FT}\boldsymbol{p}_{FT\rightarrow CoM} \\
                    \boldsymbol{F}_\mathrm{bias} \\
                    \boldsymbol{\tau}_\mathrm{bias}
                \end{bmatrix}
                  &\gets \begin{bmatrix}
                    \theta_1 \\
                    \theta_{2:4} \\
                    \theta_{5:7} \\
                    \theta_{8:10} \\
                \end{bmatrix}
        \end{aligned}


    Args:
        data (List[CalibData] or CalibData): batches of data collected to run regression from
        R_FT_E (np.ndarray): constant rotation matrix from end-effector to tool frame
        plot (bool, optional):  optional plotting flag. Defaults to True.
        figsize(tuple, optional): figure size if plot is enabled
        plot(bool, optional): generates plot of regression if True. Defaults to False
        call_plot(bool, optional): calls ``plt.show()`` if True. Defaults to False.

    Returns:
        (CalibrationData, dict): calibration data and figure handles
    """

    def F_grav(θ):
        return R_FT_B @ θ[0:3] + θ[3:6][None, :]

    def M_grav(θ):
        return np.cross(θ[0:3], R_FT_B @ B_grav_vec) + θ[3:6][None, :]

    def J_F(θ):
        return np.linalg.norm(F_grav(θ) - F_full_msr[:, 0:3], axis=-1)

    def J_M(θ):
        return np.linalg.norm(M_grav(θ) - F_full_msr[:, 3:6], axis=-1)

    def plot_calibration():
        fig, axs = plt.subplots(3, 1, figsize=figsize)
        fig.suptitle("FT measures vs gravitation offset")

        axs[0].plot(F_full_msr[:, 0:3], alpha=0.7)
        axs[0].plot(F_c)
        axs[0].legend(
            [f"$F_{{{x}, \\mathrm{{raw}}}}$" for x in ("x", "y", "z")] +
            [f"$F_{{{x}, \\mathrm{{calibrated}}}}$" for x in ("x", "y", "z")],
            ncol=2)
        axs[0].set_title(f'Force regression results')

        axs[1].plot(F_full_msr[:, 3:6], alpha=0.7)
        axs[1].plot(M_c)
        axs[1].legend(
            [f"$M_{{{x}, \\mathrm{{raw}}}}$" for x in ("x", "y", "z")] +
            [f"$M_{{{x}, \\mathrm{{calibrated}}}}$" for x in ("x", "y", "z")],
            ncol=2)
        axs[1].set_title(f'Torque regression results')

        axs[2].plot(F_full_msr[:, 0:3] - F_c)
        axs[2].plot(F_full_msr[:, 3:6] - M_c)
        axs[2].legend(
            [f"$\\varepsilon_{{f, {x}}}$" for x in ("x", "y", "z")] +
            [f"$\\varepsilon_{{\\tau, {x}}}$" for x in ("x", "y", "z")],
            ncol=2
        )
        axs[2].set_title(f'Residuals after calibration')
        return fig, axs

    # bias = np.zeros(6)
    noise = np.zeros(6)
    F_full_msr = np.vstack([x.wrench for x in data]) if isinstance(data, (list, tuple)) else data.wrench
    R_B_E = np.vstack([x.R for x in data]) if isinstance(data, (list, tuple)) else data.R
    bias = 0.7 * np.mean(F_full_msr, axis=0)
    # F_full_msr -= F_avg
    R_FT_B = np.einsum('ij, dkj -> dik', R_FT_E, R_B_E)  # equals:  R_FT_E @ R_B_E.T in batched manner

    rospy.logdebug("Starting Calibration Regression...")
    # force measure regression
    F_calib = optimize.least_squares(J_F, np.r_[np.zeros(2), 1.0, bias[0:3]], gtol=1e-11, xtol=1e-11, ftol=1e-10)
    if not F_calib.success:
        rospy.logwarn(f"could not derive calibration for force-measures: {F_calib.message}")
    B_grav_vec = F_calib.x[0:3]
    bias[0:3] = F_calib.x[3:6]
    F_c = F_grav(np.r_[B_grav_vec, bias[0:3]])
    noise[:3] = np.std(F_full_msr[:, 0:3] - F_c, axis=0)
    rospy.logwarn(f"So far our bias is {bias}")
    # torque measure regression
    M_calib = optimize.least_squares(J_M, np.r_[np.zeros(2), 1e-2, bias[3:6]], gtol=1e-11, xtol=1e-11, ftol=1e-8)
    if not M_calib.success:
        rospy.logwarn(f"could not derive calibration for torque-measures: {M_calib.message}")
    com = M_calib.x[0:3]
    bias[3:6] = M_calib.x[3:6]
    M_c = M_grav(np.r_[com, bias[3:6]])
    noise[3:6] = np.std(F_full_msr[:, 3:6] - M_c, axis=0)
    rospy.logdebug("... finished calibration process")
    calib_data = CalibrationParameters(bias=bias,  # + F_avg,
                                       noise=noise, B_grav_vec=B_grav_vec, FT_com=com)
    if plot:
        f, a = plot_calibration()
        if call_plot:
            plt.show()
        return calib_data, dict(fig=f, axs=a)
    return calib_data, dict()


class FTData(RosBaseHandle):
    """
    Simple handle class to subscribe to FT-sensor data

    """

    def __init__(self, ft_offset=(0., 0., 10.), ft_rot=(0., 0., 3.0 * np.pi / 4.)):
        self._wrench = np.zeros(6)
        self._calib_data = CalibrationParameters()
        self.bias_hack = np.zeros(6)
        self._F_gravity_load = np.zeros(6)
        self._topic_name = "/topic_name"
        self._sub = None
        self.calibrated = False

    def init_ros(self, cobot_prefix, topic_name=None):
        """Initialize ROS interface -> subscribe to raw
        FT-sensor data via subscriber listening to ``topic_name``.

        the ``ft_topic_name`` is obtained from the parameter server via
        ``{cobot_prefix}/{ft_sensor_ns}/ft_topic_name``, e.g.

        ``/hrr_cobot/ft_sensor/ft_topic_name``

        Args:
            cobot_prefix(str): ROS-namespace of current cobot instance
            topic_name(str): name of the FT-sensor ros topic.
        """
        ft_ctrl_ns = fix_prefix(f"{cobot_prefix}{get_param(f'{cobot_prefix}ft_sensor_ns')}")
        topic_name = get_param(f'{ft_ctrl_ns}ft_topic_name', topic_name)
        self._topic_name = f"{cobot_prefix}{topic_name}"
        self._sub = rospy.Subscriber(self._topic_name, WrenchStamped, self.cb)

    @classmethod
    def _from_ros(cls, cobot_prefix, **_):
        out = cls()
        out.init_ros(cobot_prefix=cobot_prefix)
        out.read_params()
        return out

    def update_load(self, R_ft_b):
        self._F_gravity_load[:3] = R_ft_b @ self._calib_data.B_grav_vec
        self._F_gravity_load[3:] = np.cross(self._calib_data.FT_com, self._F_gravity_load[:3])

    def set_offset(self, calibration_params: CalibrationParameters, *_, **__):
        self._calib_data = calibration_params
        try:
            self.set_params()
        except (TypeError, ValueError):
            rospy.logerr("could not ROS-parameter for calibrated sensor")
            pass
        self.calibrated = True

    def save_offset(self, filename):
        save_pickle(self._calib_data, filename, force_overwrite=True)

    def load_offset(self, filename):
        try:
            data = load_pickle(filename)
        except AssertionError as e:
            raise FileNotFoundError(e)
        if isinstance(data, CalibrationParameters):
            self._calib_data = data
            self.calibrated = True
        self.set_params()

    @property
    def offset(self):
        return self._calib_data.bias + self._F_gravity_load + self.bias_hack

    def cb(self, msg):
        # type: (WrenchStamped)->None
        self._wrench[0] = msg.wrench.force.x
        self._wrench[1] = msg.wrench.force.y
        self._wrench[2] = msg.wrench.force.z
        self._wrench[3] = msg.wrench.torque.x
        self._wrench[4] = msg.wrench.torque.y
        self._wrench[5] = msg.wrench.torque.z

    @property
    def f(self):
        return self._wrench[:3]

    @f.setter
    def f(self, value):
        self._wrench[:3] = value

    @property
    def tau(self):
        return self.wrench[3:]

    @tau.setter
    def tau(self, value):
        self._wrench[3:] = value

    @property
    def wrench(self):
        return self._wrench

    @property
    def wrench_calib(self):
        return self._wrench - self.offset

    def set_params(self):
        rospy.set_param(f"{self._topic_name}/B_grav", [float(x) for x in self._calib_data.B_grav_vec])
        rospy.set_param(f"{self._topic_name}/com", [float(x) for x in self._calib_data.FT_com])
        rospy.set_param(f"{self._topic_name}/noise", [float(x) for x in self._calib_data.noise])
        rospy.set_param(f"{self._topic_name}/bias/force", [float(x) for x in self._calib_data.bias[0:3]])
        rospy.set_param(f"{self._topic_name}/bias/torque", [float(x) for x in self._calib_data.bias[3:6]])

    def read_params(self):
        w = self._calib_data.bias.tolist()
        self.set_offset(
            CalibrationParameters(
                bias=np.r_[rospy.get_param(f"{self._topic_name}/bias/force", w[:3]),
                           rospy.get_param(f"{self._topic_name}/bias/torque", w[3:])],
                noise=np.array(rospy.get_param(f"{self._topic_name}/noise", self._calib_data.noise.tolist())),
                B_grav_vec=np.array(
                    rospy.get_param(f"{self._topic_name}/B_grav", self._calib_data.B_grav_vec.tolist())),
                FT_com=np.array(rospy.get_param(f"{self._topic_name}/com", self._calib_data.FT_com.tolist()))
            )
        )

    @property
    def noise(self):
        return self._calib_data.noise

    @property
    def bias(self):
        return self._calib_data.bias

    @property
    def calibration_data(self):
        return self._calib_data

    def __str__(self):
        return f"current force:\t{self.f} [N]\ncurrent torque:\t{self.tau} [Nm]"

    def reset_bias(self):
        """
        """
        rospy.logerr("this function is not available for non-buffered FT-handles")


class FTBuffer(FTData):
    r"""
    Buffer version of FT-handle to use in e.g. calibration routine or filtered FT-calls
    In contrast to :py:class:`~FTData`, this handle stores the sensor-readings in rolling buffer (see :py:meth:`~cb`)

    available filter types can be checked / added in property :py:meth:`~filtered_buf`.
    """

    def __init__(self, buf_size=100, filter_order=2, filter_type='savgol', **FT_kwargs):
        """
        Initialize FT-handle with a fixed buffer size and filter order / type.
        By default, the filter order is directly set from the buffer size.

        Args:
            buf_size (int, optional): buffer size to keep previous data readings for smoothing / filtering. Defaults to 100.
            filter_order (int, optional): filter order. May be irrelevant depending on the filter in use. Defaults to 2.
            filter_type (str, optional): filter type to define filter output. Defaults to 'savgol'.
            FT_kwargs(dict, optional): arguments for :py:class:`~FTData`.
        """
        super(FTBuffer, self).__init__(**FT_kwargs)
        self._cnt = 0
        self.full = False
        if buf_size % 2 == 0:
            buf_size += 1
        self._filter_length = int(buf_size // 2)
        self._filter = filter_type
        if self._filter_length % 2 == 0 and self._filter == 'savgol':
            self._filter_length -= 1
        self._filter_order = filter_order
        self._wrench_buf = np.zeros((6, buf_size))
        self._time = np.zeros(buf_size)
        self._t0 = 0.0
        self._sensor_frame = None

    def init_ros(self, cobot_prefix, topic_name=None):
        """Initialize FT-Buffer. adds the buffer size, that defaults to 
        ``self.N=100`` from the ros-parameter server, i.e. from 
        ``{ft_sensor_ns}/ft_buffer_size``, e.g.
        ``/hrr_cobot/ft_sensor/ft_buffer_size``

        Args:
            cobot_prefix(str): ROS-namespace of cobot instance
            topic_name(str): name of raw F/T-sensor ROS-topic
        """
        super(FTBuffer, self).init_ros(cobot_prefix=cobot_prefix, topic_name=topic_name)
        ft_sensor_ns = fix_prefix(f"{cobot_prefix}{get_param(f'{cobot_prefix}ft_sensor_ns')}")
        self.resize(rospy.get_param(f"{ft_sensor_ns}ft_buffer_size", self.N), drop_values=True)
        self._t0 = rospy.get_time()

    @property
    def N(self):
        return self._wrench_buf.shape[1]

    def cb(self, msg):
        super().cb(msg)
        self._sensor_frame = msg.header.frame_id
        self._wrench_buf = np.roll(self._wrench_buf, -1)
        self._time = np.roll(self._time, -1)
        self._wrench_buf[:, -1] = self.wrench
        self._time[-1] = rospy.get_time() - self._t0
        if not self.full:
            self._cnt += 1
            if self._cnt >= self.N:
                self.full = True

    def resize(self, new_size, drop_values=True):
        if new_size == self.N:
            return
        new_buf = np.zeros((6, new_size))
        if not drop_values:
            buf_len = min(self._wrench_buf.shape[-1], new_buf.shape[-1])
            new_buf[-buf_len:] = self._wrench_buf[-buf_len:]
        else:
            self.full = False
            self._cnt = 0
        self._wrench_buf = new_buf

    def reset(self) -> None:
        self._wrench_buf.fill(0.0)
        self._cnt = 0
        self.full = False
        self.read_params()

    @property
    def data_idxs(self):
        """get buffer indices where data is stored"""
        return np.where(np.linalg.norm(self._wrench_buf, axis=0) > 0)[0]

    @property
    def contact_idxs(self):
        """get buffer indices where a contact was detected"""
        return np.where(self.filtered_buf > np.abs(self._calib_data.noise))

    @property
    def bias_msg(self):
        """Get current sensor bias as a message

        Returns:
            geometry_msgs/Wrench: sensor bias obtained from recordings
        """
        return np2wrench(self._calib_data.bias)

    @property
    def filtered_buf(self):
        """get filtered version of FT-data buffer"""
        f_data = np.zeros(self._wrench_buf.shape)
        if self._filter == "savgol":
            for i in range(6):
                f_data[i, :] = signal.savgol_filter(self._wrench_buf[i, :], self.window_size, self._filter_order)
        else:
            for i in range(6):
                mv_a = moving_avg(self._wrench_buf[i, :], self._filter_length // 2)
                tmp = np.r_[self._wrench_buf[i, :], mv_a[-1] * np.ones(self._filter_length // 2)]
                f_data[i, :] = butter_filter(tmp)[:f_data.shape[1]]
        return f_data

    @property
    def filtered_sensor_data(self):
        """get filtered data without sensor offset"""
        N = 2 * self.window_size - 1
        f_data = np.zeros((6, N))
        for i in range(6):
            f_data[i, :] = signal.savgol_filter(self._wrench_buf[i, -N:], self.window_size, self._filter_order)
        return f_data[:, -1]

    @property
    def filtered(self):
        """get filtered FT-measurement"""
        return self.filtered_sensor_data - self.offset

    @property
    def window_size(self):
        return self._filter_length

    @window_size.setter
    def window_size(self, value):
        self._filter_length = value

    @property
    def in_contact(self):
        return np.any(np.abs(self.filtered) > np.abs(self._calib_data.noise) + 1.0)

    def wrench2msg(self, F, frame=None):
        return np2wrench_stamped(F, self._sensor_frame if frame is None else frame)

    @property
    def μ(self):
        return np.mean(self._wrench_buf, axis=1)

    @property
    def σ(self):
        return np.std(self._wrench_buf, axis=1)

    def reset_bias(self):
        """reset_bias contains a hackish bias reset
        By setting the sensor buffer to sleep depending on the current size, the sensor bias reset to the mean
        of the observed data.

        Note:
            in case this module is used with another sensor, the sleep rate should be adjusted to the reading rate of the sensor
        """
        rospy.sleep(1e-3 * (self._wrench_buf.shape[1] + 1))
        self.bias_hack = self.filtered_sensor_data - self._calib_data.bias - self._F_gravity_load

    def __str__(self):
        return f"{super().__str__()}\ncurrent wrench:\t{self.filtered} [N,Nm]\n=>in contact:\t{self.in_contact}"
