"""
Hybrid Control Class Handles
------------------------------


"""
from dataclasses import dataclass
import numpy as np
from geometry_msgs.msg import Vector3

__all__ = ["ControlSelector", "ForceControlHandle",
           "VelocityControlHandle", "matrix_parser"]


def matrix_parser(mat, value, val_max, val_min=0.0) -> None:
    """
    Helper function to parse data contained in ``value`` to ``mat`` while also clipping the values
    against limits ``val_max`` and ``val_min``.

    Args:
        mat(np.ndarray): numpy array to be filled
        value(Any): value to parse
        val_max(float): clipping upper limit
        val_min(float, optional): lower clipping limit
    """

    def _clip_helper(val):
        if np.isnan(val):
            return 0.0
        return min(max(val_min, val), val_max)

    if np.isscalar(value):
        mat[:] = _clip_helper(value)
    elif isinstance(value, Vector3):
        for i, x in enumerate(("x", "y", "z")):
            mat[i] = _clip_helper(getattr(value, x))
    elif isinstance(value, np.ndarray):
        vec_norm = np.linalg.norm(value)
        mat[~np.isnan(value)] = value[~np.isnan(value)]
        mat[np.isnan(value)] = 0.0
        if isinstance(val_max, np.ndarray) and isinstance(val_min, np.ndarray):
            mat[:] = np.clip(mat, val_min, val_max)
        else:
            scale = val_max if val_min == 0.0 else 0.5 * (val_max - val_min)
            if 0.0 < scale < vec_norm:
                bias = 0.0
                if val_min != 0.0:
                    bias = 0.5 * (val_max + val_min)
                    mat -= bias
                mat *= scale / vec_norm
                mat += bias
    elif isinstance(value, (list, tuple)):
        for i, x in enumerate(value):
            if isinstance(x, bool):
                mat[i] = float(x)
            else:
                mat[i] = x
    else:
        raise TypeError(f"cannot assign value {value} to {mat}")


@dataclass
class ControlSelector:
    """
    This class wraps a control selector as a diagonal 6-dimensional matrix
    for Cartesian robot control

    In here any value for S > 0 enables the dedicated control signal

    values are parsed via the function :py:meth:`~matrix_parser` and code wise

    .. code-blocK::
        python

        tmp = ControlSelector()
        tmp.S = np.ones(6)
        tmp.S_trans

        array([[1., 0., 0.],
               [0., 1., 0.],
               [0., 0., 1.]])
    """

    def __init__(self, dim=6):
        self.S_diag: np.ndarray = np.zeros(dim)

    @property
    def S_trans(self):
        return np.diag(self.S_diag[0:3] > 0)

    @property
    def S_rot(self):
        return np.diag(self.S_diag[3:6] > 0)

    @property
    def S(self):
        S_t, S_r = self.rotate_selection_matrix(np.eye(3))
        return np.r_[[S_t, np.zeros((3, 3))],
                     [np.zeros((3, 3)), S_r]]

    @S.setter
    def S(self, value):
        for i, x in enumerate(value):
            self.S_diag[i] = 1. if x else 0.

    def rotate_selection_matrix(self, R_B_C):
        r"""
        the matrix ``R_B_C`` defines the rotation from body frame to the actual control frame
        Thus the translation component and rotation component of the diagonal selection matrix
        :math:`\bf{S}` is rotated according to

        .. math::

            S_{p} = R^B_C \cdot
                \begin{bmatrix}
                    s_1 & 0 & 0\\
                    0 & s_2 & 0\\
                    0 &  0& s_3
                \end{bmatrix}
                \cdot  {R^B_C}^\top

        for the translational component.
        The final rotated matrix is given as a block-matrix

        .. math::
                \begin{bmatrix}
                    \bf{S_p} & \bf{0}\\
                    \bf{0} & \bf{S_r}\\
                \end{bmatrix}

        where the diagonal matrices are the values calculated in this functions
        """
        return R_B_C @ self.S_trans @ R_B_C.T, R_B_C @ self.S_rot @ R_B_C.T

    def orthogonal(self, other):
        r"""
        check against orthogonality compared to another selection matrix.

        .. math::

            {\bf S_1}^\top \cdot  {\bf S_1}  = 0

        Returns:
            bool: True if both Selector matrices are orthogonal or zero
        Note:
            the check of S_diag shape is kind of useless, but in python everything is python, so
            you never know...
        """
        return self.S_diag.shape == other.S_diag.shape and self.S_diag.T @ other.S_diag == 0.


@dataclass
class ForceControlHandle(ControlSelector):
    """
    Force Control Handle.
    Contains controller gains and selection matrix according to parent class :py:class:`~ControlSelector`

    Similar to the selection matrix, the gain matrix is parsed via the helper function :py:func:`~matrix_parser`.
    In order to keep transfer from ROS messages simple one can directly assign the gain matrices from ``Vector3``
    messages

    .. code-block::
        python

        tmp = ForceControlHandel()
        tmp.K_pos

        array([[0., 0., 0.],
               [0., 0., 0.],
               [0., 0., 0.]])v

    if we now assign a new message, the values will be parsed and directly clipped according to the values in
    `self._K_pos_max`

    .. code-block::
        python

        msg = Vector3()
        msg.y = 1.0
        tmp.K_pos =  msg
        tmp.K_pos

        array([[0., 0., 0.],
              [0., 0.005, 0.],
              [0., 0., 0.]])

    the procedure for ``K_rot`` is identical and thus skipped for brevity.
    The actual control command is finally obtained in the function :py:meth:`~u_F`
    """
    _K_pos_max: float = 5e-3
    _K_rot_max: float = 1e-1

    def __init__(self):
        super(ForceControlHandle, self).__init__(dim=6)
        self.K_pos_diag: np.ndarray = np.zeros(3)
        self.K_rot_diag: np.ndarray = np.zeros(3)

    def reset(self):
        self.S_diag.fill(0.)
        self.K_pos_diag.fill(0.)
        self.K_rot_diag.fill(0.)

    @property
    def K_trans(self):
        return np.diag(self.K_pos_diag)

    @property
    def K_rot(self):
        return np.diag(self.K_rot_diag)

    @K_trans.setter
    def K_trans(self, value):
        matrix_parser(self.K_pos_diag, value, self._K_pos_max)

    @K_rot.setter
    def K_rot(self, value):
        matrix_parser(self.K_rot_diag, value, self._K_rot_max)

    def u_F(self, ɛ_F, R_B_C=np.eye(3)):
        """
        Calculate Force control command for current desired fore and external force measurement.
        The final error is then given as the product of the error and the rotated selection matrix according to
        :py:meth:`~rotate_selection_matrix`

        Args:
            ɛ_F(np.ndarray): error of desired wrench and measured wrench in body frame (6 x 1)
            R_B_C(np.ndarray, optional): rotation from control to body frame. Defaults to unity matrix. (3 x 3)
        """
        S_f, S_m = self.rotate_selection_matrix(R_B_C)
        return S_f @ self.K_trans @ ɛ_F[0:3], S_m @ self.K_rot @ ɛ_F[3:6]

    @property
    def valid(self):
        return not (np.linalg.norm(self.K_trans) == 0.0 and np.linalg.norm(self.S_trans) != 0.0 or
                    np.linalg.norm(self.K_rot) == 0.0 and np.linalg.norm(self.S_rot) != 0.0)


@dataclass
class VelocityControlHandle(ControlSelector):
    """
    Velocity control handle that uses the selection matrix to enable a
    scaled velocity control command
    """

    def __init__(self, v_max, v_rot_max):
        super(VelocityControlHandle, self).__init__(dim=6)
        self._v_des: np.ndarray = np.zeros(6)
        self._v_max = v_max
        self._v_rot_max = v_rot_max
        self.C_vel = False

    def reset(self):
        self.S_diag.fill(0.)
        self._v_des.fill(0.)
        self.C_vel = False

    def u_v(self, R_B_C=np.eye(3)):
        """
        Scale velocity according to maximum allowed velocity magnitude

        Args:
            R_B_C(np.ndarray, optional): rotation from control to body frame. Defaults to unity matrix. (3 x 3)

        Returns:
            np.ndarray, np.ndarray: feed-forward Cartesian steering twist (3 x 1), (3 x 1)
        """
        if self.C_vel:
            return (R_B_C @ self.S_trans @ self.C_v_des,
                    R_B_C @ self.S_rot @ self.C_ω_des)
        else:
            return (R_B_C @ self.S_trans @ R_B_C.T @ self.B_v_des,
                    R_B_C @ self.S_rot @ R_B_C.T @ self.B_ω_des)

    @property
    def B_v_des(self):
        if self.C_vel:
            return np.zeros(3)
        return self._v_des[0:3]

    @property
    def C_v_des(self):
        if self.C_vel:
            return self._v_des[0:3]
        return np.zeros(3)

    @property
    def B_ω_des(self):
        if self.C_vel:
            return np.zeros(3)
        return self._v_des[3:6]

    @property
    def C_ω_des(self):
        if self.C_vel:
            return self._v_des[3:6]
        return np.zeros(3)

    def _set_vel(self, value):
        matrix_parser(self._v_des[0:3], value, val_min=-self._v_max, val_max=self._v_max)

    def _set_rot(self, value):
        matrix_parser(self._v_des[3:6], value, val_min=-self._v_rot_max, val_max=self._v_rot_max)

    @B_v_des.setter
    def B_v_des(self, value):
        self._set_vel(value)
        self.C_vel = False

    @C_v_des.setter
    def C_v_des(self, value):
        self._set_vel(value)
        self.C_vel = True

    @B_ω_des.setter
    def B_ω_des(self, value):
        self._set_rot(value)
        self.C_vel = False

    @C_ω_des.setter
    def C_ω_des(self, value):
        self._set_rot(value)
        self.C_vel = True

    @property
    def valid(self):
        return not (np.linalg.norm(self._v_des) == 0.0 and np.linalg.norm(self.S_trans) != 0.0 or
                    np.linalg.norm(self.B_ω_des) == 0.0 and np.linalg.norm(self.S_rot) != 0.0)
