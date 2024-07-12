"""
rotation handlers and helpers
--------------------------------
"""
import numpy as np
import warnings


try:
    __all__ = ["homog2mat", "homog2pos", "posquat2homog", "homog2quat",
               "quaternion_distance", "quaternion_error", "quat2human", "quaternion_rotate",
               "quaternion_zeroOrder_integrate", "integrate_quaternion_difference",
               "pose_error", "cartesian_servo_set_value", "clip_velocity",
               "calc_goal_pose", "calc_EE_pre_pose", "calc_EE_goal_pose"
               ]
    import spatialmath as sm
    import quaternion

    def _assert_quat(q):
        if isinstance(q,  np.quaternion):
            return q
        return np.quaternion(*q)

    def _assertSE3(T):
        if isinstance(T, sm.SE3):
            return T
        T_SE3 = sm.SE3(T[:3, 3])
        T_SE3.A[:3, :3] = T[:3, :3]
        return T_SE3

    def homog2pos(T):
        """
        Get position from homogeneous transformation matrix

        Args:
            T(ar_type): homogeneous coordinate matrix

        Returns:
            p(ar_type): position vector (3x1)
        """
        return _assertSE3(T).t

    def homog2mat(T):
        """
        Get rotation matrix from homogeneous transformation matrix

        Args:
            T(ar_type): homogeneous coordinate matrix

        Returns:
            p(ar_type): position vector (3x1)
        """
        return _assertSE3(T).A[:3, :3]

    def homog2quat(T):
        """
        Get quaternion from homogeneous transformation matrix

        Args:
            T(np.ndarray or sm.SE3): homogeneous coordinate matrix

        Returns:
            p(ar_type): position vector (3x1)
        """
        return quaternion.from_rotation_matrix(homog2mat(T))

    def posquat2homog(p, q):
        """
        Position vector and quaternion to homogeneous transformation matrix

        Args:
            p(ar_type): position vector
            q(np.quaternion): quaternion

        Returns:
            sm.SE3
        """
        q = _assert_quat(q)
        T = sm.SE3(p)
        T.A[:3, :3] = quaternion.as_rotation_matrix(q)
        return T

    def _np_inverse_homog(T):
        T_ = np.eye(4)
        T_[:3, :3] = T[:3, :3].T
        T_[:3, 3] = np.dot(-T[:3, :3].T, T[:3, 3])
        return T

    def quaternion_error(q_des, q_is):
        """
        Calculate quaternion error

        Args:
            q_des(np.quaternion):
            q_is(np.quaternion):
        """
        return _assert_quat(q_is).conjugate() * _assert_quat(q_des)

    def quaternion_distance(q1, q2):
        """
        Calculate distance between 2 quaternions

        Args:
            q1:
            q2:

        Returns:
            torch.Tensor: multiplied quaternion
        """
        q_tmp = quaternion_error(q1, q2)
        return np.arccos(q_tmp.w) * 2

    def quaternion_rotate(q_A_B, B_vec) -> np.ndarray:
        """
        Helper function to run quaternion rotation to transform vector in frame :math:`B` to :math:`A`

        Args:
            q_A_B(np.quaternion): quaternion to rotate from B to A
            B_vec(np.ndarray): vector in frame :math:`B`

        Returns:
            np.ndarray: math:`{}^{A}{\bf x}`
        """
        return (q_A_B * np.quaternion(0.0, *B_vec) * q_A_B.conjugate()).vec

    def integrate_quaternion_difference(ω, dt):
        norm = np.linalg.norm(ω)
        if norm > 1e-12:
            w = np.cos(norm * dt * 0.5)
            vec = ω / norm * np.sin(norm * dt * 0.5)
            return np.quaternion(w, *vec)
        return np.quaternion(1.0, 0.0, 0.0, 0.0)

    def quaternion_zeroOrder_integrate(q_cur, ω, dt):
        return _assert_quat(q_cur) * integrate_quaternion_difference(ω, dt)

    def quat2human(q):
        angles = np.rad2deg(quaternion.as_euler_angles(_assert_quat(q)))
        return f"RPY: α: {angles[0]:.2f}°,\tβ: {angles[1]:.2f}°\tɣ:{angles[2]:.2f}°"

    def pose_error(T_des, T_is, clamp_pos=np.inf, clamp_rot=np.inf):
        """
        Get pose error as a 6-dim vector of.
        E.g. getting the pose difference in end effector frame e given desired and current pose
        of the end effector in the base frame, this results in

        .. math::

            {}^{E}T_{ep} := ({}^{W} T_{e} )^{-1} {}^{W} T_{ep}

        where :math:`{}^{W} T_{ep}` is ``T_des`` and :math:`{}^{W} T_{e}` is ``T_is``

        Args:
            T_des(sm.SE3):  desired pose to be reached
            T_is(sm.SE3):   current pose
            clamp_pos(float): clamp position error by this value. Defaults to inf
            clamp_rot(float): clamp position error by this value. Defaults to inf

        Returns:
            np.ndarray: (scaled) error vector for given poses

        """
        T_e_err = _assertSE3(T_is).inv() * _assertSE3(T_des)
        return np.r_[np.clip(T_e_err.t, -clamp_pos, clamp_pos),
                     np.clip(T_e_err.rpy("rad"), -clamp_rot, clamp_rot)]

    def clip_velocity(v_cmd, err, dt, v_max):
        """
        Clip commanded velocity.

        - max velocity given as :math:`|e| / dt`
        - additionally allow `v_max` to define user velocity constraint

        Args:
            v_cmd(np.ndarray): velocity from e.g. servo gain controller (6x1)
            err(np.ndarray): Cartesian error-vector  (6x1)
            dt(float): time increment
            v_max(float): maximum velocity

        Returns:
            np.ndarray: constrained Cartesian velocity
        """
        cmd_err = np.minimum(np.abs(err / dt), np.ones(6) * v_max)
        return np.clip(v_cmd, -cmd_err, cmd_err)

    def cartesian_servo_set_value(T_des, T_is, gain=10.0, dt=0.0, v_max=np.inf):
        """
        Simple Cartesian Servo Gain Controller with optional clipping.

        Args:
            T_is (sm.SE3): current pose
            T_des (sm.SE3): desired pose
            gain (float, optional): [description]. Defaults to 10.0.
            dt (float, optional): time increment. Defaults to 0.0.
            v_max (float, optional): maximum velocity. Defaults to np.inf.

        Returns:
            tuple[np.ndarray, float]: Cartesian velocity and error
        """
        err = pose_error(T_des, T_is)
        K_p = np.eye(6) * gain
        if dt > 0.0:
            return clip_velocity(K_p @ err, err, dt, v_max), err
        elif 0 < v_max < np.inf:
            return np.clip(K_p @ err, -v_max, v_max), err
        return K_p @ err, err

    def calc_goal_pose(normal, p_location, y_axis=np.r_[0., 1., 0.]):
        """
        Calculate Goal Pose given the normal vector and the location of the object center
        and optionally the :math:`y`-axis of the goal-pose.

        The rotation of the goal-pose is then given by the flipped normal-vector as the :math:`z`-axis.
        Using the optional :math:`y`-axis a right handed coordinate system is constructed which is
        transformed into a SO(3) rotation matrix.
        Please refer to the
        `spatial math python tool-box <https://petercorke.github.io/spatialmath-python/func_3d.html?highlight=oa2r#spatialmath.base.transforms3d.oa2r>`_
        for further information.

        Using the location as the desired translation component, the full SE(3)-pose is returned.

        Args:
            normal (np.ndarray): surface normal (3 x 1)
            p_location (np.ndarray): goal point (3 x 1)
            y_axis (np.ndarray, optional): :math:`y`-axis of the goal pose. Defaults to np.r_[0., 1., 0.].

        Returns:
            sm.SE3: goal-pose
        """
        T_des = sm.SE3(p_location)
        T_des.A[:3, :3] = sm.base.oa2r(y_axis, -normal)
        return T_des

    def calc_EE_goal_pose(B_normal, B_p_location, T_C_E=None, B_y_axis=np.r_[0., 1., 0.], safety_distance=0.05):
        r"""
        extend :py:func:`calc_goal_pose` by transforming the
        resulting goal-pose, which is supposed to be given in `base-frame`
        coordinates, into the dedicated EE-pose that the robot should steer to
        reach the goal-pose with the tip / control-frame :math:`C`.

        .. math::

            {}^{B}{\bf T}_{E} = {}^{B}{\bf T}_{C} {}^{C}{\bf T}_{E}

        Args:
            B_normal (np.ndarray): normal vector in base frame :math:`{}^{B}{\bf n} \in \mathbb{R}^{3}`
            B_p_location(np.ndarray): goal-position in base-frame :math:`{}^{B}{\bf p}_{BC} \in \mathbb{R}^{3}`.
            B_y_axis (np.ndarray, optional): reference y-axis to generate pose / right-hand CS in base-frame. Defaults to :math:`{}^{B}{\bf e}_{y}`.
            T_C_E(sm.SE3 or None, optional): homogeneous transformation from end-effector to tool-frame. Defaults to None.

        Returns:
            sm.SE3: pre-pose of end-effector wrt to base-frame.

        Raises:
            AssertionError: if safety_distance is negative
        """
        T_B_C_goal = calc_goal_pose(
            normal=B_normal, p_location=B_p_location, y_axis=B_y_axis)
        if T_C_E is None:
            return T_B_C_goal
        return T_B_C_goal @ T_C_E

    def calc_EE_pre_pose(B_normal, B_p_location, T_C_E=None, B_y_axis=np.r_[0., 1., 0.], safety_distance=5e-2):
        r"""Calculate EE-pre pose via hovering the robot above the surface

        The surface is defined by its normal, while the ``safety_distance``
        describes the hovering height that the robot should be steered to

        Args:
            B_normal (np.ndarray): normal vector in base frame :math:`{}^{B}{\bf n} \in \mathbb{R}^{3}`
            B_p_location(np.ndarray): goal-position in base-frame :math:`{}^{B}{\bf p}_{BC} \in \mathbb{R}^{3}`.
            B_y_axis (np.ndarray, optional): reference y-axis to generate pose / right-hand CS in base-frame. Defaults to :math:`{}^{B}{\bf e}_{y}`.
            T_C_E(sm.SE3 or None, optional): homogeneous transformation from end-effector to tool-frame. Defaults to None.
            safety_distance (float, optional): hovering distance above surface. Defaults to 0.05.

        Returns:
            sm.SE3: pre-pose of end-effector wrt to base-frame.

        Raises:
            AssertionError: if safety_distance is negative
        """
        assert safety_distance >= 0.0, "hovering distance is negative"
        return sm.SE3(B_normal * safety_distance) @ calc_EE_goal_pose(B_normal=B_normal, B_p_location=B_p_location, B_y_axis=B_y_axis, T_C_E=T_C_E)


except ModuleNotFoundError:
    __all__ = []
    warnings.warn(
        "This module expects numpy-quaternion for quaternion calculation. and python-spatialmath for SE3 poses" +
        "\n\t conda install -c conda-forge quaternion\n\t python -m pip install numpy-quaternion"
        "\n\t python -m pip install spatialmath-python"
    )
