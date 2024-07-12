#!/usr/bin/pyton3
"""
Unscrewing skill
-------------------------------------


Required ROS-parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^

In hrr-cobot namespace, e.g. /hrr_cobot

==================================== ================== =======================================================================
Parameter                            default value      notes
==================================== ================== =======================================================================
sensitive_grasping_action_srv_name                      action-service name
==================================== ================== =======================================================================

In private namespace of current skill

========================= ================== =======================================================================
Parameter                 default value      notes
========================= ================== =======================================================================
f_contact                 2.0                contact force to detect contact with the environment [N]
hover_distance            0.01               hovering distance above goal-pose [m]
scale_vel                 0.1                scaling of desired velocity for approach (and search if available)
========================= ================== =======================================================================

.. warning::

    the velocity scaling is applied on `cobot.v_max`, so the behavior will change if that value is altered.

.. note::

    to speed up searching: the actual skill is mainly implemented in  :py:meth:`~Unscrew.unscrew`
"""
import copy
import os
from typing import Sequence, Union

import actionlib
import numpy as np
import rospy
import spatialmath as sm
from hr_recycler_msgs.msg import UnscrewAction, UnscrewFeedback, UnscrewResult, ToolType, SkillResult

import hrr_common
import hrr_controllers
from hrr_cobot_robot.manipulation_skills.skill_base import SkillBase, SimpleMp

__all__ = ["Unscrew"]



class Unscrew(SkillBase):

    def __init__(self, f_contact=2.0, hover_distance=2e-2,
                 f_unscrew=10.0, f_insert=20.0,
                 buffer_size=100,
                 closing_width=1e-2):
        self.align_buf = np.zeros((buffer_size, 6))
        super().__init__(name="unscrew_v2")
        self._T_B_E_goal = None
        self.f_contact = f_contact
        self.f_insert = f_insert
        self.f_unscrew = f_unscrew
        self.hover_distance = hover_distance
        self.drop_off_hover_distance = 0.15
        self.B_screw_pos = np.zeros(3)
        self.B_surface_normal = np.r_[0, 0, 1]
        self.B_y_default = np.r_[0, 1, 0]
        self.closing_width = closing_width
        self.scale_pos_vel = 0.1
        self.success = False
        self.num_trials = 5
        self._p0_contact = np.zeros(3)
        self._K_f = 4e-4
        self._f_critical = 45.0
        self._F0 = np.zeros(6)
        self._ref_pose = sm.SE3()
        self._screwdriver_program = 1
        self._shitty_ft = False
        self._unscrew_time = 12.0
        self._screw_height = 8e-3
        self._feedback = UnscrewFeedback()
        self.fixed_pose = sm.SE3()
        self.fixed_pose.A[:4,:4] = [[ 0.81119,  0.00089,  0.58478,  0.58263],
                                   [-0.58245, -0.08804,  0.80809, -0.23876],
                                   [ 0.0522 , -0.99612, -0.0709 ,  0.50576],
                                   [ 0.     ,  0.     ,  0.     ,  1.     ]]

    def init_ros(self, action_srv_name):
        """initialize action-service and start. Refer to :py:meth:`~execute_skill_cb` for insights"""
        self._as = actionlib.SimpleActionServer(action_srv_name, UnscrewAction,
                                                execute_cb=self.execute_skill_cb, auto_start=False)
        self._as.start()

    @classmethod
    def _from_ros(cls, cobot_prefix, skill_prefix="~", cobot=None):
        skill_prefix = hrr_common.fix_prefix(skill_prefix)
        out = cls(f_contact=hrr_common.get_param(f"{skill_prefix}f_contact", 2.0),
                  hover_distance=hrr_common.get_param(f"{skill_prefix}hover_distance", 1e-2)
                  )
        out.init_skill_base(cobot_prefix=cobot_prefix, cobot=cobot)
        out.init_ros(hrr_common.get_param(f"{cobot_prefix}unscrew_action_srv_name"))
        out.scale_pos_vel = hrr_common.get_param(f"{skill_prefix}scale_vel", out.scale_pos_vel)
        out.observer.set_buffers(out.cobot)
        return out

    def recovery(self) -> None:
        """Recover robot from an error, i.e. drive back to pre-pose via
        the :py:meth:`hrr_cobot_robot.hrr_cobot_control.HrrCobotControl.goTo` command.

        .. note::

            this function will increase the `F_max` wrench threshold to allow movement by 10%.
            This may be insufficient in some cases, and will require a manual reset then.
            Similarly, the threshold is reset at the end, so in case the function is interrupted,
            the threshold might be false afterwards.
        """
        F_max_prev = self.cobot.F_max
        if np.linalg.norm(self.cobot.B_F_msr) > self.cobot.F_max:
            self.cobot.F_max = max(1.1 * np.linalg.norm(self.cobot.B_F_msr), F_max_prev)
        if isinstance(self._T_B_E_goal, sm.SE3):
            self.cobot.goTo(sm.SE3(self.hover_distance * self.B_surface_normal) @ self._T_B_E_goal)
        self.cobot.F_max = F_max_prev

    def step(self, contact_as_success=False) -> Union[bool, None]:
        """
        A default update step, via :py:meth:`hrr_cobot_robot.hrr_cobot_control.HrrCobotControl.update`, i.e.
        the update the current control-command according to the active control-law and update internal cobot states.

        Args:
            contact_as_success(bool, optional): flag to evaluate surface_contact as a success-event. Defaults to False.

        Returns:
            Union[bool, None]: None if no event has been detected. True, if success, False if an error occurs.
        """
        self.cobot.update()
        self.observer.update(self.cobot)
        if contact_as_success and self.surface_contact:
            return True
        if not self.action_sever_valid:
            return False

    def progress(self, contact_as_success=False, T_stop=np.inf, success_func=None) -> bool:
        """
        detachable version of :py:meth:`~step`, i.e. this function is run
        until an error or success-state of a curren manipulation-primitive is encountered.

        Args:
            contact_as_success(bool, optional): flag to evaluate surface_contact as a success-event. Defaults to False.
            T_stop(float, optional): timeout to set break in update-loop. Defaults to None
            success_func(callable or None) optional success funciton
        Returns:
            bool: True if manipulation-primitive has been executed successfully, False otherwise.
        """
        if self.failure:
            return False
        t0 = rospy.get_time()
        while self.cobot.state is not None:
            fb = self.step(contact_as_success=contact_as_success)
            if fb is not None:
                return fb
            if rospy.get_time() - t0 >= T_stop:
                break
            if success_func is not None:
                if success_func():
                    return True
        return self.cobot.safety_check()

    def process_steps(self, steps: Sequence[SimpleMp]) -> None:
        """
        Process a sequence of :py:class:`~SimpleMP` by iteratively

        #. check for failure and exit if necessary
        #. update action-service feedback (c.f. :py:meth:`~update_feedback_msg`)
        #. if the current :py:class:`~SimpleMP` has the `wrench_bias_hack` set, do as commanded
        #. initialize mp according to `f_init` and the provided function-arguments
        #. call :py:meth:`~progress` to process current mp
        #. if necessary, cancel execution
        #. set robot to stop before initiating a new MP

        Args:
            steps(Sequence[SimpleMp]): list of MPs to process.
        """
        for mp in steps:
            if self.failure:
                return
            self.update_feedback_msg(msg=f"Initiating {mp.name}")
            if mp.wrench_bias_hack:
                self.cobot.FT.reset_bias()
                self.cobot.update_tf()
                self._F0 = self.cobot.B_F_msr.copy()
            mp.f_init(*mp.args, **mp.kwargs)
            if not self.progress(mp.contact_as_success, mp.T_max, success_func=mp.success_func):
                self.cancel(msg=f'failed during {mp.name}')
            self.cobot.stop()

    @property
    def surface_contact(self) -> bool:
        r"""evaluate if current F/T-measurement represents an environment contact, i.e.

        .. math::

            \left({\bf F}_{\mathrm{msr}} - {\bf F}_{\mathrm{bias-hack}, 0} - {\boldsymbol{\sigma}}_{\mathrm{noise}}\right)^{\top} {}^{B}{\bf n}

        Returns:
             bool: True, if the above is larger than `f_contact` (c.f. table above)
        """
        return ((self.cobot.B_F_msr[:3] - self._F0[0:3]) -
                self.cobot.FT.noise[:3]).T @ self.B_surface_normal >= self.f_contact

    def screw_remove_success(self) -> bool:
        self.success = True
        return True
        # return ((self.cobot.T_E_C_robot.t - self._p0_contact).T @ self.B_surface_normal).sum() >= self._screw_height

    def unscrew(self, B_screw_pos=None, B_normal=None, T_B_E_goal=None):
        """
        Actual unscrewing task, that consists of the main sub-steps
         (for convenience, each sub-step is implemented in a separate function):

        #. approach_object, i.e. approach the surface in a force-sensitive manner. This consists of the MPs:

            * ``approach_pre_pose`` that uses `self.cobot.move_to_pose` to initiate a Cartesian servoing control
            * ``approach_surface`` that uses `self.cobot.set_py_hybrid_force_vel_command` to initiate a velocity control-motion along the negative surface-normal vector. the set-values are adjusted in ``approach_kwargs``

        #. ``search`` - currently removed / not implemented
        #. ``insert screw`` that uses set_py_hybrid_force_vel_command according to the set-values in `insert_kwargs`
        #. ``remove_screw`` that uses set_py_hybrid_force_vel_command according to the set-values in `unscrew_kwargs`

        This process is repeated for at most `self.num_trials` or until the skill is run successfully or a failure occurs.

        Args:
            B_screw_pos(np.ndarray, optional): screw-location in base-frame
            B_normal(np.ndarray, optional): surface-normal in base-frame
            T_B_E_goal(sm.SE3, optional): EE-goal pose in screw-head

        .. note::

            The arguments may seem confusing, but it allows to either provide the goal-pos/normals or just the
            ee-pose from external code, to diminish the issue of internal EE->tip transformations
        """

        def approach_object():
            self.process_steps((
                SimpleMp(name="approach_pre_pose", f_init=self.cobot.move_to_pose,
                         args=(sm.SE3(self.hover_distance * self.B_surface_normal) @ T_B_E_goal)),
                SimpleMp(name="approach_surface", f_init=self.cobot.set_py_hybrid_force_vel_command,
                         kwargs=approach_kwargs,
                         contact_as_success=True)
            ))

        def insert_screw():
            self.process_steps((
                SimpleMp(name="insert", f_init=self.cobot.set_py_hybrid_force_vel_command, kwargs=insert_kwargs,
                         T_max=2.0),
            ))

        def remove_screw():
            # force_z0 = abs((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[2])
            # print(force_z0)
            # self.cobot.run_screwdriver_program(self._screwdriver_program, run_time=5)
            # rospy.sleep(6)
            # print(abs((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[2]))
            # self.success = abs((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[2]) > force_z0
            self.cobot.run_screwdriver_program(self._screwdriver_program, run_time=self._unscrew_time)
            #self.process_steps((
                #SimpleMp(name="unscrew", f_init=self.cobot.set_py_hybrid_force_vel_command, kwargs=unscrew_kwargs,
                      #   T_max=self._unscrew_time, success_func=self.screw_remove_success),
           # ))

            rospy.sleep(5)
            #self.cobot.goTo(sm.SE3(self.B_surface_normal * 0.01) @ self.cobot.T_B_E_robot, v_max=0.001)
            self.success = True
            #self.failure = False

        def search():
            T_B_E_goali = self._T_B_E_goal
            success = False
            search_vec = [sm.SE3(np.r_[0, 0, 0]), sm.SE3(np.r_[-0.002, 0, 0]),
                          sm.SE3(np.r_[0.002, 0, 0]),
                          sm.SE3(np.r_[0, 0.002, 0]), sm.SE3(np.r_[0, -0.002, 0])]
            for i in range(len(search_vec)):

                T_B_E_goali = search_vec[i] @ T_B_E_goali
                self.process_steps((
                    SimpleMp(name="approach_pre_pose", f_init=self.cobot.move_to_pose,
                             args=(sm.SE3(self.hover_distance * self.B_surface_normal) @ T_B_E_goali)),
                    SimpleMp(name="approach_surface", f_init=self.cobot.set_py_hybrid_force_vel_command,
                             kwargs=approach_kwargs,
                             contact_as_success=True))
                    )

                if not success and self.action_sever_valid:
                    self._p0_contact = np.copy(self.cobot.T_E_C_robot.t)
                    forces_x = []
                    forces_y = []
                    self.cobot.run_screwdriver_program(self._screwdriver_program, run_time=0.5)
                    # dist = np.array([0, 0, 0.002])
                    # self.cobot.move_to_pose((sm.SE3(dist) * self.B_surface_normal) @ T_B_E_goali)
                    self.cobot.init_sns_vel()
                    v_test = np.zeros(6)
                    for t in range(200):
                        v_test[1] = 3e-3 * np.sin(t / 100.0 * 2 * np.pi)
                        self.cobot.update(u_cmd=v_test, u_cmd_frame=self.cobot.ctrl_frame)
                        forces_x.append((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[0])
                        forces_y.append((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[1])
                    for t in range(200):
                        v_test[0] = 3e-3 * np.sin(t / 100.0 * 2 * np.pi)
                        self.cobot.update(u_cmd=v_test, u_cmd_frame=self.cobot.ctrl_frame)
                        forces_x.append((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[0])
                        forces_y.append((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[1])
                    self.cobot.stop()
                    print(np.std(forces_x), np.std(forces_y))
                    if np.std(forces_x) > 4 and np.std(forces_y) > 4:
                        success = True
                        screw_pos = self.cobot.T_B_E_robot
                        break
                    else:
                        success = False
                        screw_pos = self.cobot.T_B_E_robot
                else:
                    self.cancel(msg="Pre-empted")
            return success, screw_pos

        if B_screw_pos is not None:
            self.B_screw_pos = B_screw_pos
        if B_normal is not None:
            self.B_surface_normal = B_normal
        if T_B_E_goal is None:
            T_B_E_goal = sm.SE3()
            T_B_E_goal.A[:4, :4] = self.fixed_pose.A.copy()
            tooltip_offset_A = (B_screw_pos) - (
                        self.fixed_pose @ self.cobot.T_E_C_robot).t  # so bzw siehe notebook
            [T_B_E_goal.t[0], T_B_E_goal.t[1], T_B_E_goal.t[2]] = T_B_E_goal.t + tooltip_offset_A


          #
          # T_B_E_goal = self.cobot.get_valid_ee_pose(self.B_screw_pos,
          #                                             B_normal=self.B_surface_normal,
          #                                             B_y_test=self.B_y_default)
        if T_B_E_goal is None:
            return self.cancel(
                msg=f"screw at {self.B_screw_pos} with normal-vector {self.B_surface_normal} is unreachable")
        # set skill parameters
        self._T_B_E_goal = T_B_E_goal
        self.cobot.init_sns_vel()

        num_trials = self.num_trials
        approach_kwargs = self.cobot.default_hybrid_kwargs()
        approach_kwargs["scale_pos"] = self.scale_pos_vel
        approach_kwargs["vel_dir"] = np.r_[-self.B_surface_normal, np.zeros(3)]
        move_up_kwargs = copy.deepcopy(approach_kwargs)
        move_up_kwargs["vel_dir"] = np.r_[self.B_surface_normal, np.zeros(3)]
        unscrew_kwargs = copy.deepcopy(approach_kwargs)
        unscrew_kwargs["vel_dir"] = np.zeros(6)
        unscrew_kwargs["K_f"] = self._K_f * np.ones(3)
        unscrew_kwargs["wrench_dir"] = np.r_[1.0, 1.0, 1.0, np.zeros(3)]
        surface_kwargs = copy.deepcopy(unscrew_kwargs)
        insert_kwargs = copy.deepcopy(unscrew_kwargs)
        insert_kwargs["B_F_des"] = np.r_[-self.B_surface_normal * self.f_insert, np.zeros(3)]
        surface_kwargs["B_F_des"] = np.r_[-self.B_surface_normal * self.f_unscrew, np.zeros(3)]
        # unscrew_kwargs["B_F_des"] = np.r_[self.B_surface_normal * self.f_unscrew, np.zeros(3)]
        unscrew_kwargs["B_F_des"] = np.zeros(6)
        unscrew_kwargs["scale_pos"] = 0.05
        unscrew_kwargs["vel_dir"] = np.r_[self.B_surface_normal * self.cobot.v_max, np.zeros(3)]

        #self.cobot.goTo(sm.SE3([0, 0, 0.1]) @ self.cobot.T_B_E_robot, v_max=0.01, check_reachable=False)
        #self.cobot.goTo(sm.SE3([0, 0, -0.1]) @ self.cobot.T_B_E_robot, v_max=0.01, check_reachable=False)
        sequential_mps = {'approach': approach_object,
                          'search': search,
                          'insert screw': insert_screw,
                          'remove_screw': remove_screw}

        screw_found, screw_position = search()
        if self.failure:
            rospy.logerr("failed to execute search")
            return self.cancel(
                msg="failed to execute search")
        if screw_found:
            self._T_B_E_goal = screw_position
            remove_screw()
        else:
            rospy.logerr("failed to find screw at and around the given position")
            return self.cancel(
                msg="failed to find screw at and around the given position")

        # if self.failure:
        #     rospy.logerr("failed to execute screw removal")
        #     return self.cancel(
        #         msg="failed to execute screw removal")

        # while num_trials > 0:
        #     if self.failure:
        #         rospy.logerr(f"failed to after {num_trials} runs")
        #         return
        #
        #     # execute
        #
        #
        #
        #     for name, func in sequential_mps.items():
        #         self.update_feedback_msg(msg=f"Execute {name}")
        #         func()
        #         if self.failure:
        #             rospy.logerr(f"failed to execute {name}")
        #             return

        #if self.success:
        self.cobot.goTo(sm.SE3(self.B_surface_normal * self.hover_distance) @ self.cobot.T_B_E_robot)

        #else:
            #self.recovery()
            # num_trials += 1

    def pre_skill_execution(self, tool_id=None, hardcoded_transformation=False) -> bool:
        """This function is run before the skill is executed. It consists of

        #. checking against existing environment flags, e.g. ``USABLE_FT_DATA`` to symbolize that the F/T sensor is not utter shit
        #. call parent-class :py:meth:`~hrr_cobot_robot.manipulation_sills.skill_base.SkillBase.pre_skill_execution`
        #. set robot flags as needed
        #. cancel execution if unexpected high payload is detected.

        Args:
            tool_id(int or None, optional): desired tool-id. Defaults to None.
            hardcoded_transformation:

        Returns:
            bool: True, if skill can be executed.

        .. note::

            the `HRR_SHAFTGRINDER_HACK` has been introduced to test this skill with the shaftgrinder thing, while the
            real screwdriver was / is not available.
            This can be set in a terminal via

            .. code-block:: bash

                export HRR_SHAFTGRINDER_HACK="yes"

            or in a programmatic manner

            .. code-block:: python

                import os
                os.environ["HRR_SHAFTGRINDER_HACK"] = 'because_fuck_you'
        """
        import os
        tool_id = ToolType.SCREW_DRIVER
        try:
            if len(os.environ["HRR_SHAFTGRINDER_HACK"]) > 1:
                tool_id = ToolType.SHAFT_GRINDER
        except KeyError:
            pass
        try:
            if os.environ['USABLE_FT_DATA'] == '1':
                self._shitty_ft = False
            else:
                raise KeyError()
        except KeyError:
            self._shitty_ft = True
        out = super().pre_skill_execution(tool_id=tool_id, hardcoded_transformation=False)
        if self.cobot.tool_id != tool_id:
            self.cancel(save_data=False,
                        msg=f"current tool is set to {self.cobot.tool}. Excepted screwdriver!")
            return False
        if self._shitty_ft:
            self.cobot.FT.reset_bias()

        if np.linalg.norm(self.cobot.B_F_msr) > self._f_critical:
            self.cancel(save_data=False,
                        msg=f"Force threshold violated "
                            f"{np.linalg.norm(self.cobot.B_F_msr):.3f} >= {self._f_critical:.3f}. Recalibrate!")
            return False
        self.cobot.compensate_joint_limits = False
        return out

    def execute_skill_cb(self, goal):
        """
        Action-service callback.
        Runs :py:meth:`~pre_skill_execution` first to check, if robot is in a valid state.

        Checks for positive data for the following attributes, that fall back to the internal values otherwise:

        * ``contact_force``
        * ``insertion_force``
        * ``timeout``

        then runs :py:meth:`~unsrew` via transforming the `screw_location` and `surface_normal`
        into the base-frame of the robot as `B_screw_pos` and `B_normal`.

        Args:
            goal(AdaptiveGraspingGoal): action-service goal
        """

        def check_positive(goal_arg, self_arg):
            g_val = getattr(goal, goal_arg)
            if g_val > 0.0:
                setattr(self, self_arg, g_val)
            else:
                rospy.logwarn(self._log_str(f"received none-positive goal-argument {goal_arg}:={g_val}. "
                                            f"Use default {getattr(self, self_arg)}"))

        if not self.pre_skill_execution():
            return
        check_positive("contact_force", "f_contact")
        check_positive("insertion_force", "f_insert")
        check_positive("timeout", "timeout")
        self.unscrew(
            B_screw_pos=self._tf.A_vector_from_msg(goal.screw_location, frame_A=self.cobot.base_link),
            B_normal=self._tf.A_vector_from_msg(goal.surface_normal, frame_A=self.cobot.base_link))
        if self.success:
            self._skill_result.result = SkillResult.FINISHED
            self.end_skill("Unscrewing done!")
        else:
            self.cancel("Unscrewing failed due to unknown reasons")

    @staticmethod
    def _log_str(msg):
        return f"Unscrewing -> {msg}"

    def update_feedback_msg(self, msg: str) -> None:
        """
        update current action-service feedback message / publisher.
        collects current cobot state via
        :py:meth:`hrr_cobot_robot.hrr_cobot_handle.HrrCobotIf.cobot_state_msg`

        then publishes feedback via :py:meth:`~publish_feedback`

        Args:
            msg(str): message to be published via state-feedback
        """
        rospy.loginfo(self._log_str(msg))
        cur_state = self._feedback.state.state
        self._feedback.state = self.cobot.cobot_state_msg(msg, None)
        self._feedback.state.state = cur_state
        self.publish_feedback()

    def publish_feedback(self) -> None:
        """publish feedback if possible, i.e. the current instance has an action-service running"""
        try:
            self._as.publish_feedback(self._feedback)
        except AttributeError:
            rospy.logerr(f"failed to send feedback-message")

    @property
    def result(self) -> UnscrewResult:
        """Generate UnscrewResult from code,

        Returns:
            UnscrewResult: action-service result
        """
        res = UnscrewResult()
        self._update_skill_result()
        res.skill_result = self._skill_result
        return res


# def debug_node():
#     from hrr_cobot_robot.hrr_cobot_control import HrrCobotControl
#     rospy.loginfo(f"starting standalone grasping skill")
#     cobot = HrrCobotControl.from_ros(cobot_prefix="/hrr_cobot", compile_numba=False)
#     unscrewing = Unscrew.from_ros(cobot=cobot, cobot_prefix="/hrr_cobot",
#                                   skill_prefix="/hrr_cobot/unscrewing")
#     unscrewing.pre_skill_execution()
#     unscrewing.unscrew(np.r_[0.66, 0.0, 0.10])
#     rospy.spin()
#
#
# if __name__ == "__main__":
#     hrr_common.set_ros_environment("hrrcobotLinux54")
#     os.environ["HRR_SHAFTGRINDER_HACK"] = 'because_fuck_you'
#     os.environ["HRR_QUICK_HACK"] = '1'
#     rospy.init_node("debug_adaptive_grasping_node")
#     debug_node()
