#!/usr/bin/pyton3
"""
Sensitive grasping skill
-------------------------------------

Required ROS-parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^

In hrr-cobot namespace, e.g. /hrr_cobot

==================================== ================== =======================================================================
Parameter                            default value      notes
==================================== ================== =======================================================================
gripper_ns                                              namespace of gripper, e.g. /hrr_cobot/gripper
sensitive_grasping_action_srv_name                      action-service name
==================================== ================== =======================================================================

In private namespace of current skill

==================================== ==================== =======================================================================
Parameter                            default value        notes
==================================== ==================== =======================================================================
closing_speed                        0.01                 gripper closing speed [m/s]
closing_width                        0.02                 gripper closing width [m]
dsa_link_name                        hrr_cobot.wsg_50_dsa link name of WSG-50 DSA tool-center-point link name
drop_off_hover_distance              0.15                 hovering distance above drop-off-goal-pose [m]
f_contact                            2.0                  contact force to detect contact with the environment [N]
f_lift                               15.0                 force to remove lightbulb
grasping_timeout                     4.0                  grasping timeout [s]
grasping_strategies                  [-1,-1,-1,0,0,1]     grasping control strategies, where -1 means force-control and 1 velocity-control
hover_distance                       0.01                 hovering distance above goal-pose [m]
opening_width                        0.03                 gripper opening width [m]
tcp_link_name                        hrr_cobot.wsg_50_tcp link name of WSG-50 tool-center-point name
rotation_steps                       75                   steps for tilting motion in one direction
rotation_velocity                    0.05                 rotation velocity during tilting (**in TCP**)
scale_vel                            0.1                  scaling of desired velocity for approach (and search if available)
==================================== ==================== =======================================================================

.. warning::

    the velocity scaling is applied on `cobot.v_max`, so the behavior will change if that value is altered.

.. note::

    to speed up searching: the actual skill is mainly implemented in

    * :py:meth:`~SensitiveGrasping.remove_lightbulb`
    * :py:meth:`~SensitiveGrasping.grasp`

"""
import copy
from typing import Optional, Sequence

import actionlib
import numpy as np
import quaternion
import rospy
import spatialmath as sm
from hr_recycler_msgs.msg import AdaptiveGraspingAction, AdaptiveGraspingGoal, \
    AdaptiveGraspingFeedback, AdaptiveGraspingResult, ToolType

import hrr_common
from hrr_cobot_robot.manipulation_skills.skill_base import SkillBase, SimpleMp, get_SE3_from_pose_stamped
from wsg_50_hw import ApplyCompliantGrasping

__all__ = ["SensitiveGrasping"]


def attitude_aligned(alignment_buffer, N=50, pos_threshold=0.1, rot_threshold=1e-2) -> bool:
    if np.linalg.norm(alignment_buffer[:N, 0:2]) < pos_threshold:
        return (sum(np.linalg.norm(alignment_buffer[:N, 3:], axis=1) != 0) == N and
                np.mean(alignment_buffer[:N, 5], axis=0) <= rot_threshold) \
               or np.linalg.norm(alignment_buffer[:N, 3:]) == 0.0
    return False


class SensitiveGrasping(SkillBase):

    def __init__(self, f_contact=1.0, hover_distance=0.15,
                 buffer_size=100,
                 closing_width=1e-2):
        self.align_buf = np.zeros((buffer_size, 6))
        self.grasping_controller = None  # type: Optional[ApplyCompliantGrasping]
        super().__init__(name="sensitive_grasping")
        self._T_B_E_goal = None
        self.f_contact = f_contact
        self.hover_distance = hover_distance
        self.drop_off_hover_distance = 0.15
        self.B_object_des = sm.SE3()
        self.closing_width = closing_width
        self.scale_pos_vel = 0.1
        self.closing_speed = 0.005
        self.max_tilt = np.deg2rad(25)
        self.tcp_link = "hrr_cobot.wsg_50_tcp"
        self.dsa_link = "hrr_cobot.wsg_50_dsa"
        self.success = False
        self._rotation_steps = 75
        self._rotation_vel = 0.06
        self._open_width = 0.08
        self._tilting_angle = np.pi / 2.0
        self._lifting_force = 25.0
        self._grasp_timeout = 20
        self._K_f = 4e-4
        self._F0 = np.zeros(6)
        self._ref_pose = sm.SE3()
        self._feedback = AdaptiveGraspingFeedback()
        self.easy_version = True #Easy version of the skill

    @property
    def B_surface_normal(self):
        """Surface normal taken from pose. Expected to be the last column-vector of the rotation matrix"""
        #return self.B_object_des.A[0:3, 2]
        return np.r_[0,0,1]
        
    @property
    def B_tcp_y_axis(self):        # if new_tool_str == "wsg_50_dsa":
        #     rospy.loginfo("Trying to home wsg 50 dsa again!")
        #     # cobot.change_tool("nothing")
        #     # rospy.sleep(3)
        #     # cobot.change_tool("wsg_50_dsa")
        #     rospy.sleep(3)
        #     cobot.gripper.reset()
        #     rospy.sleep(0.5)
        #     cobot.gripper.send_pos(0.07, si=True)
        """Surface normal taken from pose. Expected to be the middle column-vector of the rotation matrix"""
        #return self.B_object_des.A[0:3, 1]
        return np.r_[0,1,0]
    
    @property
    def strategies(self) -> np.ndarray:
        r"""current grasping strategies as the diagonal vector in :math:`\mathbb{R}^{6}`
        Used as a property to allow type-checking in the setter, as it must hold

        .. math::

            \mathtt{diag}({\bf S}_F)^\top \mathtt{diag}({\bf S}_v) = 0

        Eventually, this is forwarded to
        :py:meth:`wsg_50_hw.ApplyCompliantGrasping.set_strategies` via
        two Boolean arrays for force- and velocity selections.
        """
        return self.grasping_controller.strategies

    @strategies.setter
    def strategies(self, value: np.ndarray):
        value = np.array(value)
        S_F = value == self.grasping_controller.F_CTRL
        S_v = value == self.grasping_controller.V_CTRL
        if S_F.T @ S_v:
            rospy.logwarn("cannot enable two strategies along an axis. Disable axes")
            idx = np.where(S_F * S_v)
            S_F[idx] = False
            S_v[idx] = False
        self.grasping_controller.set_strategies(S_F=S_F, S_v=S_v)

    @property
    def closing_width(self):
        """
        Using property to apply value-checking when setting a new value.
        The closing width of the gripper is limited to minimum [7mm, 110mm] as
        the DSA-gripper fingers are approximately 3.5mm thick and 110mm is the stroke length.

        Returns:
            float: gripper closing width in m

        """
        return self._closing_width

    @closing_width.setter
    def closing_width(self, value):
        self._closing_width = min(110e-3, max(value, 7e-3))

    def init_ros(self, action_srv_name,
                 gripper_ns):
        """initialize grasping controller and simple-action server for gripper command,
        that will execute :py:meth:`~execute_skill_cb` when called.
        """
        self.grasping_controller = ApplyCompliantGrasping.from_ros(gripper_ns)
        self._as = actionlib.SimpleActionServer(action_srv_name, AdaptiveGraspingAction,
                                                execute_cb=self.execute_skill_cb, auto_start=False)
        self._as.start()

    @classmethod
    def _from_ros(cls, cobot_prefix, skill_prefix="~", cobot=None):
        skill_prefix = hrr_common.fix_prefix(skill_prefix)
        out = cls(f_contact=hrr_common.get_param(f"{skill_prefix}f_contact", 1.0),
                  hover_distance=hrr_common.get_param(f"{skill_prefix}hover_distance", 1e-2)
                  )
        out.init_skill_base(cobot_prefix=cobot_prefix, cobot=cobot)
        gripper_ns = f"{hrr_common.fix_ns(cobot_prefix)}{hrr_common.fix_prefix(hrr_common.get_param(f'{cobot_prefix}gripper_ns'))}"
        out.init_ros(
            hrr_common.get_param(f"{cobot_prefix}sensitive_grasping_action_srv_name"),
            gripper_ns=gripper_ns
        )
        out.strategies = hrr_common.get_param(
            f"{skill_prefix}grasping_strategies",
            np.r_[out.grasping_controller.F_CTRL, out.grasping_controller.F_CTRL, out.grasping_controller.F_CTRL,
                  0, 0, out.grasping_controller.V_CTRL])
        out.scale_pos_vel = hrr_common.get_param(f"{skill_prefix}scale_vel", out.scale_pos_vel)
        out.closing_speed = hrr_common.get_param(f"{skill_prefix}closing_speed", out.closing_speed)
        out.closing_width = hrr_common.get_param(f"{skill_prefix}closing_width", out.closing_width)
        #out._open_width = hrr_common.get_param(f"{skill_prefix}opening_width", out._open_width)
        out.tcp_link = hrr_common.get_param(f"{skill_prefix}tcp_link_name", out.tcp_link)
        out.dsa_link = hrr_common.get_param(f"{skill_prefix}dsa_link_name", out.dsa_link)
        out._rotation_steps = hrr_common.get_param(f"{skill_prefix}rotation_steps", out._rotation_steps)
        out.drop_off_hover_distance = hrr_common.get_param(f"{skill_prefix}drop_off_hover_distance",
                                                           out.drop_off_hover_distance)
        out._lifting_force = hrr_common.get_param(f"{skill_prefix}f_lift", out._lifting_force)
        out._rotation_vel = hrr_common.get_param(f"{skill_prefix}rotation_velocity", out._rotation_vel)
        out._grasp_timeout = hrr_common.get_param(f"{skill_prefix}grasp_timeout", out._grasp_timeout)
        out.observer.set_buffers(out.cobot)
        return out

    def recovery(self):
        """Recover robot from an error, i.e. drive back to pre-pose via
        the :py:meth:`hrr_cobot_robot.hrr_cobot_control.HrrCobotControl.goTo` command.

        Also set gripper velocity to 0.0 and try to open 0.2 seconds after initiating the recovery motion.

        .. note::

            this function will increase the `F_max` wrench threshold to allow movement by 10%.
            This may be insufficient in some cases, and will require a manual reset then.
            Similarly, the threshold is reset at the end, so in case the function is interrupted,
            the threshold might be false afterwards.
        """
        F_max_prev = self.cobot.F_max
        if np.linalg.norm(self.cobot.B_F_msr) > self.cobot.F_max:
            self.cobot.F_max = max(1.1 * np.linalg.norm(self.cobot.B_F_msr), F_max_prev)
        self.cobot.gripper.set_vel(0.0)
        t_pub = rospy.get_time()
        if isinstance(self._T_B_E_goal, sm.SE3):
            self.cobot.move_to_pose(sm.SE3(self.hover_distance * self.B_surface_normal) @ self._T_B_E_goal)
        while self.cobot.state not in [None, "FAILURE"]:
            self.cobot.update()
            if rospy.get_time() - t_pub > 0.2:
                self.cobot.gripper.send_pos(self._open_width, si=True)
                t_pub = rospy.get_time()
        self.cobot.F_max = F_max_prev

    @property
    def action_server_valid(self) -> bool:
        # todo: this property is deprecated. the stop is not in action_server_valid
        out = super().action_server_valid
        if not out:
            self.cobot.stop()
        return out

    def step(self, contact_as_success=False):
        """
        A default update step, via :py:meth:`hrr_cobot_robot.hrr_cobot_control.HrrCobotControl.update`, i.e.
        the update the current control-command according to the active control-law and update internal cobot states.

        Args:
            contact_as_success(bool, optional): flag to evaluate surface_contact as a success-event. Defaults to False.

        Returns:
            Union[bool, None]: None if no event has been detected. True, if success, False if an error occurs.
        """
        self.cobot.update()
        if contact_as_success and self.surface_contact:
            return True
        if not self.action_server_valid:
            return False

    def progress(self, contact_as_success=False, T_stop=np.inf):
        """
        detachable version of :py:meth:`~step`, i.e. this function is run
        until an error or success-state of a curren manipulation-primitive is encountered.

        Args:
            contact_as_success(bool, optional): flag to evaluate surface_contact as a success-event. Defaults to False.
            T_stop(float, optional): timeout to set break in update-loop. Defaults to None

        Returns:
            bool: True if manipulation-primitive has been executed successfully, False otherwise.
        """
        if self.failure:
            return
        t0 = rospy.get_time()
        while self.cobot.state is not None:
            fb = self.step(contact_as_success=contact_as_success)
            if fb is not None:
                return fb
            if rospy.get_time() - t0 >= T_stop:
                break
        return self.cobot.safety_check()

    def process_steps(self, steps: Sequence[SimpleMp]):
        """
        Process a sequence of :py:class:`~SimpleMP` by iteratively

        #. check for failure and exit if necessary
        #. update action-service feedback (c.f. :py:meth:`~update_feedback_msg`)
        #. if the current :py:class:`~SimpleMP` has the `wrench_bias_hack` set, do as commanded
        #. initialize mp according to `f_init` and the provided function-arguments
        #. call :py:meth:`~progress`             rospy.loginfo(f"using hover distance {self.hover_distance}")
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
            if not self.progress(mp.contact_as_success, mp.T_max):
                self.cancel(msg=f'failed during {mp.name}')
            self.cobot.stop()

    @property
    def surface_contact(self) -> bool:
        r"""evaluate if current F/T-measurement represents a environment contact, i.e.

        .. math::

            \left({\bf F}_{\mathrm{msr}} - {\bf F}_{\mathrm{bias-hack}, 0} - {\boldsymbol{\sigma}}_{\mathrm{noise}}\right)^{\top} {}^{B}{\bf n}

        Returns:
             bool: True, if the above is larger than `f_contact` (c.f. table above)
        """ 
        
        return (np.linalg.norm((self.cobot.B_F_msr[:3] - self._F0[0:3]) - 
                self.cobot.FT.noise[:3]) >= self.f_contact)
        
        # it was: return ((self.cobot.B_F_msr[:3] - self._F0[0:3]) -
        #         self.cobot.FT.noise[:3]).T @ self.B_surface_normal >= self.f_contact

    def grasp(self, grasp_timeout, shitsbroken=True):
        r"""Actual grasping routine, i.e. grasp the object and correct pose depending on feedback.
        """
        rospy.loginfo(f"Grasping now, will close until clothing width below {self._closing_width}")
        def _inner_hack_loop():
            self.cobot.set_compliant(np.r_[1, 1, np.zeros(3), 1], use_py_vel=True)
            self.cobot.gripper.set_vel(-np.abs(self.closing_speed), si=True)

        def _fail():
            if not self.action_server_valid or self.cobot.state == "FAILURE":
                self.cobot.gripper.set_vel(0.0, si=False)
                self.cancel(msg="failed to grasp object")
                return True
            return False

        self.cobot.reset_sensor()
        t0 = rospy.get_time()
        rospy.sleep(1)
        self.cobot.FT.reset_bias()
        rospy.loginfo(f"Starting while loop with {self.cobot.gripper.width}")
        while self.cobot.gripper.width > self._closing_width:
            if _fail():
                rospy.loginfo("Failed, while loop done")
                return False
            if shitsbroken:
                _inner_hack_loop()
                if rospy.get_time() - t0 > grasp_timeout:
                    rospy.loginfo("Grapsing time exceeded, while loop done")
                    self.cobot.update()
                    self.cobot.stop()
                    return True
            self.cobot.update()
        self.cobot.update()
        self.cobot.stop()
        return True

    def goTo_with_cancel(self, pose, v_max=0.04):
    # Like goTo but checks if action server was cancelled or timeout reached.
        self.cobot.move_to_pose(pose, v_max=v_max)
        for t in range(int(100 * self.cobot.hz)):
            if not self.action_server_valid:
                self.cancel(msg="Lightbulb grasping pre-empted")
                break
            elif self.cobot.state is None:
                return True
            elif self.cobot.state == "error":
                return self.end_skill(msg="Lightbulb grasping failed. Cobot in state 'error'.")
            else:
                self.cobot.update()

    
    def remove_lightbulb(self, T_B_object_des=None, T_B_E_goal=None):
        def approach_object():
            rospy.loginfo("Going to beginning pose with joint interpolation")
            self.cobot.move_to_joint_pose(np.r_[0, 0, -1.5708, 0., 1.5708, 1.5708], stochastic=False)
            self.cobot.gripper.send_pos(self._open_width, si=True)
            rospy.loginfo(f"Set gripper to width + {self._open_width}, real width is {self.cobot.gripper.width} ")
            self.hover_distance = 0.05
            self.goTo_with_cancel(sm.SE3(1.5*self.hover_distance * self.B_surface_normal) @ T_B_E_goal, v_max=0.1)
            self.process_steps((
                SimpleMp(name="approach_pre_pose", f_init=self.cobot.move_to_pose,
                         args=(sm.SE3(self.hover_distance * self.B_surface_normal) @ T_B_E_goal)),
                SimpleMp(name="approach_surface", f_init=self.cobot.set_py_hybrid_force_vel_command,
                         kwargs=approach_kwargs,
                         contact_as_success=True)))

        def first_grasp():
            self.update_feedback_msg("Initiating gripper closure")
            #self.cobot.gripper.set_vel(-np.abs(self.closing_speed), si=True)
            self.goTo_with_cancel(sm.SE3(0.0025 * self.B_surface_normal) @ self.cobot.T_B_E_robot, v_max = 0.05)
            self.grasp(20)

        def open_fingers(dt=0.01, compliant=True):
            #change this to open a fixed distance
            self.update_feedback_msg("release object")
            if self.easy_version:
                self.cobot.set_compliant(np.r_[1, 1, np.zeros(3), 1], use_py_vel=True)
                self.cobot.gripper.send_pos(self.cobot.gripper.width + 0.007, si=True)
                #used to be rospy sleep(0.2)
                for t in range(int(0.2/self.cobot.hz)):
                    self.cobot.update()

        def reposition_gripper():
            self.process_steps((
                SimpleMp(name="vertical_reposition", f_init=self.cobot.set_py_hybrid_force_vel_command,
                         kwargs=approach_kwargs, T_max=0.2), #was T_max = 1 but crazy shit happened :) 
                SimpleMp(name="vertical_reposition_contact_search", f_init=self.cobot.set_py_hybrid_force_vel_command,
                         kwargs=approach_kwargs, contact_as_success=True),
                SimpleMp(name="vertical_reposition", f_init=self.cobot.set_py_hybrid_force_vel_command,
                         kwargs=move_up_kwargs, T_max=0.5),
            ))
            self.update_feedback_msg("Initiating gripper closure")
            #self.cobot.gripper.set_vel(-np.abs(self.closing_speed), si=True)

        def remove_bulb(T_max):
            if self.easy_version:
                self.cobot.gripper.set_vel(-0.03, si=True)
                F0 = self.cobot.B_F_msr.copy()
                p0 = np.copy(self.cobot.T_B_E_robot.t)
                z0 = p0[2]
                removed = True
                self.cobot.move_to_pose(sm.SE3(self.hover_distance * self.B_surface_normal) @ self.cobot.T_B_E_robot, v_max=0.01)
                for t in range(int(10000 * self.cobot.hz)):
                    if self.cobot.state is None:
                        break
                        #We're at prepose
                    elif self.cobot.state == "error":
                        return self.end_skill(msg="Adaptive grasping failed. Cobot in state 'error'.")
                    if np.abs(((self.cobot.B_F_msr[:3] - F0[:3]) - self.cobot.FT.noise[:3]).T @ self.B_surface_normal) >= self._lifting_force:
                        rospy.loginfo(f"Force threshold {self._lifting_force} exceeded. Removal not succesful")
                        removed = False
                        self.cobot.gripper.set_vel(0, si=True)
                        self.cobot.gripper.send_pos(self.cobot.gripper.width + 0.0025, si=True)
                        rospy.sleep(0.1)
                        self.goTo_with_cancel(self.cobot.FK(self.cobot.q_calib), v_max=0.02)
                        break
                    if self.cobot.T_B_E_robot.t[2] - z0 > 0.02:
                        rospy.loginfo("stop upwards motion, gone far enough. Removal succesful")
                        removed = True
                        break
                    self.cobot.update()
            self.success = True

        if T_B_object_des is not None:
            self.B_object_des = T_B_object_des
        if T_B_E_goal is None:
            T_B_E_goal = hrr_common.calc_EE_goal_pose(B_normal=self.B_surface_normal, B_y_axis=self.B_tcp_y_axis,
                                                      T_C_E=self.cobot.T_E_C_robot.inv(),
                                                      B_p_location=self.B_object_des.t)
        # set skill parameters
        self._T_B_E_goal = T_B_E_goal
        self.cobot.init_sns_vel()
        approach_kwargs = self.cobot.default_hybrid_kwargs()
        approach_kwargs["scale_pos"] = self.scale_pos_vel
        approach_kwargs["vel_dir"] = np.r_[-self.B_surface_normal, np.zeros(3)]
        move_up_kwargs = copy.deepcopy(approach_kwargs)
        move_up_kwargs["vel_dir"] = np.r_[self.B_surface_normal, np.zeros(3)]
        pull_bulb_kwargs = copy.deepcopy(approach_kwargs)
        pull_bulb_kwargs["vel_dir"] = np.zeros(6)
        pull_bulb_kwargs["K_f"] = self._K_f * np.ones(3)
        pull_bulb_kwargs["wrench_dir"] = np.r_[1.0, 1.0, 1.0, np.zeros(3)]
        tf_msg = hrr_common.get_empty_msg_stamped(self.tcp_link, self.dsa_link)
        rotate_vel = np.zeros(6)
        cur_angle = 0.0
        self.success = False
        sequential_mps = {
            'approach': approach_object, 'initial grasp': first_grasp,
            'open_fingers': open_fingers, 'reposition finger': reposition_gripper}
        # execute
        for name, func in sequential_mps.items():
            self.update_feedback_msg(msg=f"Execute {name}")
            func()
            if self.failure:
                return
        self.grasp(grasp_timeout = 3)
        
        q0 = quaternion.from_rotation_matrix(self.cobot.T_B_C_robot.R)
        remove_bulb(self._rotation_steps * self.cobot.dt)
        
        #open_fingers()
        rospy.loginfo("Going back up")
        self.goTo_with_cancel(sm.SE3([0, 0, 2*self.hover_distance])@self.cobot.T_B_E_robot)

    def execute_skill_cb(self, goal):
        """
        Action-service callback.

        Runs :py:meth:`~hrr_cobot_robot.manipulation_sills.skill_base.SkillBase.pre_skill_execution`
         first to check, if robot is in a valid state.

        Checks for positive data for the following attributes, that fall back to the internal values otherwise:

        * ``contact_force``
        * ``gripper_open_width``
        * ``tilting_angle``
        * ``timeout``

        then runs :py:meth:`~remove_lightbulb` via transforming the `screw_location` and `surface_normal`
        into the base-frame of the robot as `B_screw_pos` and `B_normal`.

        Args:
            goal(AdaptiveGraspingGoal): action-service goal
        """
        rospy.loginfo(f"This is the dispose pose: {goal.dispose_off_location_pose.pose}")
        rospy.loginfo(f"This is object center: {goal.object_center.pose}")
        def check_positive(goal_arg, self_arg):
            g_val = getattr(goal, goal_arg)
            if g_val > 0.0:
                setattr(self, self_arg, g_val)
            else:
                rospy.logwarn(self._log_str(f"received none-positive goal-argument {goal_arg}:={g_val}. "
                                            f"Use default {getattr(self, self_arg)}"))
        if not self.pre_skill_execution(ToolType.WSG_50_DSA, hardcoded_transformation=False):
            return
        #cobot.FT.reset_bias
        self.cobot.gripper.reset()
        rospy.sleep(3)
        if self.cobot.gripper.width == 0:
            rospy.loginfo("Gripper width was 0 after reset, trying to start driver again.")
            self.cobot.change_tool("nothing")
            rospy.sleep(2)
            self.cobot.change_tool("wsg_50_dsa")
            rospy.sleep(2)
            self.cobot.gripper.reset()
            rospy.sleep(3)
            if self.cobot.gripper.width == 0:
                self.cancel(msg="Lightbulb grasping cancelled automatically since gripper driver not responding")
        # oldwidth = np.copy(self.cobot.gripper.width)
        # rospy.loginfo("Trying to close gripper a bit and see if width changes")
        # self.cobot.gripper.set_vel(-0.1,si=True)
        # rospy.sleep(0.2)
        # self.cobot.gripper.set_vel(0.0, si=True)
        # if oldwidth == self.cobot.gripper.width:
        #     self.cancel(msg="Lightbulb grasping cancelled automatically since gripper does not move")
        # else:
        #     rospy.loginfo("Could close gripper, resuming skill")

        check_positive("contact_force", "f_contact")
        check_positive("gripper_open_width", "_open_width")
        check_positive("tilting_angle", "_tilting_angle")
        check_positive("timeout", "timeout")
        B_object_des = self._tf.A_pose(goal.object_center, frame_A=self.cobot.base_link)
        #if not self.check_pose(B_object_des):
        #    return self.cancel(msg=f"final destination pose {B_object_des} is not reachable")
        self.B_object_des = B_object_des
        rospy.loginfo(f"Given position {B_object_des.t}")
        self.remove_lightbulb()
        if self.success:
            #Get pose with correct position for disposal
            #rospy.loginfo(f"This is the dispose quaternion: {goal.dispose_off_location_pose}")
            #rospy.loginfo(f"And this is the SE3 from it: {get_SE3_from_pose_stamped(goal.dispose_off_location_pose)}")
            #dispose_pos = get_SE3_from_pose_stamped(goal.dispose_off_location_pose) @ self.cobot.T_E_C_robot
            #Attach this position to the current orientation
            dispose_pose = self.cobot.T_B_E_robot.copy()
            [dispose_pose.t[0], dispose_pose.t[1], dispose_pose.t[2]] = [0.0192, 0.54566, 0.186+0.3]
            #go there
            self.goTo_with_cancel(sm.SE3([0, 0, 0.1]) @ dispose_pose, v_max=0.08)
            self.goTo_with_cancel(dispose_pose, v_max = 0.05)
            # self.cobot.gripper.set_vel(0, si=True)
            self.cobot.gripper.reset()
            rospy.sleep(1.5)
            self.cobot.gripper.send_pos(0.07, si=True)
            self.end_skill(msg="succeeded with grasping skill")
    @staticmethod
    def _log_str(msg):
        return f"Sensitive Grasping -> {msg}"

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

    def publish_feedback(self):
        """Generate CalibrateActionResult from code,

        Returns:
            CalibrateCobotResult: action-service result
        """
        try:
            self._as.publish_feedback(self._feedback)
        except AttributeError:
            rospy.logerr(f"failed to send feedback-message")

    @property
    def result(self) -> AdaptiveGraspingResult:
        """Generate AdaptiveGraspingResult from code,

        Returns:
            AdaptiveGraspingResult: action-service result
        """
        res = AdaptiveGraspingResult()
        self._update_skill_result()
        res.skill_result = self._skill_result
        return res


# def debug_node():
#     from hrr_cobot_robot.hrr_cobot_control import HrrCobotControl
#     rospy.loginfo(f"starting standalone grasping skill")
#     cobot = HrrCobotControl.from_ros(cobot_prefix="/hrr_cobot", compile_numba=False)
#     grasping = SensitiveGrasping.from_ros(cobot=cobot, cobot_prefix="/hrr_cobot",
#                                           skill_prefix="/hrr_cobot/sensitive_grasping")
#     grasping.pre_skill_execution(ToolType.WSG_50_DSA, hardcoded_transformation=False)
#     T_B_C_test = sm.SE3(0.66, 0.0, 0.05)
#     grasping.remove_lightbulb(T_B_C_test)
#     rospy.spin()
#
#
# if __name__ == "__main__":
#     rospy.init_node("debug_adaptive_grasping_node")
#     debug_node()
