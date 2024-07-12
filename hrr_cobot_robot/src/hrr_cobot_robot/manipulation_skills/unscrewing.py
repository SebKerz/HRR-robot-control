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
import secrets
from std_msgs.msg import Int8
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

fixed_pose = sm.SE3()
fixed_pose_straight = sm.SE3()

fixed_pose_straight.A[:4, :4] = [[0.81119, 0.00089, 0.58478, 0.58263],
                                 [-0.58245, -0.08804, 0.80809, -0.23876],
                                 [0.0522, -0.99612, -0.0709, 0.50576],
                                 [0., 0., 0., 1.]]
# fixed_pose_straight = fixed_pose @ T_outer

fixed_pose.A[:4, :4] = [[0.61443, 0.02356, 0.78862, 0.54851],
                        [-0.77968, -0.13481, 0.61149, -0.28939],
                        [0.12072, -0.99059, -0.06446, 0.7098],
                        [0., 0., 0., 1.]]

# TUM  # [[0.5555, -0.56509, 0.60999, 0.62164],
#                     [-0.52682, 0.3284, 0.78398, -0.25655],
#                     [-0.64334, -0.75685, -0.11528, 0.55416],
#                     [0., 0., 0., 1.]]  ## tilted by 5 degree

# TUM: fixed_joint_pose_PC = np.r_[0.69105, 0.18795, -1.56624, -1.58332, 1.56126, -3.69435]
fixed_joint_pose_MW_tilt = np.r_[0.60867, 0.42551, -1.24084, -1.53323, 1.26385, 3.34276]
fixed_joint_pose_PC = np.r_[0.72818, 0.10042, -1.51531, -1.56217, 1.38782, -3.04629]


class Unscrew(SkillBase):

    def __init__(self, f_contact=5.0, hover_distance=0.05,
                 f_unscrew=10.0, f_insert=20.0,
                 buffer_size=100,
                 closing_width=1e-2):
        self.align_buf = np.zeros((buffer_size, 6))
        super().__init__(name="unscrew_v2")
        self.do_surface_search = True
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
        self._deviceType = None
        self.force_threshold = 0.8 #Was 4.5
        self.fixed_pose_straight = sm.SE3()
        self.fixed_pose = sm.SE3()
        # for TUM. Delete for ECoreset!!

    def init_ros(self, action_srv_name):
        """initialize action-service and start. Refer to :py:meth:`~execute_skill_cb` for insights"""
        self._as = actionlib.SimpleActionServer(action_srv_name, UnscrewAction,
                                                execute_cb=self.execute_skill_cb, auto_start=False)
        self._as.start()

    @classmethod
    def _from_ros(cls, cobot_prefix, skill_prefix="~", cobot=None):
        skill_prefix = hrr_common.fix_prefix(skill_prefix)
        out = cls(f_contact=hrr_common.get_param(f"{skill_prefix}f_contact", 5.0),
                  hover_distance=hrr_common.get_param(f"{skill_prefix}hover_distance", 0.05)
                  )
        out.init_skill_base(cobot_prefix=cobot_prefix, cobot=cobot)
        out.init_ros(hrr_common.get_param(f"{cobot_prefix}unscrew_action_srv_name"))
        out.scale_pos_vel = hrr_common.get_param(f"{skill_prefix}scale_vel", out.scale_pos_vel)
        out.observer.set_buffers(out.cobot)
        return out

    def move_until_contact(self, goal_pose, v_max=0.01, force=80):
        contact = False 
        self.cobot.move_to_pose(goal_pose, v_max=v_max)
        rospy.loginfo(f"The current forces are {self.cobot.B_F_msr[:3]}")
        F0 = np.copy(self.cobot.B_F_msr[:3])
        for t in range(int(100 * self.cobot.hz)):
            if not self.action_server_valid:
                rospy.loginfo("Stopping during goTo motion since action server not valid")
                break
            elif self.cobot.state is None:
                rospy.loginfo(f"Movement done, found no contact. Force deviation {self.cobot.B_F_msr[:3] - F0}")
                break
            elif self.cobot.state == "error":
                rospy.loginfo("Problem with unscrewing goTo, cobot in state error")
                break
            elif np.linalg.norm((self.cobot.B_F_msr[:3] - F0) - self.cobot.FT.noise[:3]) >= force:
                rospy.loginfo(f"Found contact while goTo. Force deviation {self.cobot.B_F_msr[:3] - F0}")
                contact= True
                break
            else:
                self.cobot.update()
        return contact


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
        if not self.action_server_valid:
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
        forcediff = ((self.cobot.B_F_msr[:3] - self._F0[0:3]) - self.cobot.FT.noise[:3]).T @ self.B_surface_normal
        if forcediff >= self.f_contact:
            rospy.loginfo(f"made contact with force difference {forcediff}")
        return forcediff >= self.f_contact
        #return ((self.cobot.B_F_msr[:3] - self._F0[0:3]) -
        #        self.cobot.FT.noise[:3]).T @ self.B_surface_normal >= self.f_contact

    def unscrew(self, B_screw_pos=None, B_normal=None, T_B_E_goal=None):
        """
        Actual unscrewing task
        """
        def insert_screw():
            rospy.loginfo(f"Inserting with force {self.f_insert}N")
            self.process_steps((
                SimpleMp(name="insert", f_init=self.cobot.set_py_hybrid_force_vel_command, kwargs=insert_kwargs,
                         T_max=2.0),))

        def remove_screw():
            # force_z0 = abs((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[2])
            # print(force_z0)
            # self.cobot.run_screwdriver_program(self._screwdriver_program, run_time=5)
            # rospy.sleep(6)
            # print(abs((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[2]))
            # self.success = abs((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[2]) > force_z0
            
            #Go down a tiny bit, hopefully this helps put some pressure and keep inserted
            self.cobot.goTo(sm.SE3(-self.B_surface_normal * 0.001) @ self.cobot.T_B_E_robot, v_max=0.002)
            rospy.loginfo(
                f"Running screwdriver program {self._screwdriver_program} for {self._unscrew_time}s")
            self.cobot.run_screwdriver_program(self._screwdriver_program, run_time=self._unscrew_time)
            rospy.sleep(5.0)
            self.move_until_contact(sm.SE3(self.B_surface_normal * 0.03) @ self.cobot.T_B_E_robot, v_max=0.005, force=50)

        def search():
       
            T_B_E_goali = self._T_B_E_goal
            success = False
            if self._deviceType == 5:
                search_dist = 0.003
                search_vec = [sm.SE3(np.r_[0, 0, 0]), sm.SE3(np.r_[0, search_dist, 0]), sm.SE3(np.r_[0, -search_dist, 0]),
                        sm.SE3(np.r_[search_dist,0, 0]), sm.SE3(np.r_[-search_dist,0 , 0])]
            else:
                search_dist = 0.004
                search_vec = [sm.SE3(np.r_[0, 0, 0]), sm.SE3(np.r_[0, search_dist, 0]), sm.SE3(np.r_[0, -search_dist, 0]),
                        sm.SE3(np.r_[0, -2*search_dist , 0]), sm.SE3(np.r_[0, 2*search_dist , 0])]
            if self.do_surface_search:
                search_vec = [sm.SE3(np.r_[0,0,0])]
                old_threshold = self.force_threshold
                self.force_threshold = 1
            for i in range(len(search_vec)):
                T_B_E_goali = search_vec[i] @ self._T_B_E_goal
                rospy.loginfo(f"Now trying deviation {search_vec[i].t}")
                self.process_steps((
                    SimpleMp(name="approach_pre_pose", f_init=self.cobot.move_to_pose,
                            args=(sm.SE3(0.7*self.hover_distance * self.B_surface_normal) @ T_B_E_goali)),
                )
                    # SimpleMp(name="approach_surface", f_init=self.cobot.set_py_hybrid_force_vel_command,
                    #          kwargs=approach_kwargs,
                    #          contact_as_success=True, T_max=15))
                )
                self.move_until_contact(sm.SE3(- 1.5 * self.hover_distance * self.B_surface_normal) @ self.cobot.T_B_E_robot, v_max = 0.008, force=5)
                
                if not success and self.action_server_valid:
                    self._p0_contact = np.copy(self.cobot.T_E_C_robot.t)
                    forces_x = []
                    forces_y = []
                    self.cobot.run_screwdriver_program(self._screwdriver_program, run_time=1)
                    rospy.sleep(0.2)
                    self.cobot.goTo(sm.SE3(-self.B_surface_normal * 0.0015) @ self.cobot.T_B_E_robot, v_max=0.002)
                    rospy.sleep(0.5)
                    v_test = np.zeros(6)
                    forceTooHigh = False
                    rospy.sleep(0.5)
                    rospy.loginfo(f"Will wiggle now, using force threshold {self.force_threshold}")
                    for t in range(200):
                        if np.linalg.norm(self.cobot.B_F_msr)>=100:#self.cobot.F_max
                            forceTooHigh = True
                            break
                        v_test[1] = 1.5e-3 * np.sin(t / 100.0 * 2 * np.pi)
                        self.cobot.update(u_cmd=v_test, u_cmd_frame=self.cobot.ctrl_frame)
                        forces_x.append((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[0])
                        forces_y.append((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[1])
                    for t in range(200):
                        if np.linalg.norm(self.cobot.B_F_msr)>=100:#self.cobot.F_max
                            forceTooHigh = True
                            break
                        v_test[0] = 1.5e-3 * np.sin(t / 100.0 * 2 * np.pi)
                        self.cobot.update(u_cmd=v_test, u_cmd_frame=self.cobot.ctrl_frame)
                        forces_x.append((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[0])
                        forces_y.append((sm.SE3(self.cobot.B_F_msr[0:3]) @ self.cobot.T_B_C).t[1])
                    self.cobot.stop()
                    if forceTooHigh:
                        rospy.loginfo("Force was too high, we reject this probe no matter what")
                    if np.std(forces_x) > self.force_threshold and np.std(forces_y) > self.force_threshold:
                        rospy.loginfo(f"I think this is a screw, while wiggling forces have standard deviation {np.std(forces_x)}, {np.std(forces_y)}")
                        success = True
                        screw_pos = self.cobot.T_B_E_robot
                        break
                    else:
                        rospy.loginfo(f"I don't think this is a screw, while wiggling forces have standard deviation {np.std(forces_x)}, {np.std(forces_y)}")
                        success = False  
                        screw_pos = self.cobot.T_B_E_robot
                elif not self.action_server_valid:
                    self.cancel(msg="Pre-empted")
            if self.do_surface_search and not success:
                self.force_threshold = old_threshold
                #spiral: down right up up left left down down down right right right up up up up left left left left down down down down
                self.cobot.goTo(sm.SE3([0,0,0.002])@self.cobot.T_B_E_robot,v_max=0.05)
                self.cobot.goTo(sm.SE3([0.005,0.005,0.0])@self.cobot.T_B_E_robot,v_max=0.05)
                rospy.loginfo("I am starting the spiral now")
                screw_radius = 0.0042 #If we hit a screw, how much to move in the same direction?
                spiral_density = 0.005
                surface_search_speed = 0.005
                surface_search_force_threshold = 15
                spiral = "drruulldddrrruuuulllldddd"
                direction_sum = np.r_[0,0,0]
                for idx, elem in enumerate(spiral):
                    if elem == "d":
                        direction = np.r_[0,-1,0]
                    elif elem == "u":
                        direction = np.r_[0,1,0]
                    elif elem == "l":
                        direction = np.r_[1,0,0]
                    elif elem == "r":
                        direction = np.r_[-1,0,0]
                    else:
                        rospy.loginfo("Found weird string character in spiral string!")
                    if idx == len(spiral)-1: #We assume the last direction is not alone
                        if self.move_until_contact(sm.SE3(spiral_density*(direction_sum+direction))@self.cobot.T_B_E_robot,
                        v_max=surface_search_speed, force=surface_search_force_threshold ):
                            rospy.loginfo("Found screw via surface search")
                            success = True
                            screw_pos = sm.SE3(screw_radius*direction_sum/np.linalg.norm(direction_sum)) @ self.cobot.T_B_E_robot
                            self.cobot.goTo(sm.SE3([0,0,0.03])@self.cobot.T_B_E_robot,v_max=0.05)
                            self.cobot.goTo(sm.SE3([0,0,0.03])@screw_pos,v_max=0.05)
                            self.cobot.goTo(screw_pos,v_max=0.01)
                            break
                    elif direction_sum @ direction == 0 and not idx==0:
                        #New direction perpendicular
                        print(direction_sum)
                        if self.move_until_contact(sm.SE3(spiral_density*direction_sum)@self.cobot.T_B_E_robot,
                        v_max=surface_search_speed, force=surface_search_force_threshold):
                            rospy.loginfo("Found screw via surface search")
                            success = True
                            screw_pos = sm.SE3(screw_radius*direction_sum/np.linalg.norm(direction_sum)) @ self.cobot.T_B_E_robot
                            self.cobot.goTo(sm.SE3([0,0,0.03])@self.cobot.T_B_E_robot,v_max=0.05)
                            self.cobot.goTo(sm.SE3([0,0,0.03])@screw_pos,v_max=0.05)
                            self.cobot.goTo(screw_pos,v_max=0.01)
                            break
                        direction_sum = direction #Start again with new direction
                    else:
                        direction_sum += direction     
            return success, screw_pos
        
        self.B_screw_pos = B_screw_pos
        self.B_surface_normal = B_normal
        
        self.cobot.update()
        T_B_E_goal = sm.SE3()
        screwing_pose = sm.SE3()
        if self._deviceType == 2:
            if np.abs(self.B_surface_normal[2]-1)<0.008:
                #Microwave flat
                beginning_pose = np.r_[ 0.86289,  0.54018, -1.66603,  1.60357, -1.63773,  0.63349]
            else:
                beginning_pose = np.r_[ 0.72449,  0.59531, -1.42626,  1.4668 , -1.53804,  0.40122]
                #tilted
                #screwing_pose.A[:4, :4] = self.fixed_pose.A.copy()
            #    beginning_pose = fixed_joint_pose_MW_tilt#Change this to be same as annalenas fixed_pose
        elif self._deviceType == 3 or self._deviceType == 5:
            #PC
            beginning_pose = np.r_[ 0.96828,  0.17584, -1.5306 , -1.55063,  1.62805, -3.01406]
            #was fixed_joint_pose_PC before 20th october
        elif self._deviceType == 4:
            #Flat panel display
            if B_screw_pos[1]>0.05:#object center y>0
                beginning_pose = np.r_[-0.79806,  0.61373, -1.84638, -1.53851, -1.56184,  2.24939]
            else:
                beginning_pose = np.r_[ 0.8735 ,  0.55755, -1.96375,  1.63136, -1.63044,  0.94922]
        else:
            return self.cancel(msg="device type not fit for unscrewing")
        
        screwing_pose = self.cobot.FK(beginning_pose)
        T_B_E_goal.A[:4, :4] = screwing_pose.A.copy()

        tooltip_offset_A = B_screw_pos - (
                screwing_pose @ self.cobot.T_E_C_robot).t  # so bzw siehe notebook
        [T_B_E_goal.t[0], T_B_E_goal.t[1], T_B_E_goal.t[2]] = T_B_E_goal.t + tooltip_offset_A
        #Check for safety if screw position makes sense
        if self._deviceType == 2:
            #Microwave
            if B_screw_pos[2]>0.4 or B_screw_pos[2]<0.2:
                #Screw position is wrong, return success so that task planner can move on to next screw
                rospy.loginfo(f"Screw position {B_screw_pos} is rejected for microwave, sending success so we can move on.")
                self.success = True
                return
        if self._deviceType == 5:
            #PC Tower
            if B_screw_pos[2]>0.5 or B_screw_pos[2]<0.4:
                #Screw position is wrong, return success so that task planner can move on to next screw
                rospy.loginfo(f"Screw position {B_screw_pos} is rejected for FPD, sending success so we can move on.")
                self.success = True
                return

        # set skill parameters
        self._T_B_E_goal = T_B_E_goal
        self.cobot.init_sns_vel()
        approach_kwargs = self.cobot.default_hybrid_kwargs()
        approach_kwargs["scale_pos"] = self.scale_pos_vel
        approach_kwargs["vel_dir"] = np.r_[-self.B_surface_normal, np.zeros(3)]
        insert_kwargs = copy.deepcopy(approach_kwargs)
        insert_kwargs["wrench_dir"] = np.r_[1.0, 1.0, 1.0, np.zeros(3)]
        insert_kwargs["K_f"] = self._K_f * np.ones(3)
        insert_kwargs["vel_dir"] = np.zeros(6)
        insert_kwargs["B_F_des"] = np.r_[-self.B_surface_normal * self.f_insert, np.zeros(3)]

        # Go to Pre Pose
        rospy.loginfo(f"Approaching object, hover_distance {self.hover_distance}, surcace_normal {self.B_surface_normal}, screw position {self.B_screw_pos}") 
        #For safety reasons these movements stop if 10N force exceeded, and can be cancelled at any time
        if np.linalg.norm(self.cobot.T_B_E_robot.R - screwing_pose.R) < 0.001:
            rospy.loginfo("Already oriented correctly, we will use goTo for the prepose")
            rospy.loginfo(f"This is how much we want to move down: {sm.SE3(self.hover_distance * self.B_surface_normal)}")
            #self.move_until_contact(sm.SE3(-1 * self.hover_distance * self.B_surface_normal) @  self.cobot.T_B_E_robot, v_max = 0.04, force=5)
        else:
            rospy.loginfo("Not oriented correctly yet, using planner to do that")
            self.cobot.move_to_joint_pose(beginning_pose, stochastic=True)
            rospy.loginfo("Now we should be oriented correctly, we will use goTo for the prepose")
            #self.move_until_contact(sm.SE3(self.hover_distance * self.B_surface_normal) @ self._T_B_E_goal, v_max=0.04, force=5)
     
        if self.failure:
            return #self.cancel(msg="failed to execute search")
        
        # Reset the bias (Is this even needed?) 
        rospy.sleep(0.1)
        self.cobot.FT.reset_bias()
        
        #Search the screw 
        screw_found, screw_position = search()
        
        if self.failure:
            rospy.loginfo("failed to execute search")
            return #self.cancel(msg="failed to execute search")
        if screw_found:
            self._T_B_E_goal = screw_position
            self.success = True
            remove_screw()
        else:
            rospy.loginfo("failed to find screw, going back up and to beginning pose")
            self.move_until_contact(sm.SE3(self.B_surface_normal * self.hover_distance) @ self.cobot.T_B_E_robot, v_max=0.1)
            self.cobot.move_to_joint_pose(beginning_pose, stochastic=False)
            return

        self.move_until_contact(sm.SE3(self.B_surface_normal * self.hover_distance) @ self.cobot.T_B_E_robot, v_max=0.1)
        rospy.loginfo("Back to beginning pose.")
        self.cobot.move_to_joint_pose(beginning_pose, stochastic=False)

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

        self.cobot.change_tool("screwdriver")
        if not self.pre_skill_execution():
            return
        self._deviceType = rospy.wait_for_message('/hrr_cobot/deviceType', Int8).data
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
            self._skill_result.result = SkillResult.FINISHED
            self.end_skill("We think it failed, but telling task planner success so we can move on!")
            #self.cancel("Unscrewing failed due to unknown reasons")

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
