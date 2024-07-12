


HR-Recycler Cobot Python Documentation
=======================================

This is the HR-Recycler Cobot ROS-meta package Documentation.
This page simply shows the table-of contents of this documentation,
that mainly gives insights on the actual python utilites in use
throughout the ROS-(sub)-package(s).
In short, the main content is summarized as:

* :ref:`miniwiki` as a collective wiki on how to set up, start or use the presented software modules
* :ref:`hrrcommon` as a collection of ROS-utils that can used in arbitrary ROS-/robot projects
* :ref:`hrrcobot` as the main collection of python modules and software packages in use
* :ref:`hrrcontrollers` as the implementation of ROS-controllers and their individual python handles and high-level interfaces

..
  * :ref:`HrrCobotTutorials` containing some selected use-case examples / tutorials that can be used
   for testing and getting familiar with the individual code-snippets of this pacakge

With a detailed table of contents, below, the most important links are given as:

**ROBOT / PC setup**

   * :ref:`HRR_SYS_SETUP` for installing and preparing your PC for this repo.
   * :ref:`HRR_ENV_SETUP` for setting up the environment and software for this repo.
   * :ref:`HRR_ROBOT_START` for starting the ROS-pipeline with an actual robot platform.
   * :ref:`HRR_WTF` as an overview about where is to be found which package / parameter file.


**READ Robot data:** (:ref:`HRR_ROBOT_STATUS`)

   * :py:mod:`hrr_controllers.robot_status_if` as the python class API to the available robot state controllers
   * :py:mod:`hrr_controllers.sensor_handles` as the python class API for force-torque data reading

**WRITE full trajectories to robot** (:ref:`HRR_TRAJ_CMD`)

   * :py:mod:`hrr_controllers.trajectory_handling`  to send full trajectories to the robot

**WRITE / Send commands to robot in real-time** (:ref:`HRR_SNS_TRK`)

   * :py:mod:`hrr_controllers.sensor_track_velocity_commander` as the python class API to the sensor-tracking velocity-controllers
   * :py:mod:`hrr_controllers.sensor_track_compliance_commander` as the python class API to the sensor-tracking hybrid force-velocity controllers


**READ/WRITE** (:ref:`HRR_COBOT`)

   * :py:mod:`hrr_cobot_robot.hrr_cobot_handle` as the full cobot API that wraps the content from above and provides additional functionalities that rely on combined interfaces
   * :py:mod:`hrr_cobot_robot.hrr_cobot_control` extends the former by a variety of high-level commands

To close the loop, the class below is used for reading and store data from these handles during experiments:

   * :py:mod:`hrr_cobot_robot.hrr_cobot_observer` provides an interface for observing selected attributes of the ``cobot`` handles.

**Cobot tool control** (:ref:`HRR_EE_TOOLS`)

   * :py:mod:`wsg_50_hw.grasping_controllers` implements grasping control strategies.
   * :py:mod:`hrr_cobot_robot.tool_controller` wraps the python interface for the hrr end-effector tools from the ``hrr_ee_tools`` ROS-package.



Based on the handles above there are manipulation-skills implemented in

* :ref:`HRR_ROBOT_SKILLS`


Table of Contents
------------------

.. toctree::
  :glob:
  :maxdepth: 2

  README.md
  mini_wiki/index
  hrr_common/index
  hrr_controllers/index
  hrr_cobot_robot/index
  skills
  CHANGELOG
.. 
  notebooks/index




