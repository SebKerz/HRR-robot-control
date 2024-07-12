.. |check| raw:: html

   <input checked="" type="checkbox">

.. |uncheck| raw:: html

   <input type="checkbox">


Releases / Tags (HRR-Cobot Meta Package)
******************************************

Open Todos / Issues:
=====================

* |uncheck| add collision avoidance including end-effector models
* |uncheck| add example & docu for cutting
* |uncheck| add example & docu for grinding
* |uncheck| add example & docu for default grasping tests
* |uncheck| add example & docu for vacuum pick / disposal
* |uncheck| add example & docu for tool-change routine

v0.4
=======

@gabler tschö mit üss

* added code docu for unscrewing and grasping
* removed `hrr_cobot_robot->manipulation_graphs`
* |check| add example for unscrewing
* |check| add example for sensitive grasping controller

affected packages: 
- hrr_cobot (v0.4)
- hrr_cobot_robot (v.0.4)



v0.3.1
=======

Adjusted calibration routine

- added plotting function to check more easily
- removed Calibration service
- adjusted action-message
- moved to HrrCobotControl handle

affected packages: 
- hrr_msgs (v0.3.1)
- hrr_cobot_robot (v.0.3.1)
- hrr_controllers (v0.3.1)

v0.3
========

removed ``hrr_tools`` 

``hrr_controllers`` 
-----------------------------

- added digital IO state controller
- added digital Pin controller
- added digital encoder controller
- moved ``ros_interfaces`` from ``hrr_cobot_robot`` to this package, i.e. added python controller-handles as dedicated package

``hrr_cobot_robot```
-----------------------

- removed ``wsg_50_hw_local`` as source is now to be become publically available
- moved ``ros_interfaces`` to ``hrr_cobot_robot``
- added / refined tool-control interfaces

``hrr_cobot``
--------------

- slimmed docu
- added pages support (testing)

``hrr_msgs``
-------------

- moved actions to ``hr_recycler_msgs``
- added new content needed for digital controllers
  
also see ``hrr_cobot_robot`` -> controller.yaml on how to use these controllers for Tool-controllers

v0.2
==============

(almost finalized) tool-changer integration

* |check| finalized tutorials for non-skill purposes

``hrr_cobot_cobot``
---------------------------------------

updated calibration routine to cope with arbitrary force-gravity vectors (e.g. pull from side by cable)
--> previous versions are thus incompatible as returns have been adjusted

**NOTE**: current calibration routine is unsafe for certain configurations (screwdriver and partially shaft-grinder)
as there is no collision avoidance included. Use with care, workaround / solution is in progress.

``hrr_msgs``
-------------------------------

* |check| clean ``hrr_msgs`` -> ```HybridForceVelocityCmd.msg``` removed gains
  now available in GainsConfig.h in ``hrr_controllers``, (check http://wiki.ros.org/dynamic_reconfigure)

tool-changer integration
-------------------------

* adjusted URDFs (``hrr_ee_tools`` to v0.2.0)
* adjusted launch files (``hrr_common`` to v0.2.0)
* added tool-control (see ``hrr_ee_tools`` assets)
* updated ``hrr_cobot_robot`` to v0.1.7 (added tool-chaning utility to cobot instance.)
* updated ``hrr_robot`` to v0.1.7 (added tool-controller to docu)
* added ROS-interface for ``tool_controller`` to ``hrr_cobot_robot`` (see ``gripper_interfaces`` module)
* added basic tool-changing notebook tutorial (nb 07)


v0.1.6 
=============

``hrr_cobot`` updates
---------------------------

updated docu and meta package version given latest changes below

``hrr_cobot_robot`` 
-------------------------

added change tool changer routine to ``hrr_cobot_robot``
-> still misses motion planning routine.


v0.1.5
==========

``hrr_cobot_robot`` 
----------------------

* add tool change launch and scripts


v0.0.2
==========

``hrr_ee_tools``
----------------

* added URDFs for screwdriver 
* added URDFs for shaftgrinder
* added URDFs for toolchanger
* added URDFs for vacuum gripper

v0.1.4
==========

hrr_cobot_cobot 
----------------

* switched calibration routine to action-service
* added tutorials for action-service to notebook 5
* adjusted gripper API / control
* added compliant robo control from DSA / gripper readings
* |check| test new pipeline and action services
* |check| adjust all gripper content --> recordings from early September are all incompatible by now

v0.1.3
===========

hrr_common & hrr_cobot_robot
-----------------------------

* correct transformation handling, **which was used inconsitently and just working by chance**
* adjusted ``hrr_cobot_robot`` w.r.t. new transformation handling
* adjusted basic action-service hanlding for skills (not tested)


hrr_msgs
------------

* added action-services for grasping, levering (maybe dropped) and unscrewing
* added SkillResult.msg
* **makes hrr_cobot_robot inompatible with current version (v0.1.2)**

v0.1.2
================

* Removed outdated source code and packages --> refer to @backup tag
* added notebooks to main docu (to be found under ```hrr_cobot```)
* added dynamic parameter configuration config files and tested basic connection to initiate skill-learning remotely.
* basic content for a skill is implemented for unscrewing 

v0.1.1
==============

* |check| adjust skill-implementations wrt new API
* |check| Removed ```hrr_manipulation_primitives```, ```hrr_skills``` and ```hrr_manipulation_skills```
* Instead added manipulation_skills and manipulation_primitives to ```hrr_cobot_robot``` python module

v0.1
======================

* adjusted ```hrr_cobot_robot``` API
* |check| added API structure + pipeline
* |check| adjust action services with new controller interface and pipeline


v0.0.4
===========

hrr_controllers
-----------------------------------

* Added hrr_compliant controller (beta / testing)
* Removed displacement controller nonsense --> useless
* Added Doku / REAMDE-content


v0.0.3
========


hrr_controllers
---------------------------------------------------------------------

Added velocity control in arbitrary frames to ``sensor_tracking_velocity_controller``
Also added some messages to ``hrr_msgs``

Note:
^^^^^^

* the reference frame 'tcp_controller', i.e. the frame of the sensor-tracking module is ignored by this controller 
* for any other frames, the desired command is transformed from this frame to the base frame of the robot.
* added initial version of the compliant controller in C++ which should be used later


v0.0.4 
=======

removed all python cobot related handles from ``hrr_common`` to ``hrr_cobot_robot``
-------------------------------------------------------------------------------------------

* improved docu
* ``hrr_common`` now restricted to providing general utilities and helper funcions
* added some examples and documentation for hybrid control action service in ``hrr_cobot_robot``
* added basic skills as flexbe states and some example behaviors in ``hrr_manipulation_primitives`` and ``hrr_manipulation_skills``


v0.0.3
=================

added basic manipulation primitives and skills 06/21
------------------------------------------------------------

* Added basic docu for :py:mod:`hrr_common` and :py:mod:`hrr_manipulation_primitives`
* Added new packages ``hrr_manipulation_primitives`` and ``hrr_manipulation_skills``
* Removed skills from previous development location ``hrr_common``

v0.0.2
===============

Consortium meeting 05/21
--------------------------------

Latest package state at consortium meeting 20.05.2021

content

* handles for robot, FT-sensor, gripper
* general hrr_cobot master handle
* ros-controller manager helpers
* command interfaces for joint trajectory control and sensor-tracking velocity control
* basic Cartesian displacement interfaces set up but skipped as it sucks balls


v0.0.1
=========

Base verion
--------------------------

undocumented state for whatever



