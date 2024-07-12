.. _hrrcontrollers:

hrr_controllers - Python-interface to communicate with ROS-controllers
=========================================================================

This ROS-pacakge contains the implemented ROS-controllers for the `hrr_cobot`.
On top, there exists a python-module with the identical name, that provides
the python interface for the underlying ROS-controllers in use.

Namely:

* :ref:`HRR_ROBOT_STATUS` contains python handlers for arbitrary robots and an additional wrapper for
  the COMAU robots, relying on the ``hrr_controllers/TP5stateController``-controller output.
* :ref:`HRR_SNS_TRK` wraps the ROS interfaces to send the following commands

    * Cartesian velocity commands to the ``hrr_controllers/SnsTrkVelocityPoseController``/ ``hrr_controllers/SnsTrkVelocityController`` - controllers.
    * Hybrid Force-velocity commands to the ``hrr_controllers/ComplianceController`` / ``hrr_controllers/CompliancePoseController`` - controllers.

* :py:mod:`hrr_controllers.sensor_handles` wraps the force-torque related python handlers,
  which read sensor-data from the FT-sensor and allows to update the load data as well as applying some
  selected filters to smoothen the data output.
  This class is currently also used to feed filtered / calibrated data to the 
  ``sensor-track compliance controller```.
* :py:mod:`hrr_controllers.trajectory_handling` provide the interfaces to the ``comau_controllers/JointTrajectoryController`` controller
  to command full trajectories to the robot.
* :ref:`HRR_DIGITAL_CTRL` use the digital pin control of the COMAU robot to control various end-effector tools or built in robot functionalities, such as pneumatic valves, etc.



.. toctree::
   :maxdepth: 3

   README
   robot_state
   ft_interface
   digital_controllers
   sensor_track
   trajectory_control
   wsg_50


.. automodule:: hrr_controllers
    :members:
    :undoc-members:

.. automodule:: hrr_controllers.utils
   :members:
   :undoc-members:
