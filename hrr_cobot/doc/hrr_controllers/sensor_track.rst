.. _HRR_SNS_TRK:

Sensor Track Control Interfaces
-----------------------------------

this file contains the sensor-tracking control interfaces
that are available within this ROS-package.
For additional insights, a look into the ``hrr_controllers`` docu in the
``hrr_cobot`` meta-package docu is recommended.

Available Classes

- :ref:`SNS_TRK_VEL_CMD` to send Cartesian ee-velocity commands to the robot
- :ref:`SNS_TRK_CMPL_CMD` to send hybrid force-velocity profiles to the robot

Example usage is found in the second tutorial-notebook in the ``hrr_cobot`` package.

.. automodule:: hrr_controllers.sensor_track_velocity_commander
    :members:
    :undoc-members:

.. automodule:: hrr_controllers.sensor_track_compliance_commander
    :members:
    :undoc-members:
