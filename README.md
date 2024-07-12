# HR-Recycler Cobot package

This package was developed during the Horizon 2020 project **Hybrid Human-Robot RECYcling plant for electriCal and eLEctRonic equipment**, https://cordis.europa.eu/project/id/820742/results,
at the Chair of Automatic Control Engineering, Technical University Munich.

It wraps a COMAU-Racer 5 0.80 / Cobot
with dedicated tools and grippers into a combined robot
setup in simulation and hardware.


## Package-overview

- ``hrr_cobot_robot`` spawns a combined robot hardware wrapping robot, gripper and force-torque sensor to a combined robot hardware setup. It further adds additional safety layers as a virtual ROS-based _deadman-switch_ to ensure safety halts and stops at any time during operation, even when the cobot is not within a caged cell.
- ``hrr_common`` contains the URDFs, common code snippets / helper function as well as the rviz launch/configuration file to evaluate the robot setup before launching.
- ``hrr_controllers`` contains the controller modalities to be used for the HR-Recycler cobot robot.
- ``hrr_msgs`` contains the custom messages used for the cobot

The documentation and explanations are gathered / found separately here:
`<https://hr_recycler.pages.gitlab.lrz.de/>`_.

## Installation Requirements

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

It is recommended to use

- **Ubuntu 20.04 with ROS noetic**
- Ubuntu 18.04 with ROS melodic

however Ubuntu 16.04 with ROS kinetic is expected to work.
