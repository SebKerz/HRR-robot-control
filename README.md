# HR-Recycler Cobot package

This package was developed during the Horizon 2020 project **Hybrid Human-Robot RECYcling plant for electriCal and eLEctRonic equipment**, https://cordis.europa.eu/project/id/820742/results,
at the Chair of Automatic Control Engineering, Technical University Munich.

It wraps a COMAU-Racer 5 0.80 / Cobot
with dedicated tools and grippers into a combined robot
setup in simulation and hardware.

## Table of Contents

- [HR-Recycler Cobot package](#hr-recycler-cobot-package)
  - [Table of Contents](#table-of-contents)
  - [Package-overview](#package-overview)
  - [Installation](#installation)
    - [Requirements](#requirements)
    - [Building procedure & system setup](#building-procedure--system-setup)
  - [Real robot](#real-robot)
    - [Available Controllers](#available-controllers)
    - [Working with the robots](#working-with-the-robots)
  - [Current Issues](#current-issues)

## Package-overview

- ``hrr_cobot_robot`` spawns a combined robot hardware wrapping robot, gripper and force-torque sensor to a combined robot hardware setup. It further adds additional safety layers as a virtual ROS-based _deadman-switch_ to ensure safety halts and stops at any time during operation, even when the cobot is not within a caged cell.
- ``hrr_common`` contains the URDFs, common code snippets / helper function as well as the rviz launch/configuration file to evaluate the robot setup before launching.
- ``hrr_controllers`` contains the controller modalities to be used for the HR-Recycler cobot robot.
- ``hrr_msgs`` contains the custom messages used for the cobot

The documentation and explanations are gathered / found separately here:
`<https://hr_recycler.pages.gitlab.lrz.de/>`_.

## Installation

### Requirements

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

It is recommended to use

- **Ubuntu 20.04 with ROS noetic**
- Ubuntu 18.04 with ROS melodic

however Ubuntu 16.04 with ROS kinetic is expected to work.

### Building procedure & system setup

This has been moved to a dedicated [wiki section->system-setup](https://hr_recycler.pages.gitlab.lrz.de/mini_wiki/system_setup.html)

## Real robot

The cobot starting procedure is documented at the following locations

- **First Time PC setup**: The PC-setup and configuration is documented / sketched [wiki section->robot-startup](https://hr_recycler.pages.gitlab.lrz.de/mini_wiki/robot_startup.html#prepare-system-environment)
- **Starting the robot hardware and ROS-driver**: the explanation about the starting procedure of the robot is found in the [```wiki section->robot-startup->launch the hrr cobot ros driver```](https://hr_recycler.pages.gitlab.lrz.de/mini_wiki/robot_startup.html#launch-the-hrr-cobot-ros-driver)

### Available Controllers

a short outline of the available controllers is given in the
[```hrr_controllers``` package documentation](https://hr_recycler.pages.gitlab.lrz.de/hrr_cobot/hrr_controllers/index.html)

### Working with the robots

building upon the controllers, and collected utils in the
[```hrr_common``` package documentation](https://hr_recycler.pages.gitlab.lrz.de/hrr_cobot/hrr_common/index.html), the
[```hrr_cobot_robot``` package documentation](https://hr_recycler.pages.gitlab.lrz.de/hrr_cobot/hrr_cobot_robot/index.html)
wraps the content together.

Eventually, there exist a collection of jupyter tutorial notebooks in the
[notebooks](https://git.lsr.ei.tum.de/hr_recycler/hrr_cobot/-/blob/master/notebooks) directory,
that serve both as a sequential functionality check as well as an introduction for
the hardware in use.

## Current Issues

- Simulation package only partially maintained. Refer to [sim_robots](https://gitlab.lrz.de/hr_recycler/sim_robots) instead.
- MoveIT support is broken / not tested / maintained. In particular to be done:

  - [ ] adjust handles from COMAU to current interfaces
  - [ ] needs rework / fusing with previous states
