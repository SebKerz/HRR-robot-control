# The HR-Recycler Cobot Hardware Setup

This package contains the major configurations to run the HR-Recycler Cobot on platform.

## Table of contents

- [The HR-Recycler Cobot Hardware Setup](#the-hr-recycler-cobot-hardware-setup)
  - [Table of contents](#table-of-contents)
  - [Start Robot](#start-robot)
    - [Prepare Robot Hardware / TP5 Panel](#prepare-robot-hardware--tp5-panel)
    - [Start Robot programs](#start-robot-programs)
    - [Launching the robot](#launching-the-robot)
    - [Test robot](#test-robot)
  - [Adjust Robot Setup / Parameters](#adjust-robot-setup--parameters)
    - [Generate ```.COD``` files from ```.PCL```](#generate-cod-files-from-pcl)
    - [Initial preparation](#initial-preparation)
    - [Adjust ROS parameters to hardware](#adjust-ros-parameters-to-hardware)
      - [Understanding the launch file](#understanding-the-launch-file)
  - [End-Effector Tool Options](#end-effector-tool-options)

## Start Robot

> [!TIP]
> Please refer to the instructions in the wiki: 
>
> **[https://wiki.tum.de/display/lsritr/Starting+the+Comau+robot+via+ROS](https://wiki.tum.de/display/lsritr/Starting+the+Comau+robot+via+ROS)**
> 
> for a detailed instruction.

### Prepare Robot Hardware / TP5 Panel

As outlined below in [Adjust robot Setup](#adjust-robot-setup--parameters),
the robot runs multiple programs to handle communication with an external (ROS) PC-client.
Thus, we need to load the required **```.COD```**-programs to the robot:

1. `pdl_tcp_functions` - **NO HOLD** program with utility functions for the TCP/IP communication
2. `state_server` - **NO HOLD** program that contains a TCP server for publishing robot's state
3. `motion_server` - **NO HOLD** program that contains a TCP server for receiving motion commands
4. `motion_handler` - **HOLD** program that executes the motion commands

> In short: **HOLD** programs stop and start with the robot drive, while **NO HOLD** continue to run.
> More critically, **state_server** manages the reading thread and
> **motion_server** the write thread with the ROS-client (see figure at  [Adjust robot Setup](#adjust-robot-setup--parameters))

### Start Robot programs

After loading the programs, start the programs in the TP5-panel:

[!TP5 image from Wiki->log in to see this image](https://wiki.tum.de/download/attachments/748847952/PXL_20210312_122351038_scaled.jpg)

### Launching the robot

The robot is started via

```bash
roslaunch hrr_cobot_robot hrr_cobot_hw.launch
```

and launches the default cobot consisting of

- Racer 5 cobot
- Force-Torque sensor (for now a JR3 sensor)
- WSG 50 gripper (see [gripper options](#gripper-options) for further details)

these hardware files are individually loaded in the config section and can be adjusted to the specific needs as desired.

### Test robot

In order to test if the robot moves you can use the

```bash
rosrun hrr_cobot_robot enable_sns_trk --sns_controller_name sensor_track_velocity_controller
```

which should **not** result in a loopish behavior. If so, you need to restart the ```montion_handler``` on the robot as outlined in the
[wiki](https://wiki.tum.de/x/VAELMg)

## Adjust Robot Setup / Parameters

For further information refer to the ```comau_driver``` and ```comau_bringup``` ros-package that focus on the robot standalone driver.

### Generate ```.COD``` files from ```.PCL```

As the TCP communication is threaded on the robot and the PC-side, the correct programs need to be run on both sides.
So this paragraph highlights the preparation steps needed on the ```COMAU Robot``` as shown on the diagram below.

![TCP communication](https://git.lsr.ei.tum.de/hr_recycler/hrr_cobot/-/blob/master/data/media/comau_driver_tcp_comm_diagram.svg)

### Initial preparation

1. load the provided PDL programs of the ```comau-experimental``` git to the control panel
2. Translate them into ```*.cod``` files via the ```FILES``` menu in the control panel.
    - select ```*.pdl``` files
    - Press File -> Translate

> **Check all .cod files to have the correct and identical communication frequency**

- motion handler
  
  ```
  ci_receive_frequency = 200
  ```

- motion server

  ```
  ci_receive_frequency = 200
  ```

- state server

  ```
  ci_publish_frequency = 200
  ```

### Adjust ROS parameters to hardware

> [!WARNING]
> **when setting the robot control frequency: The PDL programs and yaml files are not synced! So change both values before starting the robot!**

The current driver does not adjust the frequencies remotely but expects the PC-client to adjust its values accordingly.

Thus, the ros-control loop-frequency should be set to an identical value as the frquencies above.
By default, the robot hardware parameters are set in the [robot_hw param file](https://git.lsr.ei.tum.de/hr_recycler/hrr_cobot/-/blob/master/hrr_cobot_robot/config/robot_hw.yaml) file via:

```yaml
loop_hz: 200
```

In addition the controllers in use are loaded via the [config/controllers.yaml](https://git.lsr.ei.tum.de/hr_recycler/hrr_cobot/-/blob/master/hrr_cobot_robot/config/controllers.yaml) file, where the

```yaml
loop_hz: &robot_loop_hz 200
```

variable is added to allow setting individual controller to the maximum frequency of the robot.

> [!NOTE]
> **you can set the controller frequencies arbitrarily high as they are decoupled from the actual communication frequency, but keep in mind that this does not bare any benefit, as each controller is called by the controller-manager at the frequency of the robot read-write loop**

#### Understanding the launch file

Please use

```bash
roslaunch hrr_cobot_robot hrr_cobot_hw.launch --ros-args
```

to see possible arguments and adjustments. They should have sufficient documentation / be self-explanatory and default to a custom value that allows spawning a robot without any further adjustments.

In addition check the [```hrr_common```](https://git.lsr.ei.tum.de/hr_recycler/hrr_cobot/-/blob/master/hrr_common) package to check the robot URDF in Rviz before starting the real robot
or use the [```hrr_cobot_sim```](https://git.lsr.ei.tum.de/hr_recycler/hrr_cobot/-/blob/master/hrr_cobot_sim) package to simulate the robot in Gazebo before launching it.

## End-Effector Tool Options

In order to use multiple grippers with the same launch file, the following configurations are in use ```tool_name``` to change to gripper of choice

- ``wsg_50``
- ``wsg_50_dsa``
- ``shaftgrinder``
- ``screwdriver``
- ``vacuum``
