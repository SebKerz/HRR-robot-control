/The robot does not check force values at any moment, 
# ```hrr_controllers``` ROS package

This package contains various controllers in use for the COMAU-driver / robot.

## comau_driver recap

>**warning**
>
> Please refer to the actual ``comau-driver-readme`` for further insights:
[https://gitlab.lrz.de/lsr-itr-ros/comau-experimental](https://gitlab.lrz.de/lsr-itr-ros/comau-experimental).

The ROS-controllers within this package operate via the ``hardware-interfaces`` of the ``comau-driver``
ROS-package, which is visualized as

![driver overview](./media/comau_driver_component_diagram.svg)

### Comau custom hardware interfaces

The available ROS-hardware interfaces are listed below

| Name  | read / write  | Purpose |
|---------|---------| ---|
|JointStateInterface     |   read      |   read joint data |
|[CartesianStateInterface](https://gitlab.lrz.de/lsr-itr-ros/comau-experimental/-/tree/master/comau_driver/include/comau_driver/comau_hw_interfaces/cartesian_state_interface.h)     | read         | read Comau Cartesian output|
|[TP5StateInterface](https://gitlab.lrz.de/lsr-itr-ros/comau-experimental/-/tree/master/comau_driver/include/comau_driver/comau_hw_interfaces/tp5_state_interface.h)       |read      | read TP5 / control-unit status |
| [DigitalIOStateInterface](https://gitlab.lrz.de/lsr-itr-ros/comau-experimental/-/tree/master/comau_driver/include/comau_driver/comau_hw_interfaces/digital_io_interface.h)     |  read |read DIN / DOUT pins|
|[DigitalPinStateInterface](https://gitlab.lrz.de/lsr-itr-ros/comau-experimental/-/tree/master/comau_driver/include/comau_driver/comau_hw_interfaces/digital_io_interface.h)   |read       | read specific pin |
|PositionJointInterface     |     read/write    |   read/write joint data |
|[SensorTrackingCommandInterface](https://gitlab.lrz.de/lsr-itr-ros/comau-experimental/-/tree/master/comau_driver/include/comau_driver/comau_hw_interfaces/sensor_tracking_command_interface.h)       |    read/write        | send Sensor-track command (rpy) to robot | 
|[SensorTrackingPoseCommandInterface](https://gitlab.lrz.de/lsr-itr-ros/comau-experimental/-/tree/master/comau_driver/include/comau_driver/comau_hw_interfaces/sensor_tracking_command_interface.h)       |      read/write    |  send Sensor-track command to robot
|[JointTrajectoryCommandInterface](https://gitlab.lrz.de/lsr-itr-ros/comau-experimental/-/tree/master/comau_driver/include/comau_driver/comau_hw_interfaces/joint_trajectory_command_interface.h)     |      read/write         |send Joint trajectory to robot|
|[CartesianTrajectoryCommandInterface](https://gitlab.lrz.de/lsr-itr-ros/comau-experimental/-/tree/master/comau_driver/include/comau_driver/comau_hw_interfaces/cartesian_trajectory_command_interface.h)     |    read/write      | send Cartesian trajectory (rpy) to robot| 
|  [CartesianPoseTrajectoryCommandInterface](https://gitlab.lrz.de/lsr-itr-ros/comau-experimental/-/tree/master/comau_driver/include/comau_driver/comau_hw_interfaces/cartesian_trajectory_command_interface.h)   |      read/write       |  send Cartesian trajectory to robot| 
| [DigitalPinCommandInterface](https://gitlab.lrz.de/lsr-itr-ros/comau-experimental/-/tree/master/comau_driver/include/comau_driver/comau_hw_interfaces/digital_io_interface.h)  | read / write  | read/write specific pin |

## implemented controllers

As listed in the [controllers_plugin.xml](https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/blob/main/hrr_controllers/controller_plugins.xml), the following controllers are implemented:

- ```hrr_controllers/CartesianStateController```: reads data from the ```CartesianStateInterface```, i.e. EE-Pose and publishes it to the ```tf2_ros``` broadcaster.
- ```hrr_controllers/TP5StateController```: reads data from the ```TP5StateInterface``` which consists of EE-pose, joints and robot state.
- ```hrr_controllers/SnsTrkVelocityPoseController``` and ```hrr_controllers/SnsTrkVelocityController```:
- reads and writes via the ```SensorTrackingInterface``` or the ```SensorTrackingPoseInterface```. As the former relies on roll-pitch-yaw rather than quaternions, the later is recommended to be used.
- ```hrr_controllers/ComplianceController``` and ```hrr_controllers/CompliancePoseController```:
    Basic compliance controller using an external (expected to be calibrated) FT sensor and a simple
    proportional gain controller to follow force and forward velocity commands.
    Differs only by HW-interface implementation.
- ```hrr_controllers/DigitalIOStateController``` reads and publishes the DOUT/DIN states of the robot
- ```hrr_controllers/DigitalPinController``` allows to set the state of DOUT-pins from ROS
- ```hrr_controllers/DoutEncoderController``` allows to control a digital encoder via DOUT pins

The state-controllers can be launched without further problems as reading is established without further issues once the TCP-connection with the state_server is established.

### Cartesian Velocity Control

This controller is the only feed-forward online control interface available to the robot that allows to traverse teh robot from A to B while allowing to alter the position on-the-fly.

There are some things to take into account when operating the robot with this controller:

```
The robot does not check force values at any moment, but applies the set velocity as is given the current velocity constraints. So please set these limits at dedicated low limits during development / testing!
```

- the robot resets the velocity command to zero after not receiving a new message for a predefined timeout, which can be set via the ```dead_man_timeout``` ROS-parameter.
- the commanded velocity is expected to follow the convention of the sensor-track controller and thus does not alter / transform the velocity.
In case another reference frame is chosen, the center of rotation is set to the origin of said frame.
- the controller uses the ```sensor-tracking-interface``` which is actually a correction command that is sent to the robot online. Unfortunately, these corrections are limited in total travel distance. Quick fixes:
  
  - increase the travel distance, via ROS-parameter upon launch
  - increase the travel distance by updating the sensor-track parameters online
  - switch to joint control an set goal to current configuration to reset the reference pose

### Compliant Control

```
When setting all selection values for the velocity control to ```True```,
this controller behaves (or rather should behave) identical to the [Cartesian Velocity Controller](#cartesian-velocity-control). 
So check the notes above for additional info.
```

This controller relies on external FT-reading, which may suffer from false calibration.
Thus, this controller expects a manual acknowledgement of the current FT-data before starting.

```
Handle this with care as the robot may experience undesirable behavior if incorrect FT-data is acknowledged as acceptable.
```

Before setting the acknowledgement it is impossible:

- to enable any entry of the force selection matrix
- enable force-control to the section above

This step has to be repeated any time the controller is restarted.

```
The controller allows to disable the FT-publishing. Nonetheless, this may be a useful/ essential feature to verify the current FT-data in the controller.
```

Gains are limited via the dynamic-reconfigure limits and the internal hard coded upper / lower bounds.
