# Simulating the Racer5 0.80 in Gazebo and / or Mujoco

For MoveIt, one has to take into account that the robot description is mapped within the namespace of the robot for both mujoco and gazebo simulations.
By default MoveIt looks for the global ```/robot_description``` parameter. 
In order to spare problems in the long-term due to the possibility of multiple robots and thus multiple robot description parameters, this setup has been chosen on purpose.
In order to run moveit within rviz for testing, simply add the robot namespace (by default racer5) to the moveit namespace, robot description parameter and planning scene topic and the functionality should work as usual.

## Preparation

In case you have previously run any old instances, please kill all ros and gazenp processes via

```bash
kill -9 $(pgrep ros) && kill -9 $(pgrep gzclient) && kill -9 $(pgrep gzserver)
```

## Gazebo simulation package / pipeline

The main launch procedure to spawn the cobot in ``gazebo`` is done via

1. launching the gazebo environment to your liking (kept rather simplistic here)

   ```bash
   roslaunch hrr_cobot_sim gazebo.launch
   ```

2. loading the hrr_cobot URDF, spawn it in Gazebo and load required controller config files

   ```bash
   roslaunch hrr_cobot_sim spawn_hrr_cobot.launch
   ```

As both launch files provide a variety of arguments to be adjusted on the use-case, the launch file

```bash
roslaunch hrr_cobot_sim hrr_cobot_sim_demo.launch
```

spawns a demo simulation with the cobot being set to the center of a gazebo environment. 

This package can be extended with a moveit motion planning pipeline by calling

```bash
roslaunch hrr_cobot_moveit_config moveit.launch
```

once the simulation is up and runnning.

As the content of the launch file may differ from the content written here, 
we recommend to refer to the terminal documentation for each launch-file:

```bash
roslaunch hrr_cobot_sim gazebo.launch --ros-args
roslaunch hrr_cobot_sim spawn_hrr_cobot.launch --ros-args
```

>Please note that this package does not contain any URDFs. These URDFs are all kept in the ```hrr_common```(../hrr_common) package to keep the divergence of simulation and hardware models to a minimum.

>**Note: Due to the bottom of the racer robot a small offset is a currently recommended hack, in order to avoid contact simulations in gazebo.**

