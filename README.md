# HR-Recycler Cobot package

This package was developed during the Horizon 2020 project **Hybrid Human-Robot RECYcling plant for electriCal and eLEctRonic equipment**, https://cordis.europa.eu/project/id/820742/results,
at the Chair of Automatic Control Engineering, Technical University Munich.

It wraps a COMAU-Racer 5 0.80 industrial robot or cobot with dedicated tools and grippers.

The focus was on providing 
- a framework for the ROS-based control of the robot, including motion planner and end-effector control in SE(3),
- control capabilities of end-effector tools (electric and pneumatic gripper, screwdriver, shaft grinder)
- force-guided manipulation skills that rely on F/T-measurements at the wrist of the robot to cope with uncertain visual input
  
## Package-overview

- ``hrr_cobot_robot`` contains tool controllers and manipulation skills.
- ``hrr_common`` contains URDFs, common code snippets / helper functions, as well as the rviz launch/configuration file to evaluate the robot setup before launching.
- ``hrr_controllers`` contains robot controllers.

For a more complete documentation, please reach out.
