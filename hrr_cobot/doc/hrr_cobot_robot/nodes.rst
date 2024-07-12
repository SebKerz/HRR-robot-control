
.. _HRR_ROBOT_NODES:

Available ROS-Nodes
=========================

This module lists the ROS nodes that are implemented within this package (or at least it should ...)

.. note::
    This documentation is limited to the nodes based on callable `click <https://click.palletsprojects.com/en/8.0.x>`_
    functions, that can be run as a ros-node (see below)

In case we have a simple function, e.g. `start_controller`, we can generate a ros node as

.. code-block::
    python

    import rospy
    import sys

    from hrr_cobot_robot.ros_nodes_click_commands import start_controller

    rospy.init_node("start_controller")

    if start_controller():
        sys.exit(1)
    sys.exit(0)




.. click:: hrr_cobot_robot.ros_nodes_click_commands:start_controller
   :prog: start_controller
   :nested: full

.. click:: hrr_cobot_robot.ros_nodes_click_commands:ft_sensor_calibrator
   :prog: ft_sensor_calibrator
   :nested: full

.. click:: hrr_cobot_robot.ros_nodes_click_commands:skill_server
   :prog: ft_sensor_calibrator
   :nested: full


.. click:: hrr_cobot_robot.ros_nodes_click_commands:update_tool_URDF_server
   :prog: update_tool_URDF_server
   :nested: full


