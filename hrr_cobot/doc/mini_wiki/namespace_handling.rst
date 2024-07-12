.. _hrr_cobot_control.launch: https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/blob/main/hrr_cobot_robot/launch/hrr_cobot_control.launch

.. _hrr_cobot_hw.launch: https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/blob/main/hrr_cobot_robot/launch/hrr_cobot_hw.launch

.. _hrr_common.view_robot.launch: https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/tree/main/hrr_common/launch/view_robot.launch

.. _hrr_common.robot_state_publisher.launch: https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/tree/main/hrr_common/launch/robot_state_publisher.launch

.. _hrr_cobot_robot.config.net: https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/tree/main/hrr_cobot_robot/config/net

.. _hrr_cobot_robot.config.robot_hw.yaml: https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/tree/main/hrr_cobot_robot/config/robot_hw.yaml

.. _hrr_cobot_robot.config.controllers.yaml: https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/tree/main/hrr_cobot_robot/config/controllers.yaml


.. _ROS_NAMESPACING:

Namespaces & robot names
=============================


Within ROS there is no difference between nodes, so each topic or parameter-space
is open to the whole ROS-network.
As a result, topics must be unique, and similarly parameters.
For example, URDFs of robots are stored in ``/robot_description``-ROS parameters,
and most default controller / robot-tools require access to this ROS-parameter.

In order to prevent cross-talk across robot projects / nodes / robots, it is a good practice to
introduce namespaces where and when possible.

Within the ``hrr_cobot`` everything is stacked under the umbrella of the ``cobot``-namespace,
which by default is set to ``hrr_cobot`` in the `hrr_cobot_control.launch`_-file (c.f. :ref:`HRR_WTF`).

Thus, the major namespace order is given as:

.. code-block::

    /hrr_cobot                    # cobot-namespace
     ->controller_manager         # cobot-controller manager 
     ->gripper                    # WSG 50
     ->joints                     # output from /hrr_cobot/joint_state_controller
    [...]

similarly, this holds for the ROS-parameters, e.g.

.. code-block::

    /hrr_cobot/robot_description  # Cobot URDF
    /hrr_cobot/loop_hz            # cobot update rate
    /hrr_cobot/v_max              # maximum Cartesian translation velocity 

This requires to add workaround for some default ROS-nodes, e.g.
to replace the default http://wiki.ros.org/robot_state_publisher
by a slightly modified version at `hrr_common.robot_state_publisher.launch`.

.. warning:: 
    
    this launch-file is deprecated and now handled via the `update_tool_URDF_server` @ :ref:`HRR_ROBOT_NODES`

.. note::

    Within all ROS-nodes of the ``hrr_cobot`` no parameter / topic and node
    should be in the global namespace.


URDF-prefix
^^^^^^^^^^^^^

In case there are multuple links with identical names in ROS, e.g. base_link,
tf won't work properly anymore.
Thus, one can use the argument ``robot_prefix`` within the ``hrr_cobot`` package.

This is best to be checked via the `hrr_common.view_robot.launch`_, which can be e.g. called as

.. code-block:: bash

    ❯ roslaunch hrr_common view_robot.launch robot_prefix:="hrr_cobot." tool_name:="screwdriver"


which generates the robot-output as follows

.. image:: ./media/view_robot_example.png

i.e. every link and joint of the robot kinematic chain are extended with the prefix that has been provided to the cobot.
This argument can be set in the `hrr_cobot_control.launch`_.

.. warning:: 

    This requires every ROS-parameter to also reference the valid links / names (c.f. :ref:`HRR_WTF`)




