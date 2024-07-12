

.. _hrr_commonview_robot.launch: https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/tree/main/hrr_common/launch/view_robot.launch

.. _hrr_cobot_robot.config.net: https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/tree/main/hrr_cobot_robot/config/net

.. _comau_driver ROS-package: https://gitlab.lrz.de/lsr-itr-ros/comau-experimental


.. _HRR_WTF:

Wtf... but why??
=====================================================================

When you have had a closer look on this repo, and the ones linked to your workspace,
you may have ended up with the reaction below

.. image:: http://www.quickmeme.com/img/ed/ed2ba7236c7ddd8b1093e08b077a45f8cc35029a282e9a31cb61e225ac6d33a9.jpg

Thus, this page gives a short overview about the most important ROS-launch files and the overall structure of the project,
so by the end of this page, you may happily leave with this reaction Instead

.. image:: https://media.giphy.com/media/654unty0gaFji/giphy-facebook_s.jpg


ROS-parameter files
----------------------

These files contain or should contain all parameters in use per program, i.e.

* name of ROS-topics / services etc
* reference frame nams
* flags in use

Helpful links

- http://wiki.ros.org/Parameter%20Server
- https://roboticsbackend.com/what-is-a-ros-parameter/
- http://wiki.ros.org/rospy/Overview/Parameter%20Server
- http://wiki.ros.org/roscpp/Overview/Parameter%20Server
- https://www.mathworks.com/help/ros/ref/rosparam.html

The actual content / usage is outlined in the dedicated launch-files below, but in short summarized as



Explained n short

- `hrr_cobot_robot.config.net`_ all parameters needed to launch the robot ROS-driver (except the one below)
- `hrr_cobot_robot.config.hrr_cobot.prefix.yaml`_ combined updated version for robot-driver, controllers & python handles (fuses )
- `hrr_cobot_robot.config.robot_hw.yaml`_ network configuration for a specific ethernet connection with the robot & F/T -sensor
- `hrr_cobot_robot.config.controllers.yaml`_ lists all ROS-controllers that can be used, also adds the name
- `hrr_cobot_robot.config.wsg_50.yaml`_ contains the parameters in need for spawning the WSG 50 ROS driver with DSA support & alignment control.


.. _hrr_cobot_robot.config.hrr_cobot.prefix.yaml:

HR-Recycler Cobot Config file
""""""""""""""""""""""""""""""""""

.. note::

  this version has been added shortly before final project integration to prevent issues from identical tf-frames (link names) or joint-names.
  It is plainly a fused version of the individual ROS-parameter files listed below and is launched / loaded from `hrr_cobot_control.launch`_.
  It mainly differs from the others by allowing to set the URDF prefix to ``hrr_cobot.`` (c.f. :ref:`ROS_NAMESPACING`)

.. include:: ./config/hrr_cobot.prefix.yaml
    :code: xml



Single Config files
"""""""""""""""""""""""""""

.. warning:: This became deprecated. The links have been replaced by a link to an old commit

.. _hrr_cobot_robot.config.controllers.yaml: https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/blob/aa5c92788271e00b3130a54f95677b98cde05909/hrr_cobot_robot/config/controllers.yaml

.. _hrr_cobot_robot.config.robot_hw.yaml: https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/blob/aa5c92788271e00b3130a54f95677b98cde05909/hrr_cobot_robot/config/robot_hw.yaml

.. _hrr_cobot_robot.config.wsg_50.yaml: https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/blob/aa5c92788271e00b3130a54f95677b98cde05909/hrr_cobot_robot/config/wsg_50.yaml



Launch files
-----------------

Below you find the launch files in use within this project and a short explanation what they
are actually doing / starting.


hrr_cobot_robot `hrr_cobot_control.launch`_
"""""""""""""""""""""""""""""""""""""""""""""""

The `hrr_cobot_control.launch`_-file loads the majority of config-files during startup, while being mostly restricted to
ROS-modules that are directly related with the actual robot driver.

The parameters are listed on the top of the launch-file, where the
From a terminal, one can read the options quickly via

.. code-block:: bash

    roslaunch hrr_cobot_robot hrr_cobot_control.launch --ros-args


In case the explanations above are misleading or of no further help, the content below may (hopefully) help you:

* the files in `hrr_cobot_robot.config.net`_ define the IP and port of the COMAU control box,
  i.e. the physical ethernet connection between robot and control-unit. They are expected to be launched within the namespace of the robot, and are included by default
  in the `hrr_cobot_control.launch`_-file. The value of the file can be adjusted via the ``hw_net_file``.

* `hrr_cobot_robot.config.robot_hw.yaml`_ contains the robot-driver parameters, such as update rate, sensor-track confiuration, services in use.
  For further insight, we refer to the `comau_driver ROS-package`_.
  This file is also loeaded by default within the  `hrr_cobot_control.launch`_-file, and can be adjusted via the ``hw_config_file``.

* `hrr_cobot_robot.config.controllers.yaml`_ define the ROS-controller, and are expected to be set in the identical namespace as
  the cobot, given that the namespace is within the robot-namespace. What may sound silly, is explained best with teh sketch below

  .. code-block:: bash

    > rosnode list

    hrr_cobot                    # cobot-namespace
    hrr_cobot/controller_manager  # controller-namespace (as a chield of the cobot)

    > rosparam list | grep my_awseome_controller
    /hrr_cobot/my_awseome_controller/type    # the namespace of the controller is in the cobot-namespace
    /hrr_cobot/my_awseome_controller/loop_hz
    [...]

  again this is included by default in in the  `hrr_cobot_control.launch`_-file, and can be adjusted by ``controller_config_file``.
  alternating the ``controller_config_file``
  Furthermore, `hrr_cobot_control.launch`_-file allows to set the controllers directly loaded and/or started upon start
  via the ``stopped_controllers`` (load only) and ``controllers`` (load and start) ros-argument.

* (**deprecated**) It loads the ``hrr_cobot_robot->config->hrr_cobot.yaml`` parameter file, that is needed for the :ref:`HRR_COBOT` handles.

* It sets the current tool via the ``/hrr_cobot/tool_name`` ROS-parameter of the ``hrr_cobot``.


.. _hrr_cobot_control.launch:

.. include:: ./hrr_cobot_robot_launch/hrr_cobot_control.launch
   :code: xml

hrr_cobot_robot `hrr_cobot_hw.launch`_
"""""""""""""""""""""""""""""""""""""""""


This launch file combines the following steps:


#. It sets the ``/hrr_cobot/robot_description`` parameter via the `hrr_cobot.launch`_-file
#. It spawns the `urdf_updater.launch`_ that updates the URDF, whenever the value of the ROS-parameter ``/hrr_cobot/tool_name`` is changed.
#. It spawns the ``ft_calibrator`` ROS-node that allows to calibrate the F/T-sensor when needed.
#. It launches the `robot_state_publisher.launch` file to allow for nested URDF files in the robot namespace.
#. It launches the `hrr_cobot_control.launch`_ file in the correct namespace and forwards the parameter files as outlined above.
#. optionally launches rviz if ``use_rviz`` is set to ``true``.

.. _hrr_cobot_hw.launch:

.. include:: ./hrr_cobot_robot_launch/hrr_cobot_hw.launch
  :code: xml



hrr_cobot_robot `urdf_updater.launch`_
""""""""""""""""""""""""""""""""""""""""""

.. _urdf_updater.launch:

.. include:: ./hrr_cobot_robot_launch/urdf_updater.launch
  :code: xml



hrr_common `hrr_cobot.launch`_
""""""""""""""""""""""""""""""""""""""""""

.. _hrr_cobot.launch:

.. include:: ./hrr_common_launch/hrr_cobot.launch
  :code: xml

This modular URDF loading file allows to set a bunch of parameters for the robot-URDF from a launch file and
thus in a programmatic way, as used by the URDF

.. _tool_changer:

.. include:: ./scripts/urdf_updater_server
  :code: python


