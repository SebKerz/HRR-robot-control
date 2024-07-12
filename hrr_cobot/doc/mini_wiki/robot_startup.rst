.. _HRR_ROBOT_START:

Robot Configuration and statup routine
------------------------------------------



Preparing the robot programs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As the robot programs in use changed throuought the development, please varify the current
versions in use against the dedicated explanation


Final pilot configuration / setup
"""""""""""""""""""""""""""""""""""

This expects ``hrr_cobot >= 0.2`` and ``comau_experimental >= 0.7``
After the integration of COMAU & TUM at the end of HR-Recycler,
the underlying ROS-driver has been updated thoroughly, given the
intermediate adjustments from `LMS <http://lms.mech.upatras.gr/>`_
as part of the `SHERLOCK <https://www.sherlock-project.eu/home>`_.

These updated PDL-files have been removed from the ``comau-experimental`` git after version ``0.7``
and are now stored in a dedciated repository `<https://gitlab.lrz.de/lsr-itr-ros/comau-data>`_,

.. note::

  please check the ``comau-experimental`` ``comau-data`` git against cross-compatibility

The containing files are given as,

- ``pdl_tcp_functions`` - **NO HOLD** program with utility functions for the TCP/IP communication (identical to previous versions)
- ``state_server`` - **NO HOLD** program that contains a TCP server for publishing robot's state:

  - Cartesian position
  - joint state
  - sensor-track type
  - digital IN/OUT data
  - robot state

- ``robot_server`` - **NO HOLD** program that contains a TCP server for updating current robot paramters

  - sensor-track controller parameterization
  - digital OUT values
  - TBA after final tests

- ``motion_handler`` - **HOLD** program that contains a TCP server for receiving and executing motion commands

  - Joint / Cartesian trajectories
  - Joint positions ( via MoveFly)
  - sensor-track control commands
  - robot locking / restarting commands


.. warning::

  By the time of writing, the online joint position control could not have been established.
  Instead, the joint position control is limited to joint **trajectory** control only.

.. note::

  For comau-driver versions ``<1.0`` there are some issues to take into mind

   * ``arm1_handler`` about to be replaced to **NO HOLD** with additional ``execution_controller`` as **HOLD** program
   * digital IO controllers from ``hrr_controller >= 0.3`` are missing the required hardware-interface
   * arm1_handler *


In order to start the robot, each of the programs above, needs to be

#. copied to the robot teach-pendant
#. translated to a dedicated ``.cod`` file
#. loaded to the robot ``PROG`` list (MENU->PROG)
#. Restart / Reset all programs and such that they show up a green stauts light
#. In case the robot should not only be read out but also to be controlled via ROS,
   the ``arm1_handler`` needs also to be active, this is achieved by additionally

   #. set the robot to external mode (key on the upper left hand side of the TP5 panel to the middle)
   #. enabling the robot drive
   #. starting the program via the ``START`` button

If all of the programs light up with a green status light as shown below,

.. image:: media/TP5_v1.jpg


you may proceed with the next step.



``hrr_cobot <= 0.1.7`` and ``comau_experimental < 0.7.0``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Extensive explanation available in the wiki `Starting the Comau Robot via ROS <https://wiki.tum.de/display/lsritr/Starting+the+Comau+robot+via+ROS>`_ page.

Load the following files

- ``pdl_tcp_functions`` - **NO HOLD** program with utility functions for the TCP/IP communication
- ``state_server`` - **NO HOLD** program that contains a TCP server for publishing robot's state
- ``motion_server`` - **NO HOLD** program that contains a TCP server for receiving motion commands
- ``motion_handler`` - **HOLD** program that executes the motion commands

Programs that are of type **HOLD** require the robot drive to be active, i.e. these programs will always be stopped if e.g. the emergency stop is triggered.
On the other thand the **NO HOLD** programs handle continous communication with the robot and in case of the ROS-connection will kill the ROS-driver if disabled.

.. image:: media/TP5_v1.jpg



Launch the ``hrr_cobot`` ros-driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note::

  this expects that you have build and sourced the required system configuration according to :ref:`HRR_SYS_SETUP`.

The real robot is to be launched via

.. code-block:: bash

  > roslaunch hrr_cobot_robot hrr_cobot_hw.launch

that allows to set the arguments of the robot setup using various config files, defaulting to the ones in
`hrr_cobot_robot/config <https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/tree/main/hrr_cobot_robot/config>`_
and the arguments from the launch file.
You can always check the available options via:

.. code-block:: bash

  hrr_cobot_robot hrr_cobot_hw.launch --ros-args

and keep in mind that the required arguments are optional as well as they default to empty strings and are not needed to be set to a specific value.




