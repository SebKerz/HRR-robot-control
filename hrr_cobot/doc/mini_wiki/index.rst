.. _miniwiki:

COMAU Robot Mini Wiki / Basics
================================



This package configuration expects the following setup

==================================================================   ===============================================================================
repo@tag                                                             notes
==================================================================   =============================================================================== 
https://gitlab.com/VGab/ros-wsg-50/-/tags/pilot                      ROS WSG 50 driver with DSA support
https://gitlab.lrz.de/lsr-itr-ros/ros_jr3/-/tags/pilot               JR3 F/T (ROS-) sensor driver
https://gitlab.lrz.de/lsr-itr-ros/comau-data/-/tags/pilot>           comau-data in use for driver below
https://gitlab.lrz.de/lsr-itr-ros/comau-experimental/-/tags/pilot    ROS driver for COMAU robots
https://gitlab.lrz.de/hr_recycler/hr_recycler_msgs/-/tags/pilot      Hr-Recycler Message git repo
https://gitlab.lrz.de/hr_recycler/hrr_cobot/-/tags/pilot             *this repo*
==================================================================   =============================================================================== 

Beyond the content of this wiki, there are additional pages of interest to be found below:

- `Comau Robot <https://wiki.tum.de/display/lsritr/Comau+robots>`_ defines kinematic basics of the COMAU Racer 5 and manual data
- `Starting the Comau Robot via ROS <https://wiki.tum.de/display/lsritr/Starting+the+Comau+robot+via+ROS>`_ outlines how to start the robot.
- `Wiki-> robot in terminate state <https://wiki.tum.de/display/lsritr/Robot+in+%27terminate%27+state>`_. outlines how to recover the robot from ``terminate`` mode.


.. note::

   please take into account that the links above are only accessible to staff / student member of the TUM-LSR/ITR group.

.. toctree::
   :maxdepth: 2

   system_setup
   robot_startup
   namespace_handling
   ROS_file_overview
   bugs_and_troubleshooting
   pcl

