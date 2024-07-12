Bugs / Troubleshooting
^^^^^^^^^^^^^^^^^^^^^^^^

Below we outline some common trouble-shootings or issues that arise when working with the robot.

Launch file results in a flood of error-messages
""""""""""""""""""""""""""""""""""""""""""""""""""""

After launching the ROS-driver the terminal is flooded by red error-messages that the latest message
could not been read properly.

This problem is annoying but can only be resolved by the usual fix-em all

.. image:: https://sd.keepcalms.com/i/have-you-tried-turning-it-off-and-on-again-3.png

Robot in `terminate` state
"""""""""""""""""""""""""""

This is reported in the  `Wiki-> robot in terminate state <https://wiki.tum.de/display/lsritr/Robot+in+%27terminate%27+state>`_.
In short:

#. disable robot drive
#. select all ``HOLD`` programs, e.g. ``motion_handler``
#. restart by PROGRAM-> RESTART
#. enable drive
#. Press START button

Continue by enabling ROS-controllers in ROS.

Robot in `69` state
""""""""""""""""""""""""

Actual bug in the code, appearing if you started a controller that intends to _write_ data to the robot
before it is properly reset.
So in general:

#. Enable Robot
#. Enable drive
#. Start Ros-launch
#. Test connection

Is the preferred sequence.
If you end in this state, you will have to restart the robot and the ROS-launch / driver on the real-time PC.
Congratulations you fool.

