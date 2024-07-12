# COMAU PDL workspace 
This repository contains the PDL code used for COMAU ROS interface.

Clone this repository in a windows PC that has ethernet connection with the robot controller.
Translate all four pdl files into cod files and upload them on the C5G controller using the FTP server of the robot controller.

1. `pdl_tcp_functions` - NO HOLD PDL program with utility functions for the TCP/IP communication
2. `state_server` - NO HOLD PDL program that contains a TCP server for publishing robot's state
3. `motion_server` - NO HOLD PDL program that contains a TCP server for receiving motion commands
4. `motion_handler` - HOLD PDL program that executes the motion commands
