#!/usr/bin/env shell

function auto_init_ROS(){
  if lsb_release -r | grep 14 > /dev/null; then
 	ros_dist="indigo";
  elif lsb_release -r | grep 16 > /dev/null; then
    ros_dist="kinetic";
    source /usr/share/gazebo/setup.sh
  elif lsb_release -r | grep 18 > /dev/null; then
 	ros_dist="melodic";
  elif lsb_release -r | grep 20 > /dev/null; then
 	ros_dist="noetic";
  else
    echo "distribution not yet covered by auto-init script";
    return
  fi
  if [ -d "/opt/ros/${ros_dist}" ]; then
    initROS "${ros_dist}"
  else
    echo "ros not installed at default path. please source manually"
  fi
}
