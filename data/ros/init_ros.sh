#!/usr/bin/env shell
function initROS(){
	case ${1} in
	 "indigo")
		source "/opt/ros/indigo/setup${SOURCE_ENDING}"
		export ROS_PYTHON_VERSION=2
		;;
	 "kinetic")
		source "/opt/ros/kinetic/setup${SOURCE_ENDING}"
		export ROS_PYTHON_VERSION=2
		;;
	 "melodic")
		source "/opt/ros/melodic/setup${SOURCE_ENDING}"
		;;
     "noetic")
		source "/opt/ros/noetic/setup${SOURCE_ENDING}"
		;;
	 "source")
		source "/opt/ros/src_build/setup${SOURCE_ENDING}"
		;;
	 *)
       echo "unkown option ${1} source ROS base: "
	   echo " indigo | kinetic | melodic | noetic | source"
       return;
	;;
esac
 export | grep ROS_DISTRO
}
