if [ -n "$ZSH_VERSION" ]; then
	SOURCE_ENDING=".zsh"
else
	SOURCE_ENDING=".sh"
fi

function setrosmaster {
        if [ -z $1 ]
        then
                master=localhost
        else
                master=$1
        fi
        export ROS_MASTER_URI=http://$master:11311
        export | grep ROS_MASTER_URI
}


function setgazebomaster {
        if [ -z $1 ]
        then
                master=localhost
        else
                master=$1
        fi
        export GAZEBO_MASTER_URI=http://$master:12343
        export | grep GAZEBO_MASTER_URI
}


function sourceROS {
		if [ -z ${1} ]; then
			echo " => source a ros workspace. Either provide full path to workspace or setup${SOURCE_ENDING}";
			return
		fi
        if [ -f "${1}/devel/setup${SOURCE_ENDING}" ];then
            source "${1}/devel/setup${SOURCE_ENDING}"
            echo "sourcing ROS workspace ${1}"
        elif [ -f "${1}devel/setup${SOURCE_ENDING}" ];then
            source "${1}devel/setup${SOURCE_ENDING}"
            echo "sourcing ROS workspace ${1}"
        elif [ -f "${1}" ];then
            source "${1}"
            echo "sourcing ROS workspace ${1}"
        else
            echo "skipping ROS workspace ${1} - not built!"
        fi
}

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
