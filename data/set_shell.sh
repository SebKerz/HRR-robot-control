function source_ros_helpers(){
	source "${KB_SCRIPT_PATH}/ros/auto_init_ros.sh"
	source "${KB_SCRIPT_PATH}/ros/init_ros.sh"
	source "${KB_SCRIPT_PATH}/ros/ros_sourcing.sh"
	source "${KB_SCRIPT_PATH}/ros/set_gazebo_master.sh"
	source "${KB_SCRIPT_PATH}/ros/set_ros_master.sh"
	source "${KB_SCRIPT_PATH}/ros/source_ros.sh"
	source "${KB_SCRIPT_PATH}/ros/hrr_ros.sh"
	source "${KB_SCRIPT_PATH}/ros/source_tokens.sh"
	source "${KB_SCRIPT_PATH}/ros/git_lfs_pull.sh"
	source "${KB_SCRIPT_PATH}/ros/clone_gits.sh"
}

# set shell
if [ -n "$ZSH_VERSION" ]; then
	SOURCE_ENDING=".zsh"
	{ source_ros_helpers } || {
	  echo "failed to load ROS-helper scripts";
       }
else
	SOURCE_ENDING=".sh"
	source_ros_helpers
fi
