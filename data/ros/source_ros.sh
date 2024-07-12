#!/usr/bin/env shell
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