#!/usr/bin/env shell
function set_lsr_ros_ip(){
    IP=$(ip a | grep 129| awk {'print $2'})
    delimiter="/"
    export ROS_IP="${IP%%"$delimiter"*}"
    export | grep ROS_IP
}


function hr_recycler_rosify(){
    export ROSLAUNCH_SSH_UNKNOWN=1
    auto_init_ROS
    setrosmaster  ${HRR_ROS_MASTER}
    setgazebomaster ${HRR_ROS_MASTER}
    sourceROS ${HRR_CATKIN_WS}
    sourceROS ~/_ros/multimaster_ws/devel/setup.zsh
    export ROS_IP="${HR_RECYCLER_IP}"
    export | grep ROS_IP
    condafy;
    conda activate hrr

}


function hrr_rosify(){
    export ROSLAUNCH_SSH_UNKNOWN=1
    auto_init_ROS
    setrosmaster  ${HRR_ROS_MASTER}
    setgazebomaster ${HRR_ROS_MASTER}
    sourceROS ${HRR_CATKIN_WS}
    sourceROS ~/_ros/multimaster_ws/devel/setup.zsh
    set_lsr_ros_ip;
    condafy;
    conda activate hrr
}




function hrr_rosify_sim(){
    auto_init_ROS
    sourceROS ${HRR_CATKIN_WS}
    condafy;
    conda activate hrr
}

function hrr_notebook(){
    auto_init_ROS
    sourceROS ${HRR_CATKIN_WS}
    condafy;
    jupyter notebook "${@}"
}

function hrr_hw_notebook(){
    hrr_rosify;
    conda deactivate;
    jupyter notebook "${@}"
}
