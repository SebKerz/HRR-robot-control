function add_hr_recycler_msgs(){
    git clone https://${CI_USER}:${HR_RECYCLER_MSGS_CI_TOKEN}@gitlab.lrz.de/hr_recycler/hr_recycler_msgs.git
}

function clone_gits(){
    { add_hr_recycler_msgs } || { }
    git clone https://${CI_USER}:${JR3_CI_TOKEN}@gitlab.lrz.de/lsr-itr-ros/ros_jr3.git
    git clone https://${CI_USER}:${WSG_CI_TOKEN}@gitlab.lrz.de/lsr-itr-ros/ros-wsg-50.git
    git clone https://${CI_USER}:${COMAU_CI_TOKEN}@gitlab.lrz.de/lsr-itr-ros/comau-experimental.git
    git clone https://${CI_USER}:${COMAU_DATA_CI_TOKEN}@gitlab.lrz.de/lsr-itr-ros/comau-data.git
    lfs_pull comau-experimental
    lfs_pull comau-data
}