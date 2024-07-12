#!/usr/bin/env shell

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