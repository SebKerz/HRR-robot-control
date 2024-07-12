#!/usr/bin/env shell
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