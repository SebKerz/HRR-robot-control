#!/usr/bin/env python
"""
Scripted Rosbag Recorder
--------------------------

This file allows to generate `rosbag <https://pointclouds.org/>`_ data automatically from script.

Maybe of interest:

* `bagpy <https://pypi.org/project/bagpy>`_
"""


from pathlib import Path
import os
import signal
import subprocess
from datetime import datetime

import numpy as np

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, Int8MultiArray


__all__ = ["RecordData", "start_ros_bag", "stop_ros_bag", "goodbye"]


def now_str():
    return datetime.now().strftime('%Y-%m-%d_%H:%M:%S')


def goodbye():
    print(f"Shutting down node {rospy.get_name()} on {rospy.get_node_uri()}")


class RecordData(object):

    def __init__(self, base_dir, base_bag_name):
        self.base_dir = Path(base_dir)
        self.base_bag_name = base_bag_name
        self.cnt = 0
        self.process_handle = None
        self.rec_start = None

    @property
    def rosbag_node_name(self):
        # type: () -> str
        try:
            return f"{self.base_bag_name}{self.cnt}"
        except:
            return "{}{}".format(self.base_bag_name, self.cnt)

    @property
    def rosbag_cmd(self):
        # type: () -> str
        base_file = f"{now_str()}_{self.base_bag_name}_{self.cnt}"
        cmd = f"rosbag record -a -O {self.base_dir / base_file}  __name:={self.rosbag_node_name}"
        return cmd

    @property
    def recording(self):
        # type: () -> bool
        return self.process_handle is not None


def start_ros_bag(rec_data):
    # type: (RecordData) -> None
    if rec_data.process_handle is not None or rec_data.rec_start is not None:
        rospy.logerr("cannot start rosbag. Previous recording still ongoing!")
        return

    rec_data.process_handle = subprocess.Popen(rec_data.rosbag_cmd, stdout=subprocess.PIPE, shell=True)
    rec_data.rec_start = rospy.get_time()
    rospy.logdebug("starting recording {}".format(rec_data.rosbag_node_name))


def stop_ros_bag(rec_data, t_min=1.0):
    # type: (RecordData, float) -> bool
    if rec_data.process_handle is None or rec_data.rec_start is None:
        return False
    elif rospy.get_time() - rec_data.rec_start < t_min:
        rospy.logwarn("minimum time of recording has not yet been reached. continue recording")
        return False
    node_name = rec_data.rosbag_node_name
    subprocess.Popen("rosnode kill {}".format(node_name), stdout=subprocess.PIPE, shell=True)
    rospy.logdebug("sopped recording {}".format(node_name))
    rec_data.rec_start = None
    rec_data.process_handle = None
    rec_data.cnt += 1
    return True
