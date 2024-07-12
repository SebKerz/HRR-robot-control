#!/usr/bin/python3
"""ROS for file-fetishists
author: v.gabler@tum.de
"""
from dataclasses import dataclass
import pathlib
import subprocess
import typing

import rospy
import numpy as np
import yaml


def set_msg_from_dict(data, out):
    for k, v in data.items():
        if isinstance(v, dict):
            python_pointer = [getattr(out, k)]
            set_msg_from_dict(v, python_pointer[0])
        else:
            setattr(out, k, v)


def get_msg_keys(msg):
    return list(filter(lambda x: x[0] != '_' and 'serialize' not in x, msg.__dir__()))


def msg_to_dict(msg, data):
    for k in get_msg_keys(msg):
        v = getattr(msg, k)
        if np.isscalar(v) or isinstance(v, str):
            data[k] = v
        else:
            data[k] = dict()
            msg_to_dict(v, data[k])
            if len(data[k]) == 0:
                del data[k]
    return data


@dataclass(frozen=True)
class ROSFile:
    remote_host: typing.Optional[str] = None  # remote hostname or IP
    remote_user: typing.Optional[str] = None  # remote username
    remote_dir: typing.Optional[pathlib.Path] = None  # remote path
    local_dir: typing.Optional[pathlib.Path] = None  # path to read data from
    file_name: typing.Optional[str] = None  # file name to read from
    tmp_file: typing.Optional[pathlib.Path] = None  # temporary path to dump yaml before sending

    def read_file(self, file_name=None) -> typing.Union[dict, None]:
        if file_name is None:
            file_name = self.file_name if self.local_dir is None else self.local_dir / self.file_name
        if file_name is None:
            rospy.logerr("object has no path to read data from")
        file_name = pathlib.Path(file_name)
        if file_name.exists():
            with open(file_name) as file:
                data = yaml.load(file, Loader=yaml.FullLoader)
            return data
        else:
            rospy.logerr(f"{file_name} does not exist")
            return

    def send_dict(self, data, filename):
        tmp_file = pathlib.Path("/tmp") / f"{filename}.yaml" if self.tmp_file is None else self.tmp_file
        with open(tmp_file, 'w') as file:
            yaml.dump(data, file)
        subprocess.Popen(["scp", tmp_file, f"{self.remote_user}@{self.remote_host}:{self.remote_dir / filename}"])

    def read_msg(self, msg, file_name=None):
        set_msg_from_dict(self.read_file(file_name), msg)
        return msg


if __name__ == "__main__":
    from hr_recycler_msgs.msg import CuttingGoal

    # this assumes that you can ssh to the remote client (hrrcobotLinux54) as the remote user (hrr_cobot) withtout a password
    sender_handle = ROSFile(file_name="test_file", remote_user="hrr_cobot", remote_host="hrrcobotLinux54",
                            remote_dir=pathlib.Path("/home/hrr_cobot/_fake_ros/"),
                            tmp_file=pathlib.Path("dummy_data.yaml"))
    sender_handle.send_dict(sender_handle.read_file(r'./cut_test.msg'), 'test.msg')
    sender_handle.read_msg(CuttingGoal(), './cut_test.msg')
