#!/usr/bin/env python
PKG='hrr_cobot_robot'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

from ros_interfaces import SnsTest, FTTest
import unittest



if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'ft_test', FTTest)
    rosunit.unitrun(PKG, 'sns_test', SnsTest)
    