#!/usr/bin/env python
PKG = 'hrr_cobot_robot'
import roslib; roslib.load_manifest(PKG)

from hrr_cobot_robot.ros_interfaces import (
    JointTrajectoryHandler,
    ComauRobotState, RobotState,
    FTData, FTBuffer,
    SnsTrkComplCmd, SnsTrkCmd
)
    
    
import rospy
import unittest


## A sample python unit test
class SnsTest(unittest.TestCase):

    def test_init_sns_cmd(self):
        sns = SnsTrkCmd()
        self.assertTrue(sns._pub_sns_trk_twist is None, "sensor-track publisher is None")
        self.assertTrue(sns._pub_sns_frame is None,  "sensor-track publisher is None")
        self.assertTrue(sns._viz_pub is None,  "sensor-track vsiualizer is None")

    @staticmethod
    def load_test_params():
        rospy.set_param("~sns_trk_twist_topic_name", "dummy_sns1")
        rospy.set_param("~sns_trk_topic_name", "dummy_sns2")
        rospy.set_param("~visualize_topic_name", "dummy_sns3")
        rospy.set_param("~controller_manager_ns", "dummy_controller_manager")
        
    def test_params_sns_cmd(self): # only functions with 'test_'-prefix will be run!
        self.load_test_params()
        self.assertRaises(KeyError, SnsTrkCmd._from_ros, "/false_node/")
        # rospy.set_param("~sns_trk_vel_controller_name", "dummy_controller_name")
        self.assertRaises(KeyError, SnsTrkCmd._from_ros, "~")
        # self.assertFalse(sns2._pub_sns_trk_twist is None)
        # self.assertFalse(sns2._pub_sns_frame is None)
        # self.assertFalse(sns2._viz_pub is None)
        

    def test_ros_params(self):
        self.assertRaises(KeyError, SnsTrkCmd._from_ros)
        self.assertRaises(KeyError, SnsTrkComplCmd._from_ros)



## A sample python unit test
class FTTest(unittest.TestCase):
    
    def test_params(self): # only functions with 'test_'-prefix will be run!
        self.assertRaises(KeyError, FTData._from_ros)
        self.assertRaises(KeyError, FTBuffer._from_ros)



class RosIfTests(unittest.TestSuite):

    def __init__(self):
        super(RosIfTests, self).__init__()
        self.addTest(SnsTest())
        self.addTest(FTTest())

