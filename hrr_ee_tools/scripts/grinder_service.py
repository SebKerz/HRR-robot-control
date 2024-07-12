#!/usr/bin/env python
import grinder_services.msg
import rospy
import actionlib

from std_msgs.msg import Int16

from std_srvs.srv import SetBool

from grinder_services.msg import cutting_actionAction, cutting_actionFeedback, cutting_actionResult



class GrinderServer:
    feedback = cutting_actionFeedback()
    result = cutting_actionResult()

    def __init__(self, name):
        self.action_name = name
        self.server = actionlib.SimpleActionServer(self.action_name, grinder_services.msg.cutting_actionAction, self.execute,
                                                   auto_start=False)
        self.server.start()
        self.grinder_starter = rospy.Publisher('grinder_on', Int16, queue_size=10)




    def execute(self, goal):

        # initialize service proxies. some of them are not used in this first instance of the program
        rospy.wait_for_service('init_grinder')
        initialize = rospy.ServiceProxy('init_grinder', SetBool)
        rospy.wait_for_service('set_bool_grinder')
        switch_on_and_off = rospy.ServiceProxy('set_bool_grinder', SetBool)
        # rospy.wait_for_service('lock_grinder')
        # lock = rospy.ServiceProxy('init_grinder', SetBool)
        #grinder_starter = rospy.Publisher('grinder_on', Int16, queue_size=10)

        #initializing the grinder
        try:
            response_init = initialize(True)
            if not response_init.success:
                rospy.logerr("Impossible to access the grinder.")
                self.result.action_result=False
                self.server.set_succeeded(self.result)

        except rospy.ServiceException:
            rospy.logerr("Impossible to access the grinder. A service exception occurred.")
            raise rospy.ROSInterruptException
        rospy.sleep(1)

        # turn on grinder with desired rpm
        self.grinder_starter.publish(goal.rpm)

        rate = rospy.Rate(1)
        n=0
        while not rospy.is_shutdown():

            #if action is preempted
            if self.server.is_preempt_requested():
                rospy.loginfo('Preempted')
                switch_on_and_off(False)
                self.server.set_preempted()
                self.result.action_result=False
                break

            self.feedback.working = goal.rpm
            self.server.publish_feedback(self.feedback)
            self.result.action_result= True


            n=n+1
            rospy.loginfo(n)
            if (n==30):
                rospy.loginfo('Now the execution should stop')
                self.server.set_succeeded(self.result)
                switch_on_and_off(False)
                break
            rate.sleep()

        rospy.loginfo('we out')

if __name__ == '__main__':
    rospy.init_node('cutting_action')
    server = GrinderServer(rospy.get_name())
    rospy.spin()
