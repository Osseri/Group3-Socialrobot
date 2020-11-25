#! /usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from sensor_msgs.msg import JointState


class ArmAction(object):
    # create messages that are used to publish feedback/result
    _feedback = FollowJointTrajectoryFeedback()
    _result = FollowJointTrajectoryResult()

    def __init__(self):
        self._as = actionlib.SimpleActionServer('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        print goal
        

        self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('arm_controller_server')
    server = ArmAction()
    rospy.spin()
