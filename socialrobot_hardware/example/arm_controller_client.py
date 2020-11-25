#! /usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import *
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from sensor_msgs.msg import JointState


def action_client():
    client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    client.wait_for_server()

    goal = control_msgs.msg.FollowJointTrajectoryActionGoal()    
    print goal
    client.send_goal(goal.goal)


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('arm_control_test')
        result = action_client()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")