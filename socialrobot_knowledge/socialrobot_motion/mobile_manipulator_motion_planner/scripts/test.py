#! /usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def callback(data):
    rospy.loginfo(data.position[0])

if __name__=='__main__':
    rospy.init_node("anythig")
    rospy.Subscriber('/panda/left_joint_states', JointState, callback)
    rospy.spin()