#!/usr/bin/env python
import rospy
from demo.msg import Person

def callback(data):
    rospy.loginfo("objName: %s" % data.model)
    

def listener():
    rospy.init_node('custom_listener')
    rospy.Subscriber("affordance_info", Person, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
