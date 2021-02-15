#!/usr/bin/env python
import rospy
from rosjava_custom_srv.msg import *
import time

def talker():
    pub = rospy.Publisher('/affordance_info', Person, queue_size=10)
    rospy.init_node('custom_talker')
    #r = rospy.Rate(10) #10hz
    affordance = ["contaion"]
    affordance_confidence = ["0.949"]

    msg = Person()
    msg.objName = ["cup121","cup276","cup308","cup449"]
    msg.objConfidence = ["0.321","0.310","0.355","0.995"]
    msg.objPose.header.seq = 0302
    msg.objPose.header.stamp.secs = 92254
    msg.objPose.header.stamp.nsecs = 23091203
    msg.objPose.header.frame_id = "cup449"
    msg.objPose.pose.position.x = -2.1250005
    msg.objPose.pose.position.y = 0.2334325
    msg.objPose.pose.position.z = 5.342144
    msg.objPose.pose.orientation.x = 90
    msg.objPose.pose.orientation.y = 0
    msg.objPose.pose.orientation.z = 90
    msg.objPose.pose.orientation.w = 0
    msg.bboxMin.x = 3.112
    msg.bboxMin.y = 5.42
    msg.bboxMin.z = -2.34
    msg.bboxMax.x = 7.878
    msg.bboxMax.y = 9.123
    msg.bboxMax.z = 10.23
    msg.model = "AffordanceNet_v3"
    msg.affordanceName = affordance
    msg.affordanceConfidence = affordance_confidence
 
    time.sleep(5)
   
    i = 0
    while i < 20:
        pub.publish(msg)
        i = i+1
        print "send"
    time.sleep(5)
    affordance.append('h-grasp')
    affordance_confidence.append('0.945')

    msg.affordanceName = affordance
    msg.affordanceConfidence = affordance_confidence
    pub.publish(msg)

    i = 0
    while i < 20:
        pub.publish(msg)
        i = i+1
        print "send2"
    time.sleep(5)

    affordance.append('side-grasp')
    affordance_confidence.append('0.932')

    msg.affordanceName = affordance
    msg.affordanceConfidence = affordance_confidence

    i = 0
    while i < 20:
        pub.publish(msg)
        i = i+1
        print "send3"
    time.sleep(10)


'''
    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()
'''
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
