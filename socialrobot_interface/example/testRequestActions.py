#!/usr/bin/env python

import sys
import cPickle as pickle
import rospy
import rospkg
from std_msgs.msg import String
import socialrobot_interface.msg
import socialrobot_interface.srv

import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy

def client():

	getDomain = rospy.ServiceProxy('/socialrobot_interface/actionlib/get_domain', socialrobot_interface.srv.Actionlib)
	requestActions = rospy.ServiceProxy('/socialrobot_interface/actionlib/get_action_list', socialrobot_interface.srv.Actionlib)

	req = socialrobot_interface.srv.ActionlibRequest()
	#set hardware
	req.inputs = ['Arm', 'Gripper']
	res = getDomain(req)

	#get action list
	res = requestActions(req)
	print res


if __name__ == '__main__':
	#Initialize a ROS node
	rospy.init_node("testActionlib")
	#Ask to resolv the mojito.pddl problem of the barman/domain.pddl domain
	client()
