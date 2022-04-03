#!/usr/bin/env python

import sys
import cPickle as pickle
import rospy
import rospkg
from std_msgs.msg import String
import socialrobot_interface.msg
import socialrobot_interface.srv

def client():
	print("Waiting for the service...")
	rospy.wait_for_service('/socialrobot_interface/task/get_plan')
	try:
		print("Connection to the planner server...")
		requestActions = rospy.ServiceProxy('/socialrobot_interface/task/get_plan', socialrobot_interface.srv.Task)

		print("Connection successfull, waiting for response")
		req = socialrobot_interface.srv.TaskRequest()
		req.command = "GET"
		req.target = "PLAN"

		res = requestActions(req)
		print(res)

	except Exception as e:
		print("Service call failed : %s"%e)


if __name__ == '__main__':
	#Initialize a ROS node
	rospy.init_node("testPlanner")
	#Ask to resolv the mojito.pddl problem of the barman/domain.pddl domain
	client()
