#!/usr/bin/env python

import sys
import cPickle as pickle
import rospy
import rospkg
from std_msgs.msg import String

from socialrobot_task_planner.srv import *
from socialrobot_task_msgs.srv import *
from socialrobot_task_msgs.msg import *

sys.path.append('/home/rise-workstation//Workspace/ROS/social_ws/src/socialrobot/src/socialrobot_task/socialrobot_task_planner/libs')

DOMAIN_PATH = '/home/rise-workstation/Workspace/ROS/social_ws/src/socialrobot/src/socialrobot_task/socialrobot_task_planner/pddl/domain.pddl'
PROBLEM_PATH = '/home/rise-workstation/Workspace/ROS/social_ws/src/socialrobot/src/socialrobot_task/socialrobot_task_planner/pddl/problem.pddl'

#using the service of the Planner node
#it will solve the problem synchronously
def client():
	print("Waiting for the service...")
	rospy.wait_for_service('/task_plan/get_action_sequence')
	try:
		print("Connection to the planner server...")
		requestActions = rospy.ServiceProxy('/task_plan/get_action_sequence', GetActionSeq)

		print("Connection successfull, generating of the solution...")
		request = "test"

		response = requestActions(request)
		for idx, action in enumerate(response.action_sequence.actions):
			act_list = []
			act_list.append(action.name)
			for param in action.parameters:
				act_list.append(param)
			print idx, act_list

	except Exception as e:
		print("Service call failed : %s"%e)


if __name__ == '__main__':
	#Initialize a ROS node
	rospy.init_node("testPlanner")
	#Ask to resolv the mojito.pddl problem of the barman/domain.pddl domain
	client()
