#!/usr/bin/env python
import rospy
from socialrobot_task.srv import *
from socialrobot_actionlib.srv import *
from socialrobot_actionlib.msg import *
from testProblemGeneration import generate_problem,set_problem
from load_problem import load_problem

def run():
	rospy.wait_for_service('/task_plan/get_action_sequence')
	try:
		plan_srv = rospy.ServiceProxy('/task_plan/get_action_sequence', GetActionSeq)

		req = GetActionSeqRequest()
		res = plan_srv(req)	

	except Exception as e:
		rospy.logerr("Service call failed : %s"%e)

	return res.action_sequence

if __name__ == '__main__':
	#Initialize a ROS node
	rospy.init_node("testPlanner")
	decodeAction = rospy.ServiceProxy('/actionlib/decode_action', GetPrimitiveActionList)

	#Set problem
	predicate = load_problem("default")
	problem = generate_problem(predicate)
	
	if set_problem(problem):
		#Get action sequence
		compound_action_seq = run()
		primitive_action_seq = []
		print '\n=======compound action========'
		for idx, action in enumerate(compound_action_seq):
			act_list = []
			act_list.append(action.name)
			for param in action.parameters:
				act_list.append(param)
			print idx, act_list

			#action decoding
			res = decodeAction(action)
			for act in res.primitive_action:
				primitive_action_seq.append(act)

		print '\n=======primitive action========'
		for idx, action in enumerate(primitive_action_seq):
			act_list = []
			act_list.append(action.name)
			for param in action.parameters:
				act_list.append(param)
			print idx, act_list
	else:
		print 'Task planing is failed'