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

		#print('--------Task planning Result--------------')
		#print(res)

	except Exception as e:
		rospy.logerr("Service call failed : %s"%e)

	return res.action_sequence

def current_states():
	predicates = []
	pred = Predicate()
	pred.name = 'openedContainer'
	pred.args = ['obj_fridge']
	pred.is_negative = False
	predicates.append(pred)
	return predicates

if __name__ == '__main__':
	#Initialize a ROS node
	rospy.init_node("testPlanner")
	decodeAction = rospy.ServiceProxy('/actionlib/decode_action', GetPrimitiveActionList)

	#Set problem
	predicate = load_problem("fridge")
	problem = generate_problem(predicate)
	current_states = '' #current_states()

	print('\n=======current predicates========')
	for p in current_states:
		print(p.name, p.args)

	if set_problem(problem):
		#Get action sequence
		compound_action_seq = run()
		primitive_action_seq = []
		print('\n=======compound action========')
		for idx, action in enumerate(compound_action_seq):
			act_list = []
			act_list.append(action.name)
			for param in action.values:
				act_list.append(param)
			print(idx, act_list)

			#action decoding
			req_act = GetPrimitiveActionListRequest()
			req_act.compound_action = action
			req_act.current_states = current_states
			res = decodeAction(req_act)
			for act in res.primitive_action:
				primitive_action_seq.append(act)

		print('\n=======primitive action========')
		for idx, action in enumerate(primitive_action_seq):
			act_list = []
			act_list.append(action.name)
			for param in action.values:
				act_list.append(param)
			print(idx, act_list)
	else:
		print('Task planing is failed')