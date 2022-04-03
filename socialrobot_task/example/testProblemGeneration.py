#!/usr/bin/env python

import rospy
import rospkg
from std_msgs.msg import String
import diagnostic_msgs.msg

from socialrobot_task.srv import *
from socialrobot_actionlib.msg import *
from load_problem import load_problem

def createProblem():
	
	problem = Problem()
	problem.problem_name = "pick_and_place"
	problem.domain_name = "socialrobot"

	# add instances
	#add_object(problem, "", "")
	add_object(problem, "left_hand", "Object")
	add_object(problem, "right_hand", "Object")
	add_object(problem, "juice", "Object")
	add_object(problem, "water", "Object")
	add_object(problem, "milk", "Object")
	add_object(problem, "table", "Object")
	add_object(problem, "pos_juiceInit", "Position")
	add_object(problem, "pos_milkInit", "Position")
	add_object(problem, "posWaterInit", "Position")
	add_object(problem, "pos_leftArmInit", "Position")
	add_object(problem, "pos_rightArmInit", "Position")
	add_object(problem, "temp_position", "Position")


	# add attributes
	add_fact(problem, "type", ["left_hand", "Gripper"])
	add_fact(problem, "type", ["right_hand", "Gripper"])
	add_fact(problem, "inWorkspace", ["left_hand", "TempPosition"])

	add_fact(problem, "onPhysical", ["juice", "table"])
	add_fact(problem, "onPhysical", ["milk", "table"])
	add_fact(problem, "onPhysical", ["water", "table"])

	add_fact(problem, "emptyHand", ["left_hand"])
	add_fact(problem, "emptyHand", ["right_hand"])

	add_fact(problem, "locatedAt", ["milk", "pos_milkInit"])
	add_fact(problem, "locatedAt", ["juice", "pos_juiceInit"])
	add_fact(problem, "locatedAt", ["water", "posWaterInit"])
	add_fact(problem, "locatedAt", ["left_hand", "pos_leftArmInit"])
	add_fact(problem, "locatedAt", ["right_hand", "pos_rightArmInit"])

	add_fact(problem, "detected", ["milk"])
	add_fact(problem, "detected", ["juice"])
	add_fact(problem, "detected", ["water"])

	add_fact(problem, "obstruct", ["left_hand", "milk", "juice"])
	add_fact(problem, "inWorkspace", ["left_hand", "pos_juiceInit"])
	add_fact(problem, "inWorkspace", ["left_hand", "pos_milkInit"])
	add_fact(problem, "inWorkspace", ["left_hand", "posWaterInit"])

	# add goals
	add_goal(problem, "graspedBy", ["left_hand", "milk"])

	return problem

def add_object(problem, instance_name, obj_type):
	msg = problem
	key_value = diagnostic_msgs.msg.KeyValue()
	key_value.key = obj_type		#object type
	key_value.value = instance_name	#object arguments array
	msg.objects.append(key_value)   
	return msg

def add_fact(problem, name, args, is_negative = False):
	msg = problem
	predicate = Predicate()
	predicate.name = name			#preicate name
	predicate.args = args			#predicate arguments
	predicate.is_negative = is_negative
	msg.facts.append(predicate)   
	return msg

def add_goal(problem, name, args, is_negative = False):
	msg = problem
	predicate = Predicate()
	predicate.name = name			#preicate name
	predicate.args = args			#predicate arguments
	predicate.is_negative = is_negative
	msg.goals.append(predicate)   
	return msg

def set_problem(problem):
	# service
	domain_srv = rospy.ServiceProxy('/task_plan/set_problem', SetProblem)
	return domain_srv(problem)
	
def generate_problem(predicate):	
	problem = Problem()
	problem.problem_name = predicate['Demo']['problem']
	problem.domain_name = predicate['Demo']['domain']

	# add instances
	for obj in predicate['Demo']['object']:
		if len(obj)>2:
			add_object(problem, obj[0], obj[1], True)
		else:
			add_object(problem, obj[0], obj[1])

	# add attributes
	for obj in predicate['Demo']['fact']:
		if len(obj)>2:
			add_fact(problem, obj[0], obj[1], True)
		else:
			add_fact(problem, obj[0], obj[1])

	# add goals
	for obj in predicate['Demo']['goal']:
		if obj[0] == 'not':
			add_goal(problem, obj[1][0], obj[1][1], True)
		else:
			add_goal(problem, obj[0], obj[1])

	return problem

if __name__ == '__main__':
	#Initialize a ROS node
	rospy.init_node("testProblemGenerator")

	#generate from script
	predicate = load_problem("default")
	problem = generate_problem(predicate)

	#generate manually
	# problem = createProblem()

	print set_problem(problem)