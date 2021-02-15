#!/usr/bin/env python

import sys
import cPickle as pickle
import rospy
import rospkg
from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
from socialrobot_actionlib.msg import Problem,Predicate
import socialrobot_interface.msg
import socialrobot_interface.srv

def add_object(problem, instance_name, obj_type):
	msg = problem
	key_value = KeyValue()
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

def client():
	print("Waiting for the service...")
	rospy.wait_for_service('/socialrobot/set_command')
	try:
		print("Connection to the planner server...")
		requestActions = rospy.ServiceProxy('/socialrobot/set_command', socialrobot_interface.srv.Task)

		print("Connection successfull, waiting for response")
		req = socialrobot_interface.srv.TaskRequest()
		req.command = ""
		
		req.problem = Problem()
		req.problem.problem_name = 'social_task'
		req.problem.domain_name = rospy.get_param('robot_name')

		add_object(req.problem, "pos_juice", "Position")
		add_object(req.problem, "pos_milk", "Position")
		add_object(req.problem, "pos_right_hand", "Position")
		add_object(req.problem, "pos_left_hand", "Position")
		add_object(req.problem, "pos_table", "Position")
		add_object(req.problem, "obj_socialrobot", "Object")
		add_object(req.problem, "obj_left_hand", "Object")
		add_object(req.problem, "obj_right_hand", "Object")
		add_object(req.problem, "obj_juice", "Object")
		add_object(req.problem, "obj_milk", "Object")
		add_object(req.problem, "obj_table", "Object")


		add_fact(req.problem, "locatedAt", ["obj_left_hand", "pos_left_hand"])
		add_fact(req.problem, "locatedAt", ["obj_right_hand", "pos_right_hand"])
		add_fact(req.problem, "obstruct", ["obj_left_hand", "obj_milk", "obj_juice"])
		add_fact(req.problem, "obstruct", ["obj_right_hand", "obj_milk", "obj_juice"])
		add_fact(req.problem, "inworkspace", ["obj_left_hand", "pos_milk"])
		add_fact(req.problem, "inworkspace", ["obj_left_hand", "pos_juice"])


		add_goal(req.problem, "graspedBy", ["obj_left_hand", "obj_milk"])
		
		res = requestActions(req)
		print res

	except Exception as e:
		print("Service call failed : %s"%e)


if __name__ == '__main__':
	#Initialize a ROS node
	rospy.init_node("testDemo")
	#Ask to resolv the mojito.pddl problem of the barman/domain.pddl domain
	client()
