#!/usr/bin/env python

import sys
import cPickle as pickle
import rospy
import rospkg
import rosparam
from std_msgs.msg import String

from socialrobot_task_planner.srv import *
from socialrobot_task_msgs.srv import *
from socialrobot_task_msgs.msg import *
# import task planner modules
from ProblemGenerator import ProblemGenerator
from DomainGenerator import DomainGenerator
from TaskPlanner import TaskPlanner

ros_root = rospkg.get_ros_root()
r = rospkg.RosPack()
path = r.get_path('socialrobot_task_planner')
sys.path.append(path + '/libs')

DOMAIN_PATH = path + '/pddl/domain.pddl'
PROBLEM_PATH = path + '/pddl/problem.pddl'

class TaskManager:
	'''
	Task manager
	'''

	def __init__(self):
		'''
		Initialize a ROS node
		'''
		# ROS services
		rospy.Service('/task_plan/get_action_sequence', GetActionSeq, self._callback_get_action_sequence)

		rospy.loginfo('[TaskManager] Service Started!')

		self.action_sequence = ActionSequence()
		self.tp = TaskPlanner()
		self.pg = ProblemGenerator()
		self.dg = DomainGenerator()
		

	def __del__(self):
		'''

		'''
		rospy.loginfo('[TaskManager] Service terminated!')

	def _get_domain_name(self):
		domain_name = ''
		if rospy.has_param('/robot_name'):
			domain_name = rospy.get_param('/robot_name')        
		return domain_name

	def _callback_get_action_sequence(self, req):
		'''
		callback for service
		'''
		res = GetActionSeqResponse()
		domain = self._get_domain_name()
		# generate PDDL domain & problem files
		if not domain == '':
			# write PROBLEM.pddl
			self.pg.generate(domain)
			# write DOMAIN.pddl
			self.dg.generate(domain)
		else:
			rospy.logerr("Can't load robot description")

		# call the task planner
		print("Waiting for the pddl planning service...")
		rospy.wait_for_service('/task_plan/pddl_plan')
		try:
			print("Connection to the planner server...")
			problemSolver = rospy.ServiceProxy('/task_plan/pddl_plan', Planning)

			print("Connection OK, generating the solution...")
			request = PlanningRequest()
			request.problemPath = PROBLEM_PATH
			request.domainPath = DOMAIN_PATH

			plan = problemSolver(request)
			res = self._readData(plan)

			self.action_sequence = res.action_sequence
			return res

		except Exception as e:
			print("Service call failed : %s"%e)

			return res

	def _readData(self, data):
		'''
		read plan from JSON 
		'''
		action_sequence = GetActionSeqResponse()

		serializedJsonPythonObject = data.jsonObject
		operationStatus = data.planStatus
		problemResolved = data.problemResolved

		#If the problem has been resolved
		if problemResolved == True:
			#we generate the object using cPickles
			print("Generation of the json object from the serialized version...")
			sequentialPlan = pickle.loads(serializedJsonPythonObject)
			print("Object generated...")

			# convert into the msg
			action_sequence = self._json2msg(sequentialPlan)
		else:
			#Else we look after the operationStatus to know what went wrong
			if operationStatus == "fileNotFound":
				print("The problem could not been solved. Because the files could not been found...")
			else:
				print("Something went wrong and the problem could not been solved")
			action_sequence.plan_result = 0

		return action_sequence

	def _json2msg(self, sequentialPlan):
		'''
		Convert JSON plan into ROS msg
		'''
		res = GetActionSeqResponse()
		res.header.stamp = rospy.Time.now()
		res.plan_result = 1

		# print the action name and all the parameters
		for json_action in sequentialPlan.actions():
			res.action_sequence.actions
			
			action = Action()
			action.name = json_action._get_name()

			for parameter in json_action._get_parameters():
				action.parameters.append(parameter)

			# get preconditions of an action
			# TODO: modify ROS action msg model
			preconds = json_action._get_preconditions()
			action = self._get_preconditions(action, preconds)

			# get effects of an action
			cond_express = json_action._get_cond_expressions()
			effects = cond_express[0]._get_effects()	
			action = self._get_effects(action, effects)
			res.action_sequence.actions.append(action)

		return res

	def _get_preconditions(self, action, conditions):
		'''
		get fluents of action
		'''
		for neg in conditions._get_negatives():
			negative = Fluent()
			negative.predicate = neg._predicat
			for arg in neg._get_list_arg():
				negative.args.append(arg)
			action.precondition.negatives.append(negative)
		for pos in conditions._get_positives():
			positive = Fluent()
			positive.predicate = pos._predicat
			for arg in pos._get_list_arg():
				positive.args.append(arg)
			action.precondition.positives.append(positive)

		return action

	def _get_effects(self, action, conditions):
		'''
		get fluents of action
		'''
		for neg in conditions._get_negatives():
			negative = Fluent()
			negative.predicate = neg._predicat
			for arg in neg._get_list_arg():
				negative.args.append(arg)
			action.effect.negatives.append(negative)
		for pos in conditions._get_positives():
			positive = Fluent()
			positive.predicate = pos._predicat
			for arg in pos._get_list_arg():
				positive.args.append(arg)
			action.effect.positives.append(positive)

		return action

if __name__ == '__main__':

	# ros initialize
    rospy.init_node("task_manager")

    # Task manager
    tm = TaskManager()

    rospy.spin()