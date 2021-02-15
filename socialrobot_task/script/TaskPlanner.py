#!/usr/bin/env python

import rospy
import rospkg
import os
import sys
import json
import ConfigParser

# Msg structure for the topic communication
from std_msgs.msg import String
from socialrobot_actionlib.msg import *
from socialrobot_task.srv import *

# Serialization
import cPickle as pickle

class TaskPlanner: 
    def __init__(self):
        rospy.loginfo("Initializing the Planner Class....")

        # init path parameters
        self.LIB_PATH = ''
        self.JSON_PATH = ''
        self.PLANNER_PATH = ''
        self.plan_method = '0'

        # get the library path for planner
        if rospy.has_param('/task_plan/json_path'):
            self.JSON_PATH = rospy.get_param('/task_plan/json_path')
            self.PLANNER_PATH = rospy.get_param('/task_plan/planner_path')
            self.LIB_PATH = rospy.get_param('/task_plan/lib_path')
        else:
            rospy.logwarn('cannot find task planner parameters.')
            rospack = rospkg.RosPack()
            PKG_PATH = rospack.get_path('socialrobot_task')
            self.LIB_PATH = PKG_PATH + '/libs'
            self.JSON_PATH = PKG_PATH + '/data/plan.json'
            self.PLANNER_PATH = PKG_PATH + '/data/pddl4j-3.8.2.jar'
        sys.path.append(self.LIB_PATH)        
        globals()['adaptator'] = __import__('AdaptatorPlanJsonPython')
    
    def _json2msg(self, sequentialPlan):
		'''
		Convert JSON plan into ROS msg
		'''
		res = GetActionSeqResponse()
		res.header.stamp = rospy.Time.now()
		res.plan_result = GetActionSeqResponse.SUCCESS

		# print the action name and all the parameters
		for json_action in sequentialPlan.actions():
			action = Action()
			action.name = json_action._get_name()

			#action params
			for parameter in json_action._get_parameters():
				action.parameters.append(parameter)

			# get preconditions of an action
			preconds = json_action._get_preconditions()
			self._get_preconditions(action, preconds)

			# get effects of an action
			cond_express = json_action._get_cond_expressions()
			effects = cond_express[0]._get_effects()	
			self._get_effects(action, effects)
			
			res.action_sequence.append(action)

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

    # Planning function
    def resolvProblem(self, res, domain_path, problem_path):
        # initialize
        if(os.path.isfile(self.JSON_PATH)):
            file = open(self.JSON_PATH,"r+")
            file.truncate(0)
            file.close()

        # planning
        if(os.path.isfile(problem_path) and os.path.isfile(domain_path)):
            javaCommand = "java -jar " + self.PLANNER_PATH + " -o " + \
                domain_path + " -f " + problem_path + " -json " + self.JSON_PATH + " -p " + self.plan_method
        else:
            javaCommand = "Error: PDDL model Files not found..."
            return 

        print("javaCommand : " + javaCommand)

        # Launch the java command
        # use the .jar file giving him the problem, the domain and the path
        # to the json file to create/edit
        os.system(javaCommand)
        try:
            sequentialPlan = adaptator.getSequentialPlanFromJson(self.JSON_PATH)
        except Exception as e:
            print("JSON file could not be parsed\nProblem in the resolution")
            return 

        # read plan from json        
        return self._json2msg(sequentialPlan)


	# def _readData(self, data):
	# 	'''
	# 	read plan from JSON 
	# 	'''
	# 	action_sequence = GetActionSeqResponse()

	# 	serializedJsonPythonObject = data.jsonObject
	# 	operationStatus = data.planStatus
	# 	problemResolved = data.problemResolved

	# 	#If the problem has been resolved
	# 	if problemResolved == True:
	# 		#we generate the object using cPickles
	# 		print("Generation of the json object from the serialized version...")
	# 		sequentialPlan = pickle.loads(serializedJsonPythonObject)
	# 		print("Object generated...")

	# 		# convert into the msg
	# 		action_sequence = self._json2msg(sequentialPlan)
	# 	else:
	# 		#Else we look after the operationStatus to know what went wrong
	# 		if operationStatus == "fileNotFound":
	# 			print("The problem could not been solved. Because the files could not been found...")
	# 		else:
	# 			print("Something went wrong and the problem could not been solved")
	# 		action_sequence.plan_result = 0

		# return action_sequence