#!/usr/bin/env python

import sys
import cPickle as pickle
import rospy
import rospkg
from std_msgs.msg import String

sys.path.append('/home/rise-workstation//Workspace/ROS/social_ws/src/socialrobot/src/socialrobot_task/socialrobot_task_planner/libs')

from socialrobot_task_planner.srv import *

DOMAIN_PATH = '/home/rise-workstation/Workspace/ROS/social_ws/src/socialrobot/src/socialrobot_task/socialrobot_task_planner/pddl/domain.pddl'
PROBLEM_PATH = '/home/rise-workstation/Workspace/ROS/social_ws/src/socialrobot/src/socialrobot_task/socialrobot_task_planner/pddl/problem.pddl'

#using the service of the Planner node
#it will solve the problem synchronously
def client():
	print("Waiting for the service...")
	rospy.wait_for_service('/task_plan/pddl_plan')
	try:
		print("Connection to the planner server...")
		problemSolver = rospy.ServiceProxy('/task_plan/pddl_plan', Planning)

		print("Connection successfull, generation of the solution...")
		request = PlanningRequest()
		request.problemPath = PROBLEM_PATH
		request.domainPath = DOMAIN_PATH

		response = problemSolver(request)
		readData(response)

	except Exception as e:
		print("Service call failed : %s"%e)


def readData(req):
	#The message received is formated like this :
	#PathToThePlanAsJsonFile___problemResolved___pathToTheDomainFile/NameOfTheExempleDirectory
	#___pathToTheProblemFile/NameOfTheExempleProblem___SerializedVersionOfThePlanAsAnObject___OperationStatus

	#For exemple if we ask to solve the problem mojito.pddl in the barman directory (from the exemple directory)
	#pddl4j_rospy/src/jsonFiles/plan.json___True___barman___mojito___OBJECT-SERIALIZED___Ok

	jsonPath = req.jsonPath
	serializedJsonPythonObject = req.jsonObject
	operationStatus = req.planStatus
	problemResolved = req.problemResolved

	#If the problem has been resolved
	if problemResolved == True:
		#we generate the object using cPickles
		print("Generation of the json object from the serialized version...")
		sequentialPlan = pickle.loads(serializedJsonPythonObject)
		print("Object generated...")
		#And we display the actions of the plan
		displayAction(sequentialPlan)
	else:
		#Else we look after the operationStatus to know what went wrong
		if operationStatus == "fileNotFound":
			print("The problem could not been solved. Because the files could not been found...")
		else:
			print("Something went wrong and the problem could not been solved")


def displayAction(sequentialPlan):
	#printing the action name and all the parameters
	i=0
	for action in sequentialPlan.actions():
		print("action " + str(i) + ":" + action._get_name()),
		for parameter in action._get_parameters():
			print(parameter),
		print("\n"),
		i+=1

if __name__ == '__main__':
	#Initialize a ROS node
	rospy.init_node("testPlanner")
	#Ask to resolv the mojito.pddl problem of the barman/domain.pddl domain
	client()
