#!/usr/bin/env python

import rospy
import rospkg
import os
import sys
import json
import ConfigParser

# Msg structure for the topic communication
from std_msgs.msg import String
from socialrobot_task_planner.srv import *

# Serialization
import cPickle as pickle

class TaskPlanner:    

    def __init__(self, sync="sync"):
        '''
		Initialize the planner by launching the server 
		:param: string sync: "sync" or "async" 
	    :return: void
		'''
        rospy.loginfo("Initializing the Planner Class....")

        # init path parameters
        self.LIB_PATH = ''
        self.JSON_PATH = ''
        self.PLANNER_PATH = ''

        # set comm mode
        self.sync = sync

        # get the library path for planner
        if rospy.has_param('/task_plan/json_path'):
            print('find parameters.')
            self.JSON_PATH = rospy.get_param('/task_plan/json_path')
            self.PLANNER_PATH = rospy.get_param('/task_plan/planner_path')
            self.LIB_PATH = rospy.get_param('/task_plan/lib_path')
        else:
            rospy.logwarn('cannot find task planner parameters.')
            rospack = rospkg.RosPack()
            PKG_PATH = rospack.get_path('socialrobot_task_planner')
            self.LIB_PATH = PKG_PATH + '/libs'
            self.JSON_PATH = PKG_PATH + '/data/plan.json'
            self.PLANNER_PATH = PKG_PATH + '/data/pddl4j-3.8.2.jar'
        sys.path.append(self.LIB_PATH)        
        globals()['adaptator'] = __import__('AdaptatorPlanJsonPython')

        # set planner 0: A* 1: FF
        self.plan_method = '0'
        if rospy.has_param('/task_plan/plan_method'):
            self.plan_method = str(rospy.get_param('/task_plan/plan_method'))

        # define topic names
        self.service_name = '/task_plan/pddl_plan'
        self.subscriber_name = '/task_plan/request'
        self.publisher_name = '/task_plan/response'

        self.launch()


    def setSynchro(self, sync):
        self.sync = sync

    def launch(self):
        rospy.loginfo('Task planner is initiated')
        try:
            if self.sync == "sync":
                print("Planner launched in synchronous mode...")
                self.serverDomainNameProblem()
            elif self.sync == "async":
                print("Planner launched in asynchronous mode...")
                self.listenerDomainNameProblem()
        except Exception as e:
            if self.sync == "sync":
                print("Planner launched in synchronous mode...")
                self.serverDomainNameProblem()
            elif self.sync == "async":
                print("Planner launched in asynchronous mode...")
                self.listenerDomainNameProblem()

    # 			Communication functions (topic and services)		   
    ####################################################################
    ###			Asynchronous Communication 					

    def listenerDomainNameProblem(self):
        '''
        listen on the topic domain_problem_from_controller_topic
        It get a String msg structured like [problemDirectory__problemName]
        The callback function is resolvProblemAsTopic which take the data received in parameter
        :param: void
        :return: void
        '''
        rospy.Subscriber(self.topic_name, String, self.resolvProblemAsTopic)
        print(">> Ready to be requested, waiting a std_msgs/String...")
        print(">> Topic : " + self.topic_name)
        print(">> Callback : resolvProblemAsTopic...")
        print("##################################################################")

    def talkerJsonObject(self, data):
        '''
        create the answer as a String message structured like 
        '''
        timeToSleepBeforeSending = rospy.Rate(5)
        pub = rospy.Publisher(self.publisher_name, String, queue_size=10)
        #TODO
        # sleep before checking if someone is subscribed to the topic
        rospy.Rate(10).sleep()
        # waiting for someone to subscribe to the topic before publishing
        while pub.get_num_connections() < 1:
            print("Waiting for someone to connect to the plan_from_pddl4j_topic...")
            rospy.Rate(1).sleep()
        # writing on the topic
        rospy.loginfo(data)
        timeToSleepBeforeSending.sleep()
        pub.publish(data)

        print("Still ready to resolv a problem on the topic...")

    ####################################################################
    ###				Synchronous Communication 		
    # 			
    def serverDomainNameProblem(self):
        '''
        Wait to be requested by a RequestPlannerPlanification structure service message
        Call resolvProblemAsService as a callback function
        :param: void 
        :return: void
        '''
        s = rospy.Service(self.service_name, Planning, self.resolvProblemAsService)
        print(">> Ready to be requested, waiting a RequestPlannerPlanification...")
        print(">> Service : " + self.service_name)
        print(">> Callback : resolvProblemAsService...")
        print("##################################################################")        


    ####################################################################
    #						Callback functions					   
    ####################################################################

    def resolvProblemAsTopic(self, req):
        returnData = self.resolvProblem(req)
        self.talkerJsonObject(returnData)

    def resolvProblemAsService(self, req):
        returnData = self.resolvProblem(req)        
        return returnData

    
    # Planning function
    def resolvProblem(self, req):
        '''
        Try to resolv the problem in req.problemDirectory and req.problemName or in the req.data message by calling pddl4j.jar
        The java core is resolving the problem and modifying the json file
        :param: req= [problemPath, domatinPath] 
        :return: res= [problemResolved, planStatus, jsonPath, jsonObject]
        '''

        print("Loading PROBLEM and DOMAIN files...")
        
        problemPath = req.problemPath
        domainPath = req.domainPath

        operationStatus = "INIT"
        problemResolved = False

        if(os.path.isfile(problemPath) and os.path.isfile(domainPath)):
            javaCommand = "java -jar " + self.PLANNER_PATH + " -o " + \
                domainPath + " -f " + problemPath + " -json " + self.JSON_PATH + " -p " + self.plan_method
        else:
            javaCommand = "Error: PDDL model Files not found..."
            operationStatus = "fileNotFound"

        print("javaCommand : " + javaCommand)

        if(operationStatus != "fileNotFound"):
            # Launch the java command
            # use the .jar file giving him the problem, the domain and the path
            # to the json file to create/edit
            os.system(javaCommand)
            try:
                self.sequentialPlan = adaptator.getSequentialPlanFromJson(self.JSON_PATH)
                print self.sequentialPlan
                operationStatus = "Ok"
                problemResolved = True
            except Exception as e:
                print("JSON file could not be parsed\nProblem in the resolution")
                self.sequentialPlan = "False"
                operationStatus = "Error"
        else:
            print("One of the file could not be found...")
            self.sequentialPlan = "False"

        #returnData = [self.JSON_PATH, problemResolved, problemDirectory,
        #              problemName, self.sequentialPlan, operationStatus]

        returnData = PlanningResponse()
        returnData.problemResolved = problemResolved
        returnData.planStatus = operationStatus
        returnData.jsonPath = self.JSON_PATH
        returnData.jsonObject = pickle.dumps(self.sequentialPlan, pickle.HIGHEST_PROTOCOL)
        
        return returnData


if __name__ == "__main__":

    rospy.init_node('task_planner', anonymous=True)

    planner = TaskPlanner()
    rospy.spin()