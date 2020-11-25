#!/usr/bin/env python

import rospy
import rospkg
import os
import sys

from socialrobot_task.srv import *
from socialrobot_actionlib.msg import *

class FastDownwardPlanner:    

    def __init__(self):

        # get the library path for planner
        rospack = rospkg.RosPack()
        PKG_PATH = rospack.get_path('socialrobot_task')        
        self.PLANNER_PATH = PKG_PATH + '/libs/fast-downward/fast-downward.py'
        self.PLAN_PATH = PKG_PATH + '/data/plan'
        self.PLAN_OPTION = '--plan-file '+ self.PLAN_PATH
        self.COMPONENT_OPTION = '--search "lazy_greedy([ff()], preferred=[ff()])"'

   
    # Planning function
    def resolvProblem(self, res, domainPath, problemPath):
        '''
        '''
        print("Loading PROBLEM and DOMAIN files...")
        
        # initialize
        operationStatus = "Init"
        problemResolved = False
        if(os.path.isfile(self.PLAN_OPTION)):
            file = open(self.PLAN_OPTION,"r+")
            file.truncate(0)
            file.close()
            pass

        # planning
        if(os.path.isfile(problemPath) and os.path.isfile(domainPath)):
            command = "python " + self.PLANNER_PATH + " " + self.PLAN_OPTION + " " + domainPath + " " + problemPath + " " + self.COMPONENT_OPTION
        else:
            command = "Error: PDDL model Files not found..."
            res.plan_result = res.FAIL

        if(operationStatus != "FileNotFound"):            
            # execute fast-downward planner
            os.system(command)
            # read plan
            try:
                plan_file = open(self.PLAN_PATH, "r")
                action_list = plan_file.readlines()                
                for line_data in action_list[:-1]:
                    act_data = line_data[1:len(line_data)-2].split(' ')
                    action = Action()
                    action.name = act_data.pop(0)
                    action.parameters = act_data
                    res.action_sequence.append(action)
                    res.plan_result = res.SUCCESS

            except Exception as e:
                print("plan file could not be parsed.")
                res.plan_result = res.FAIL
        else:
            rospy.logerr("PDDL file could not be found...")
            res.plan_result = res.FAIL
        
        return res