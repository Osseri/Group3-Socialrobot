#!/usr/bin/env python

import rospy
import rospkg
import os
import sys

# Msg structure for the topic communication
from std_msgs.msg import String
from socialrobot_task_msgs.srv import *
from socialrobot_task_msgs.msg import *
import socialrobot_interface.msg
import socialrobot_interface.srv

# Serialization
import cPickle as pickle

class ProblemGenerator:    

    def __init__(self):       
        rospy.loginfo("Initializing ProblemGenerator")
        # init path parameters
        self.PROBLEM_PATH = ''
        self.domain_name = ''
        self.problem = Problem()

        # get the library path for planner
        if rospy.has_param('/task_plan/problem_path'):
            self.PROBLEM_PATH = rospy.get_param('/task_plan/problem_path')
        else:
            rospack = rospkg.RosPack()
            PKG_PATH = rospack.get_path('socialrobot_task_planner')
            self.PROBLEM_PATH = PKG_PATH + '/pddl/problem.pddl'

        rospy.loginfo('ProblemGenerator is initiated')

    def generate(self, domain="social_robot"):
        '''
        request : predicate PDDL models
        response : result 
        '''
        rospy.loginfo("Generating PDDL problem file.")
        
        # service 
        rospy.loginfo("ProblemGenerator trying to connect to the knowledge interface...")
        rospy.wait_for_service('/socialrobot_interface/knowledge/get_problem')    

        problem_srv = rospy.ServiceProxy('/socialrobot_interface/knowledge/get_problem', socialrobot_interface.srv.Knowledge)

        # Generate Problem
        self.domain_name = domain
        get_problem_req = socialrobot_interface.srv.KnowledgeRequest()
        get_problem_req.command = ""
        
        problem_info = problem_srv(get_problem_req).problem
        
        res = self._generate(problem_info)

        if(res == False):
            rospy.loginfo("Generating PDDL problem file is failed")
            return res

        rospy.loginfo("PDDL problem file is generated.")
        return res

    def _generate(self, problem_info):
        '''
        Make the problem.pddl file in specific directory
        '''
        result = True
        # open pddl file
        f = open(self.PROBLEM_PATH, 'w+')
        f.write("(define\n")
        
        # problem
        self._write_problem(f, problem_info)
        # domain
        self._write_domain(f, problem_info)
        #
        self._write_objects(f, problem_info)
        # 
        self._write_init(f, problem_info)
        #
        self._write_goal(f, problem_info)

        f.write(")")
        f.close()
        return result

    def _write_domain(self, file, problem):
        if self.domain_name == '':
            rospy.logerr("ProblemGenerator needs domain name")
        else:
            file.write("\t(:domain " + self.domain_name + ")\n")
        return

    def _write_problem(self, file, problem):
        file.write("\t(problem " + problem.problem_name + ")\n")
        return
    
    def _write_init(self, file, problem):
        file.write("\t(:init\n")        
        self._write_predicates(file, problem, 'INIT')
        
        file.write("\t)\n")
        return

    def _write_goal(self, file, problem):
        file.write("\t(:goal\n\t\t(and\n")
        self._write_predicates(file, problem, 'GOAL')
        file.write("\t\t)\n\t)\n")
        return

    def _write_objects(self, file, problem):
        file.write("\t(:objects\n")
        
        # convert key_value to dict
        dictionary = self._key_values2dict(problem.objects)
        #
        
        for k in dictionary.keys():
            file.write("\t\t")
            for v in dictionary[k]:
                file.write(v + " ")
            file.write("- "+ k +"\n")
        file.write("\t)\n")
        
        return

    def _write_predicates(self, file, problem, state):
        if state == "INIT":
            for f in problem.facts:
                if f.is_negative == False :         #positive predicate
                    file.write("\t\t(" + f.name)
                    for a in f.args:
                        file.write(" " + a)
                    file.write(")\n")  
                elif f.is_negative == True :        #negative predicate
                    file.write("\t\t(not (" + f.name)
                    for a in f.args:
                        file.write(" " + a)
                    file.write("))\n")  
        elif state == "GOAL":
            for f in problem.goals:
                if f.is_negative == False :         #positive predicate
                    file.write("\t\t(" + f.name)
                    for a in f.args:
                        file.write(" " + a)
                    file.write(")\n")  
                elif f.is_negative == True :        #negative predicate
                    file.write("\t\t(not (" + f.name)
                    for a in f.args:
                        file.write(" " + a)
                    file.write("))\n")  

    
    def _key_values2dict(self, key_values):
        dictionary = {}

        for i, arg in enumerate(key_values):
            key = arg.key
            value = arg.value
            if( key in dictionary):
                dictionary[key].append(value)
            else:
                dictionary.update({key:[]})
                dictionary[key].append(value)

        return dictionary


if __name__ == "__main__":

    rospy.init_node('problem_generator', anonymous=True)
    pg = ProblemGenerator()
    pg.generate()
