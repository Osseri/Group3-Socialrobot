#!/usr/bin/env python

import rospy
import rospkg
import os
import sys
import cPickle as pickle
import yaml

# Msg structure for the topic communication
from std_msgs.msg import String
from socialrobot_task.srv import *
from socialrobot_actionlib.msg import *


class ProblemGenerator:    

    def __init__(self):       
        # init path parameters
        self.PROBLEM_PATH = ''
        self.domain_name = ''
        self.problem = Problem()

        # get the library path for planner
        if rospy.has_param('/task_plan/problem_path'):
            self.PROBLEM_PATH = rospy.get_param('/task_plan/problem_path')
        else:
            rospack = rospkg.RosPack()
            PKG_PATH = rospack.get_path('socialrobot_task')
            self.PROBLEM_PATH = PKG_PATH + '/pddl/problem.pddl'
        rospy.Service('/task_plan/set_problem', SetProblem, self._callback_set_problem)

    def _callback_set_problem(self, req):
        res = SetProblemResponse()
        
        res.result = self.set_problem(problem = req.problem)
        self.generate()

        return res

    def generate(self, domain="socialrobot"):
        rospy.loginfo("Generating PDDL problem file...")   
        
        # Generate Problem
        self.domain_name = domain   
        if self.problem.domain_name:
            self.problem.domain_name = domain   
            res = self._generate(self.problem)
        else:
            rospy.logerr("Setup the initial state.")
            return False

        if(res == False):
            rospy.logerr("Generating PDDL problem file is failed")
            return False

        rospy.loginfo("PDDL problem file is generated.")
        return True

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
            return
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

    def set_problem(self, demo_name = 'prob_default', problem = None):
        
        # if problem is not input parameter, load from local file
        if problem == None:
            problem = Problem()
            rospack = rospkg.RosPack()
            task_pkg_dir = rospack.get_path("socialrobot_task_planner")
            file_path = task_pkg_dir + '/pddl/example/' + demo_name + '.yaml'
            with open(file_path, 'r') as f:
                example = yaml.load(f) 

            problem.problem_name =  example['Demo']['problem']
            problem.domain_name =  example['Demo']['domain']
            for obj in example['Demo']['object']:
                self._add_object(problem, obj[0], obj[1])
            for prob in example['Demo']['fact']:
                self._add_fact(problem, prob[0], prob[1])
            for goal in example['Demo']['goal']:
                self._add_goal(problem, goal[0], goal[1])    
        # 
        else:
            pass

        self.problem = problem        
        return True

    def _add_object(self, problem, instance_name, obj_type):
        msg = problem
        key_value = diagnostic_msgs.msg.KeyValue()
        key_value.key = obj_type		    
        key_value.value = instance_name		
        msg.objects.append(key_value)   
        return msg

    def _add_fact(self, problem, name, args):
        msg = problem
        predicate = socialrobot_task_msgs.msg.Predicate()
        predicate.name = name			    
        predicate.args = args			   
        msg.facts.append(predicate)   
        return msg

    def _add_goal(self, problem, name, args):
        msg = problem
        predicate = socialrobot_task_msgs.msg.Predicate()
        predicate.name = name			   
        predicate.args = args			  
        msg.goals.append(predicate)   
        return msg

if __name__ == "__main__":

    rospy.init_node('problem_generator', anonymous=True)
    pg = ProblemGenerator()
    pg.set_problem(demo_name = 'prob_default')
    pg.generate()
