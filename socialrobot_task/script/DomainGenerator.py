#!/usr/bin/env python

import rospy
import rospkg
import os
import sys

# Msg structure for the topic communication
from std_msgs.msg import String
from socialrobot_actionlib.srv import *
from socialrobot_actionlib.msg import *

# Serialization
import cPickle as pickle

class DomainGenerator:    

    def __init__(self):       

        # init parameters
        self.DOMAIN_PATH = ''
        self.domain_name = ""
        self.requirements = []
        self.types = []
        self.constants = []
        self.actions = {}
        self.predicates = {}

        # get the library path for planner
        if rospy.has_param('/task_plan/domain_path'):
            self.DOMAIN_PATH = rospy.get_param('/task_plan/domain_path')
        else:
            rospack = rospkg.RosPack()
            PKG_PATH = rospack.get_path('socialrobot_task')
            self.DOMAIN_PATH = PKG_PATH + '/pddl/domain.pddl'

        # service list
        self.domain_srv = rospy.ServiceProxy('/actionlib/get_domain', GetDomain)   

    def generate(self, domain="socialrobot"):
        '''
        Make the domain.pddl file in specific directory
        '''
        self.domain_name = domain

        # get the domain info from action library
        if self._get_domain_info():

            # open pddl file
            f = open(self.DOMAIN_PATH, 'w+')
            f.write("(define\n")

            # domain
            self._write_domain(f)
            #
            self._write_requirements(f)
            # 
            self._write_types(f)
            #
            self._write_constants(f)
            #
            self._write_predicates(f)
            #
            self._write_actions(f)

            f.write(")")
            f.close()
            rospy.loginfo("DomainGenerator created domain.pddl file.")
            return True
        else:
            rospy.logerr("DomainGenerator is failed to create domain.pddl")
            return False

    def _write_actions(self, file):
        file.write("\t;;;;;;;;;;;;;;;;;;;;;;;;;;; Define actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;\n")

        for action in self.actions:
            file.write("\t(:action ")
            file.write(action)
            
            #action parameter
            file.write("\n\t\t:parameters (")
            for idx, param in enumerate(self.actions[action]['parameters']):
                file.write(param['name'])
                file.write(" - ")
                file.write(param['type'])
                if not idx == len(self.actions[action]['parameters'])-1:
                    file.write(" ")
            file.write(")\n")
            
            #action precondition
            file.write("\t\t:precondition\n")
            file.write("\t\t(and\n")            
            for pred  in self.actions[action]["precondition"]:                
                self._write_action_predicates(file, pred)            
            file.write("\t\t)\n")

            
            #action effects
            file.write("\t\t:effect\n")
            file.write("\t\t(and\n")
            for pred  in self.actions[action]["effect"]:                
                self._write_action_predicates(file, pred)            
            file.write("\t\t)\n")

            file.write("\t)\n")

        return
        
    def _write_action_predicates(self, file, pred):        
            
        pred_name = pred[0]
        if pred_name == "not" or pred_name == "and" or pred_name == "or":
            file.write("\t\t\t(")
            file.write(pred_name)
            file.write("\n")
            for sub_pred in pred[1:]:                
                self._write_action_predicates(file,sub_pred)
            file.write("\t\t\t)\n")     

        elif pred_name == "forall":
            # set parameters
            file.write("\t\t\t(forall (")
            for idx, param in enumerate(pred[1]):
                file.write(param['name'])
                file.write(" - ")
                file.write(param['type'])
                if not idx == len(pred[1])-1:
                    file.write(" ")
            file.write(")\n")
            # set predicates
            self._write_action_predicates(file, pred[2])
            file.write("\t\t\t)\n")

        elif pred_name == "when":
            file.write("\t\t\t(when \n")
            # set parameters
            self._write_action_predicates(file, pred[1])
            # set predicates
            self._write_action_predicates(file, pred[2])
            file.write("\t\t\t)\n")
            
        else:  
            file.write("\t\t\t(")
            if pred_name == "equal" or pred_name == "=":        #equality conidtion
                pred_name ="="
            file.write(pred_name)
            #file.write(" ")  

            params = pred[1:]
            for param in params:
                file.write(" ")
                file.write(param['name'])
                #file.write(" - ")
                #file.write(param["type"])
            file.write(")\n")    

        return 

    def _write_predicates(self, file):
        file.write("\t(:predicates\n")
        for pred in self.predicates:
            file.write("\t\t(")
            file.write(pred)
            #file.write(" ")  
            params = self.predicates[pred]
            for param in params:
                file.write(" ")
                file.write(param['name'])
                file.write(" - ")
                file.write(param["type"])
            file.write(")\n")    

        file.write("\t)\n")

        return True

    def _write_constants(self, file):

        file.write("\t(:constants\n")
        for key in self.types:
            if len(self.types[key]) >0:
                file.write("\t\t")
                for value in self.types[key]:                    
                    file.write(value)
                    file.write(" ")
                file.write("- ")
                file.write(key)
            file.write("\n")

        file.write("\t)\n")
        return True

    def _write_types(self, file):

        file.write("\t(:types\n")
        for t in self.types:
            file.write("\t\t")
            file.write(t)
            file.write("\n")

        file.write("\t)\n")
        return True

    def _write_requirements(self, file):

        file.write("\t(:requirements\n")
            
        for req in self.requirements:
            file.write("\t\t")
            file.write(req)
            file.write("\n")
        file.write("\t)\n")

        return True

    def _write_domain(self, file):

        file.write("\t(domain " + self.domain_name + ")\n")

        return True

    def _get_group_list(self):        
        # set default hardware group
        group_list = ['Arm','Gripper','mobile']
        return group_list

    def _get_domain_name(self):

        if rospy.has_param('/robot_name'):
            self.domain_name = rospy.get_param('/robot_name')
        else:
            return False
        return True


    def _get_domain_info(self):

        # get domain info
        get_domain_req = GetDomainRequest()            
        get_domain_req.group_list = self._get_group_list()
        
        rospy.loginfo("DomainGenerator trying to connect to the Action Library service...")
        rospy.wait_for_service('/actionlib/get_domain')    
        domain_info = self.domain_srv(get_domain_req)

        self.requirements = domain_info.requirements
        self.types = pickle.loads(domain_info.types)
        self.actions = pickle.loads(domain_info.actions)
        self.predicates = pickle.loads(domain_info.predicates)
        
        return domain_info.result

if __name__ == "__main__":
    rospy.init_node('problem_generator', anonymous=True)
    dg = DomainGenerator()
    dg.generate()
