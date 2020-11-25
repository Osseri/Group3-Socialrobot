#!/usr/bin/env python
# license removed for breviry
import os
import sys
import signal
import json
import cPickle as pickle
import rospy
import rospkg
import rosparam
from std_msgs.msg import Header
from std_msgs.msg import String

from domain_parser import PDDL_Parser
from socialrobot_actionlib.srv import *
from socialrobot_actionlib.msg import *

class ActionLib():
    '''
    action library
    '''
    actionlib_conf = ""

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('socialrobot_actionlib')

        # Check action model file
        if rospy.has_param('library_path'):
            self.actionlib_path = rospy.get_param('library_path')
        else:
            try:
                rospy.logwarn('action library cannot be found! a local path will be loaded.')
                rp = rospkg.RosPack()
                actionlib_path = rp.get_path('socialrobot_actionlib') + '/config/action_library.pddl'
                self.actionlib_path = actionlib_path
            except:
                rospy.logerr('Cannot load action model files...')

        # parse action models
        self.domain = PDDL_Parser()
        self.domain.parse_domain(self.actionlib_path)
        self.actions = self.domain.actions
        self.available_actions = self.actions

        num_act = len(self.actions)
        rospy.loginfo('[ActionLibrary] %d action is loaded.' %num_act)

    def run(self):
        # ROS services
        rospy.Service('/actionlib/get_action_list', GetActionList, self._callback_get_action_list)
        rospy.Service('/actionlib/get_action_info', GetActionInfo, self._callback_get_action_info)
        rospy.Service('/actionlib/check_action', CheckAction, self._callback_check_action)
        rospy.Service('/actionlib/get_domain', GetDomain, self._callback_get_domain)
        rospy.Service('/actionlib/decode_action', GetPrimitiveActionList, self._callback_decode_action) 

    def __del__(self):
        # Exit
        rospy.loginfo('[ActionLibrary] Service terminated!')

    def _callback_get_action_list(self, req):
        
        res = GetActionListResponse()

        # All action list
        if req.action_type == req.ALL:
            for act_name in self.actions.keys():
                res.actions.append(String(act_name))            
        # Available action list in current robot configuration
        elif req.action_type == req.AVAILABLE_ACTIONS:
            for act_name in self.available_actions.keys():
                res.actions.append(String(act_name))
        else:
            rospy.logerr("Request parameter 'action_type' should be 'ALL' or 'AVAILABLE_ACTIONS'")

        return res

    def _callback_get_domain(self, req):

        res = GetDomainResponse()
        hardware_list = map(str.lower, req.group_list )
        actions = {}

        if len(hardware_list) < 1:
            rospy.logwarn("Robot hardware constraints input is required")           
            actions = self.actions
            res.actions = pickle.dumps(self.actions, pickle.HIGHEST_PROTOCOL) 
        else:            
            for act_name in self.actions.keys():
                group = map(str.lower, self.actions[act_name]['group'])
                if group == [var for var in group if var in hardware_list]:
                    actions[act_name] = self.actions[act_name]
            self.available_actions = actions              
            res.actions = pickle.dumps(self.available_actions, pickle.HIGHEST_PROTOCOL) 

        res.requirements = self.domain.requirements
        res.types  = pickle.dumps(self.domain.constants, pickle.HIGHEST_PROTOCOL)
        res.predicates = pickle.dumps(self.domain.predicates, pickle.HIGHEST_PROTOCOL)
        res.result = True
        
        return res

    def _callback_check_action(self, req):
        '''
        Check the action possibility
        '''

        res = CheckActionResponse()
        
        return res

    def _callback_decode_action(self, req):  
        compound_action = req.compound_action         
        res = GetPrimitiveActionListResponse()
        res.primitive_action = self._decode_action(compound_action) 
        return res

    def _decode_action(self, compound_action):
        res = []
        if not compound_action.name in self.actions.keys():
            rospy.logerr("Action %s is not in action library." %compound_action.name)
            return res            
        
        if 'primitives' in self.actions[compound_action.name].keys():
            primitive_actions = self.actions[compound_action.name]['primitives']
            for i, primitive_action in enumerate(primitive_actions):
                action = socialrobot_actionlib.msg.Action()
                action.name = primitive_action['name']

                # get primitive action parameter from compound action
                action.parameters = self._decode_param(compound_action, i)
                
                action.controller = (self.actions[action.name]['controller'])
                action.group = (self.actions[action.name]['group'])
                action.planner = (self.actions[action.name]['planner'])
                
                sub_actions = self._decode_action(action)
                for prim in sub_actions:
                    res.append(prim)

        # no primitives
        else:
            action = socialrobot_actionlib.msg.Action()
            action.name = compound_action.name

            action.parameters = compound_action.parameters
            action.controller = (self.actions[action.name]['controller'])
            action.group = (self.actions[action.name]['group'])
            action.planner = (self.actions[action.name]['planner'])
            res.append(action)
            
        return res

    def _decode_param(self, compound_action, idx):
        param_list = []
        if len(self.actions[compound_action.name]['primitives']) < 1:
            param_list = compound_action.parameters
        else:
            compound_param = []
            for param in self.actions[compound_action.name]['parameters']:
                compound_param.append(param['name'])
            primitive_param = self.actions[compound_action.name]['primitives'][idx]['parameters']

            for param in primitive_param:
                if param in compound_param:
                    idx = compound_param.index(param)
                    param_list.append(compound_action.parameters[idx])
                else:
                    rospy.logwarn("Cannot find %s action parameter in compound action %s" %(param, compound_action.name))

        return param_list

    def _callback_get_action_info(self, req):
        res = GetActionInfoResponse()
        action_name = req.action_name
        predicates = req. predicates
        options = req.options

        if not action_name in self.available_actions.keys():
            rospy.logwarn("Cannot find '%s' action in the action list." %action_name)
            res.result = False
            return res
        if 'parameters' in self.actions[action_name].keys():
            for param in self.actions[action_name]['parameters']:
                res.parameters.append(param['name'])          
        if 'primitives' in self.actions[action_name].keys():  
            for prim in self.actions[action_name]['primitives']:
                res.primitives.append(prim['name'])                       
        if 'controller' in self.actions[action_name].keys():     
            res.controller = self.actions[action_name]['controller']           
        if 'group' in self.actions[action_name].keys():             
            res.group = self.actions[action_name]['group']           
        if 'planner' in self.actions[action_name].keys(): 
            res.planner = self.actions[action_name]['planner']     
        if 'precondition' in self.actions[action_name].keys(): 
            res.precondition.append(pickle.dumps(self.actions[action_name]['precondition'], pickle.HIGHEST_PROTOCOL))     
        if 'effect' in self.actions[action_name].keys(): 
            res.effect.append(pickle.dumps(self.actions[action_name]['effect'], pickle.HIGHEST_PROTOCOL))
        res.result = True

        return res



if __name__ =='__main__':
    al = ActionLib()
    al.run()    
    rospy.spin()
