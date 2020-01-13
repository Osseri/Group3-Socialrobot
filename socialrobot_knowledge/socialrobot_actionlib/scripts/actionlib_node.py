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

import action_reader
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
        rospy.loginfo('[ActionLibrary] Service Started!')

        # Check action model file
        if rospy.has_param('rdf_path'):
            self.actionlib_conf = rospy.get_param('rdf_path')
        else:
            try:
                rospy.loginfo('action model cannot be found! a local configuration will be loaded.')
                rp = rospkg.RosPack()
                conf_path = rp.get_path('socialrobot_actionlib') + '/config/'
                self.actionlib_conf = conf_path
            except:
                rospy.logerr('Cannot load action model files...')

        # action reader
        self.actionReader = action_reader.ActionReader(self.actionlib_conf)

    def run(self):
        # ROS services
        rospy.Service('/actionlib/get_action_list', GetActionList, self._callback_get_action_list)
        rospy.Service('/actionlib/get_action_info', GetActionInfo, self._callback_get_action_info)
        rospy.Service('/actionlib/check_action', CheckAction, self._callback_check_action)
        rospy.Service('/actionlib/get_domain', GetDomain, self._callback_get_domain)
        rospy.Service('/actionlib/decode_action', GetPrimitiveActionList, self._callback_decode_action) 

        rospy.spin()

    def __del__(self):
        # Exit
        rospy.loginfo('[ActionLibrary] Service terminated!')

    def _callback_get_action_list(self, req):

        res = GetActionListResponse()

        # All action list
        if req.action_type == req.ALL:
            for action in self.actionReader.action_list:
                res.actions.append(String(action))            

        # Available action list in current robot configuration
        elif req.action_type == req.AVAILABLE_ACTIONS:
            for action in self.actionReader.available_action_list:
                res.actions.append(String(action))
        else:
            rospy.logerr("Request parameter 'action_type' should be ALL or VAILABLE_ACTIONS")
            return res 

        return res

    def _callback_get_domain(self, req):
        hardware_list = req.group_list   

        res = GetDomainResponse()
        actions = {}
        if len(hardware_list) < 1:
            rospy.logwarn("Robot hardware constraints input is required")           
            actions = self.actionReader.actions
        else:            
            available_action_list = self.actionReader.getAvailableActions(hardware_list)
            for action in available_action_list:
                actions[action] = self.actionReader.actions[action]
    
        res.actions = pickle.dumps(actions, pickle.HIGHEST_PROTOCOL)
        res.requirements = self.actionReader.requirements
        res.types  = pickle.dumps(self.actionReader.types, pickle.HIGHEST_PROTOCOL)
        res.predicates = pickle.dumps(self.actionReader.predicates, pickle.HIGHEST_PROTOCOL)
        res.result = True

        return res

    def _callback_check_action(self, req):
        '''
        Check the action possibility
        '''

        res = CheckActionResponse()
        
        return res

    def _callback_decode_action(self, req):
        res = GetPrimitiveActionListResponse()        
        
        compound_action = req.compound_action
        res.primitive_action = self._decode_action(compound_action)      

        return res

    def _decode_action(self, compound_action):
        res = []
        if not compound_action.name in self.actionReader.actions.keys():
            rospy.logerr("Action %s is not in action library." %compound_action.name)
            return res
        
        primitive_actions = self.actionReader.actions[compound_action.name]['primitives']
        # primitives exist
        if len(primitive_actions)>0:
            for primitive_action in primitive_actions:
                action = socialrobot_actionlib.msg.Action()
                action.name = primitive_action

                # get primitive action parameter from compound action
                action.parameters = self._decode_param(compound_action, primitive_action)
                action.controller = (self.actionReader.actions[primitive_action]['constraints']['controller'])
                action.group = (self.actionReader.actions[primitive_action]['constraints']['group'])
                action.planner = (self.actionReader.actions[primitive_action]['constraints']['planner'])
                
                res2 = self._decode_action(action)
                for prim in res2:
                    res.append(prim)

        # no primitives
        else:
            action = socialrobot_actionlib.msg.Action()
            action.name = compound_action.name

            action.parameters = self._decode_param(compound_action, None)
            action.controller = (self.actionReader.actions[compound_action.name]['constraints']['controller'])
            action.group = (self.actionReader.actions[compound_action.name]['constraints']['group'])
            action.planner = (self.actionReader.actions[compound_action.name]['constraints']['planner'])
            res.append(action)

        return res

    def _decode_param(self, compound_action, primitive_action):
        param_list = []
        if primitive_action == None:
            param_list = compound_action.parameters
        else:
            com = self.actionReader.actions[compound_action.name]['parameters']
            prim = self.actionReader.actions[primitive_action]['parameters']
            for param in prim:
                if param in com:
                    idx = com.index(param)
                    param_list.append(compound_action.parameters[idx])

                else:
                    rospy.logwarn("Cannot find %s action parameter in the %s action" %(primitive_action,compound_action.name))


        return param_list

    def _callback_get_action_info(self, req):
        res = GetActionInfoResponse()
        action_name = req.action_name
        predicates = req. predicates
        options = req.options

        if not action_name in self.actionReader.action_list:
            rospy.logwarn("Cannot find '%s' action in the actionlib" %action_name)
            res.result = False
            return res
        
        res.result = True
        action_info = self.actionReader.getActionInfo(action_name, predicates)

        res.parameters = action_info['parameters']
        res.precondition = action_info['precondition']
        res.effect = action_info['effect']
        res.primitives = action_info['primitives']
        res.controller = action_info['controller']
        res.group = action_info['group']
        res.planner = action_info['planner']

        return res



if __name__ =='__main__':
    al = ActionLib()
    al.run()
