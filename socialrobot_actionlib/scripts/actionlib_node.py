#!/usr/bin/env python
# license removed for breviry
import os
import sys
import signal
import json
import cPickle as pickle
import itertools
import copy
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
        elif req.action_type == req.COMPOUND_ACTIONS:
            for act in self.actions:
                if self.actions[act].has_key('primitives'):
                    res.actions.append(String(act))
        elif req.action_type == req.PRIMITIVE_ACTIONS:
            for act in self.actions:
                if not self.actions[act].has_key('primitives'):
                    res.actions.append(String(act))
        else:
            rospy.logerr("Request parameter 'action_type' is required.")

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
        res.result = True
        
        return res

    def _callback_decode_action(self, req):  
        compound_action = req.compound_action    
        current_states = req.current_states
        res = GetPrimitiveActionListResponse()
        res.primitive_action = self._decode_action(compound_action, current_states) 
        return res

    def _decode_action(self, compound_action, current_states):
        '''
        compound_action : requested action for decoding
        current_states : [optional] current state predicates
        '''
        res = []
        if not compound_action.name in self.actions.keys():
            rospy.logerr("Action %s is not in action library." %compound_action.name)
            return res            
        
        # get primitive action list
        primitive_actions = []
        self.get_primitives(compound_action, current_states, primitive_actions)
        
        # groundify parameters
        for act in primitive_actions:
            ground_act = self.groundify_action(act['name'], act['value'])
            res.append(ground_act)
        return res

    def _callback_get_action_info(self, req):
        res = GetActionInfoResponse()
        action_name = req.action_name
        params = req.params

        if not action_name in self.available_actions.keys():
            rospy.logwarn("Cannot find '%s' action in the action list." %action_name)
            res.result = False
            return res
        else:        
            res.result = True
            res.action = self.groundify_action(action_name, params)

        return res

    def create_type_map(self, values, parameters):
        type_map = {}
        for i, param in enumerate(parameters):
            type_map.setdefault(param['name'], {})
            type_map[param['name']]['value'] = values[i]
            type_map[param['name']]['type'] = param['type']
        return type_map

    def check_condition(self, condition_predicates, current_states):
        '''
        condition_predicates [socialrobot_actionlib/Predicate]
        current_states [socialrobot_actionlib/Predicate]
        '''
        # no current states
        if not current_states:
            return True
            
        neg_pred = []
        pos_pred = []
        current_pred = []
        for cond in condition_predicates:
            if not cond.is_negative:
                pred = [cond.name.lower()]
                for arg in cond.args:
                    pred.append(arg.lower())
                pos_pred.append(pred)
            else:                
                pred = [cond.name.lower()]
                for arg in cond.args:
                    pred.append(arg.lower())
                neg_pred.append(pred)
        for state in current_states:
            pred = [state.name.lower()]
            for arg in state.args:
                pred.append(arg.lower())
            current_pred.append(pred)
            
        for pos in pos_pred:
            if pos not in current_pred:
                return False
        for neg in neg_pred:
            if neg in current_pred:
                return False
        return True

    def groundify_action(self, action_name, action_variables, current_states=None):
        
        action_model = self.actions[action_name]
        
        act = Action()
        act.name = action_name
        act.controller = action_model['controller']
        act.group = action_model['group']
        act.planner = action_model['planner']

        params = action_model['parameters']
        precond = action_model['precondition']
        effect = action_model['effect']
        for i, param in enumerate(params):
            act.type.append(action_model['parameters'][i]['type'])
            act.parameters.append(action_model['parameters'][i]['name'])
        act.values = action_variables

        if action_variables:
            # create parameter map
            type_map = self.create_type_map(action_variables, action_model['parameters'])

            # groundify primitives
            primitive_actions = [] 
            if 'primitives' in action_model.keys():
                primitives = []
                for i in action_model['primitives']:
                    self.split_primitives(i, primitives)           
                for i in primitives:                 
                    act.primitives.append(i['name'])

            # groundify predicates          
            pos_precond = []
            neg_precond = []
            pos_effect = []
            neg_effect = []              
            for p in precond:     
                self.groundify_predicate(p, type_map, pos_precond, neg_precond)        
            for e in effect:     
                self.groundify_predicate(e, type_map, pos_effect, neg_effect)

            # convert to msg format
            
            if pos_precond:
                for precond in pos_precond:
                    fluent = Fluent(predicate=precond[0], args=precond[1])  
                    act.precondition.positives.append(fluent)            

            if neg_precond:
                for precond in neg_precond:
                    fluent = Fluent(predicate=precond[0], args=precond[1])  
                    act.precondition.negatives.append(fluent)  
                
            if pos_effect:
                for precond in pos_effect:
                    fluent = Fluent(predicate=precond[0], args=precond[1])  
                    act.effect.positives.append(fluent)  
                
            if neg_effect:
                for precond in neg_effect:
                    fluent = Fluent(predicate=precond[0], args=precond[1])  
                    act.effect.negatives.append(fluent)              

        return act

    def groundify_predicate(self, predicate, type_map, pos_predicate, neg_predicate, current_state=None):
        '''
        mapping the objects into action parameters
        '''
        # TODO: consider current predicate states
        pred_type = predicate[0]
        pred_var = predicate[1:]
        neg_ground_pred = []
        pos_ground_pred = []
        
        # 
        if pred_type == 'forall': 
            pass
        elif pred_type == 'when':
            pass                
        elif pred_type == 'exists':
            pass
        elif pred_type == 'and':
            pass
        elif pred_type == 'or':   
            pass
        elif pred_type == 'equal':   
            pass
        elif pred_type == 'not': 
            neg_ground_pred = [pred_var[0][0], pred_var[0][1:]]
        elif pred_type == 'type':
            pass
            
        else:           
            pos_ground_pred = [pred_type, pred_var]
            
        # groundify
        if pos_ground_pred:        
            pred, params = pos_ground_pred
            for i,param in enumerate(params):
                if type_map.has_key(param['name']):
                    params[i] = type_map[param['name']]['value'] 
                else:   # constant value
                    params[i] = param['name']
            pos_predicate.append(pos_ground_pred)
            
        if neg_ground_pred:
            pred, params = neg_ground_pred
            for i,param in enumerate(params):
                if type_map.has_key(param['name']):
                    params[i] = type_map[param['name']]['value']
                else:   # constant value
                    params[i] = param['name']
            neg_predicate.append(neg_ground_pred)

    def groudify_predicates(self, predicates, type_map):
        '''
        assign parameters value into symbolic parameters
        '''
        pred_list = []        
        for p in predicates:
            pred = Predicate()
            # negative predicate
            if p[0] == 'not':
                p = p[1]
                pred.is_negative = True

            # positive predicate            
            pred.name = p[0]
            pred_params = p[1:]
            for param in pred_params:
                value = type_map[param['name']]['value']
                pred.args.append(value)
            pred_list.append(pred)
        return pred_list

    def get_primitives(self, compound_action, current_states, action_list, compound_action_values=None):
        # create compound action's parameter map
        action_values = []
        if compound_action_values != None:
            type_map = self.create_type_map(compound_action_values, self.actions[compound_action.name]['parameters'])
        else:
            type_map = self.create_type_map(compound_action.values, self.actions[compound_action.name]['parameters'])

        for param in self.actions[compound_action.name]['parameters']:
            #param['value'] = type_map[param['name']]['value']
            action_values.append(type_map[param['name']]['value'])
        
        # high level compound action
        if 'primitives' in self.actions[compound_action.name].keys():
            primitives = self.actions[compound_action.name]['primitives']

            for i, p in enumerate(primitives):
                # conditional primitive case
                if str(type(p)) != "<type 'dict'>":
                    cond_type = p[0]
                    condition_predicates = []
                    primitive_actions = []

                    # get conditional predicates and primitive actions
                    if cond_type == 'forall': 
                        pass
                    elif cond_type == 'when':
                        self.split_predicates(p[1][0], condition_predicates)
                        condition_predicates = self.groudify_predicates(condition_predicates, type_map)
                        self.split_primitives(p[2], primitive_actions)
                    elif cond_type == 'exists':
                        pass
                    
                    # add primitive actions if in current condition
                    if self.check_condition(condition_predicates, current_states):
                        for a in primitive_actions:
                            act_msg = self.dict_action_to_action_msg(a)
                            primitive_values = self.inherit_values(type_map, a['parameters'])
                            self.get_primitives(act_msg, current_states, action_list, primitive_values)

                # no conditional primitive case
                else:                    
                    act_msg = self.dict_action_to_action_msg(p)
                    primitive_values = self.inherit_values(type_map, p['parameters'])
                    self.get_primitives(act_msg, current_states, action_list, primitive_values)

        # lowest level primitive action
        else:
            action_msg = self.groundify_action(compound_action.name, compound_action.values)
            dict_action = self.action_msg_to_dict_action(action_msg)
            if compound_action_values != None:
                dict_action['value'] = compound_action_values
            else:
                dict_action['value'] = compound_action.values
            action_list.append(dict_action)

    def inherit_values(self, parent_type_map, child_parameters):
        values = []
        for param in child_parameters:
            try:    # value is in the type map
                values.append(parent_type_map[param]['value'])
            except: # value is constant value
                values.append(param)
        return values

    def split_predicates(self, predicates, pred_list):
        if predicates[0] == 'and':
            for p in predicates[1:]:
                self.split_predicates(p, pred_list)
        else:
            pred_list.append(predicates)

    def split_primitives(self, primitive, prim_list):
        if str(type(primitive)) == "<type 'list'>":
            if primitive[0] == 'and':
                for p in primitive[1:]:
                    self.split_primitives(p, prim_list)
        elif str(type(primitive)) == "<type 'dict'>":
            prim_list.append(primitive)
    
    def dict_action_to_action_msg(self, dict_action):
        '''
        convert dict action format into ROS action msg
        '''
        return self.groundify_action(dict_action['name'], dict_action['parameters'])

    def action_msg_to_dict_action(self, action_msg):
        '''
        convert ROS action msg format into dict format 
        '''
        dict_action = {'name':action_msg.name,
                        'parameters':action_msg.parameters}
        return dict_action

if __name__ =='__main__':
    al = ActionLib()
    al.run()    
    rospy.spin()
