#!/usr/bin/env python
import rospy
import rospkg
import rosparam
import rosservice
import math
import yaml
import numpy as np

import diagnostic_msgs.msg
import socialrobot_task.srv
from socialrobot_perception_msgs.msg import Objects, Object
from socialrobot_behavior.msg import PlannerInputs, PlannerOutputs
from socialrobot_behavior.msg import *
from socialrobot_behavior.srv import *
from sensor_msgs.msg import *
from rosjava_custom_srv.msg import *
from rosjava_custom_srv.srv import *
from socialrobot_actionlib.msg import Problem, Predicate
from mongodb_store.message_store import MessageStoreProxy
from interface import InterfaceBase

class KnowledgeInterface(InterfaceBase):
    #TODO: remove hard coding
    predicates = [
    ['on_Physical', 'Top', 'Bottom', '0', '0'],
    ['empty_hand', 'Hand', '0', '0', '0'],
    ['opened_hand', 'Hand', '0', '0', '0'],
    ['graspedBy', 'Object', 'Hand', '0', '0'],
    ['detected_object', 'Object', '0', '0', '0'],
    ['locatedAt', 'Object', 'Place', '0', '0'],
    ['in_ContGeneric', 'Object1', 'Object2', '0', '0'],
    ['aboveOf', 'Object1', 'Object2', '0', '0'],
    ['belowOf', 'Object1', 'Object2', '0', '0'],
    ['inFrontOf', 'Object1', 'Object2', '0', '0'],
    ['behind', 'Object1', 'Object2', '0', '0'],
    ['near', 'Object1', 'Object2', '0', '0'],
    ['empty_container', 'Object', '0', '0', '0']]

    def __init__(self):   
        super(KnowledgeInterface, self).__init__()
        '''
        Module for Knowledge manager
        '''
        rospy.loginfo("Initializing KnowledgeInterface...")

        # set the data label name for DB and clear data
        self.msg_store = MessageStoreProxy()
        self.current_state = "/knowledge/current_state" 
        self.msg_store.insert_named("/knowledge/current_state", Problem()) 
        self.msg_store.update_named("/knowledge/current_state", Problem()) 

        # subscriber
        obj_topic = "/perception/objects"
        state_topic = "/context_manager/monitor/reception"
        rospy.Subscriber(obj_topic, Objects, self._callback_objects)
        rospy.Subscriber(state_topic, MonitorServiceRequest, self._callback_states)

        # service
        self.context_srv = rospy.ServiceProxy("/context_manager/monitor/service", MonitorSimilarService)

        # 
        self.is_context_manager = False
        self.detected_objects = []
        self.grasped_object = ''

    def get_current_states(self):        
        msg_state = self.msg_store.query_named(self.current_state, Problem._type)
        return msg_state[0].facts

    def _callback_objects(self, data):
        objects = []
        for obj in data.detected_objects:
            objects.append(obj)
        self.detected_objects = objects
        return

    def _callback_states(self, data):
        if not self.is_context_manager:
            self.is_context_manager = True
        return

    def _convert_format(self, state):
        # convert data format    
        pred = Predicate()    
        pred.name = state.predicate    
        if "_" in pred.name:
            pred.name = pred.name.replace("_",'')

        for i, j in enumerate([state.param1,state.param2,state.param3,state.param4]):
            if "'http://www.arbi.com/ontologies/arbi.owl#" in j:
                j = j.replace("'http://www.arbi.com/ontologies/arbi.owl#",'obj_')
            if "_1'" in j:
                j = j.replace("_1'",'')
            if "'" in j:
                j = j.replace("'",'')
            # if "[" in j and "]" in j:
            #     j = j.replace("[",'')
            #     j = j.replace("]",'')
            #     j = j.split(',')
            #     for idx, string in enumerate(j):
            #         j[idx] = float(string)
            if j:
                pred.args.append(j)
        # exeption only for 'locatedAt'   
        if pred.name=='locatedAt':  
            pred.args[1] = pred.args[0].replace('obj','pos')
            #pred.args[1] = 'pos_' + pred.args[0]
        return pred

    def update(self): 
        '''
        update current states from context manager
        '''
        if self.is_context_manager:
            try:
                rospy.wait_for_service("/context_manager/monitor/service", timeout=1)
                current_states = Problem()
                # query for current state predicates
                req = MonitorSimilarServiceRequest()
                req.status = 100
                req.manager = "TaskManager"
                for pred in self.predicates:   
                    req.predicate = pred[0]
                    req.param1 = pred[1]
                    req.param2 = pred[2]
                    req.param3 = pred[3]
                    req.param4 = pred[4]        
                    res = self.context_srv(req)

                    # check response is not empty
                    for response in res.response:
                        state = self._convert_format(response)
                        current_states.facts.append(state)

                # update data into DB
                self.msg_store.update_named(self.current_state, current_states) 
            except:
                pass
        return
  
    def get_values(self, action):
        '''
        Get metric data from knowledge
        '''        
        plan_req = GetMotionRequest()

        action_name = action.name
        target_body = action.parameters[0]
        rospy.loginfo("Getting metric values from knowledge for %s..."%action_name)

        # TODO: replace dummy knowledge to KGU module        

        # Define target body        
        if rospy.has_param('robot_name'):
            robot_name = rosparam.get_param("/robot_name") 
        else:
            robot_name = 'social_robot'       
        # Define environment
        if rospy.has_param('robot_hw'):
            hw_interface = rosparam.get_param("/robot_hw") 
        else:
            hw_interface = 'vrep'   

        # Set action parameter
        # TODO: assign metric values automatically. now hardcoding....
        if action_name == "open_hand" or action_name == "close_hand":
            plan_req.planner_name = "openclose"
            if  "left" in target_body:
                plan_req.inputs.targetBody = PlannerInputs().LEFT_GRIPPER
            elif "right" in target_body:
                plan_req.inputs.targetBody = PlannerInputs().RIGHT_GRIPPER

            # pose template
            gripper_behavior =''
            gripper_open = [0.0, 0.0]
            gripper_close = [1.0, 0.0]
            if action_name == "open_hand":
                gripper_behavior = gripper_open
            elif action_name == "close_hand":
                gripper_behavior = gripper_close 
            
            gripper_box = [0.0, 0.0]
            gripper_circle = [0.0, 0.3]
            gripper_hook = [0.0, 1.0]

            # set grasp pose: close and box shape
            plan_req.inputs.graspPose = map(lambda x,y: x+y, gripper_behavior, gripper_box) 

        elif action_name == "approach_arm":
            # set approach direction
            plan_req.planner_name = "approach"    
            plan_req.inputs.approachDirection = plan_req.inputs.APPROACH_SIDE
            
            if  "left" in target_body:
                plan_req.inputs.targetBody = PlannerInputs().LEFT_ARM
            elif "right" in target_body:
                plan_req.inputs.targetBody = PlannerInputs().RIGHT_ARM
     
            # Set objects information in environment
            target_object = []
            target_object.append(action.parameters[1])
            plan_req.inputs.targetObject = target_object

            for obj in self.detected_objects:
                plan_req.inputs.obstacle_ids.append(obj.name.data)
                plan_req.inputs.obstacles.append(obj.bb3d)

        elif action_name == "move_arm":
            plan_req.planner_name = "movearm"
            target_joint_state = JointState()
            if robot_name == "skkurobot":
                if "left" in target_body:
                    plan_req.inputs.targetBody = PlannerInputs().LEFT_ARM
                    target_joint_state.name = ['j1_joint', 'j2_joint', 'j3_joint', 'j4_joint', 'j5_joint', 'j6_joint', 'j7_joint']
                elif "right" in target_body:
                    plan_req.inputs.targetBody = PlannerInputs().RIGHT_ARM
                    target_joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            elif robot_name == "social_robot":
                if "left" in target_body:
                    plan_req.inputs.targetBody = PlannerInputs().LEFT_ARM
                    target_joint_state.name = ['LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll']
                elif "right" in target_body:
                    plan_req.inputs.targetBody = PlannerInputs().RIGHT_ARM
                    target_joint_state.name = ['RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll']

            # set obstacles
            for obj in self.detected_objects:
                if not plan_req.inputs.targetObject == obj.name.data: 
                    plan_req.inputs.obstacle_ids.append(obj.name.data)
                    plan_req.inputs.obstacles.append(obj.bb3d)

            # set relocate position (VREP ONLY)
            # TODO:check grapsed object by gripper
            plan_req.inputs.targetObject = ['obj_juice']

            if robot_name == "skkurobot" and hw_interface == 'vrep':
                if "left" in target_body:
                    target_joint_state.position =  [2.029279854423499, 1.0646964200569589, -0.17102309698723056, -1.9245484903916832, 0.46603118745835337, -1.6544888709705696, -1.0367903166736518]
                elif "right" in target_body:
                    target_joint_state.position =  [-0.9078047859959626, 0.7629818976339967, 3.6664837706657227, -1.8813961700996802, 2.509087598490403, -0.9198758121934532]
                

            elif robot_name == "social_robot" and hw_interface == 'vrep':
                if "left" in target_body:
                    target_joint_state.position =  [-0.3130157473543418, -0.5804372548012368, 0.23360350353256745, -1.3802869517948697, 2.1506161276780276, -0.3235098268703248]

            plan_req.inputs.jointGoal = target_joint_state

        elif action_name == "approach_base":
            plan_req.planner_name = action.planner[0]
            plan_req.inputs.targetBody = PlannerInputs().MOBILE_BASE
            
            # Set objects information in environment
            target_object = []
            target_object.append(action.parameters[2])
            plan_req.inputs.targetObject = target_object

            for obj in self.detected_objects:
                plan_req.inputs.obstacle_ids.append(obj.name.data)
                plan_req.inputs.obstacles.append(obj.bb3d)
            
        elif action_name == "move_base":
            plan_req.planner_name = action.planner[0]
            plan_req.inputs.targetBody = PlannerInputs().MOBILE_BASE
            
            # Set objects information in environment
            target_object = []
            target_object.append(action.parameters[2])
            plan_req.inputs.targetObject = target_object

            for obj in self.detected_objects:
                plan_req.inputs.obstacle_ids.append(obj.name.data)
                plan_req.inputs.obstacles.append(obj.bb3d)
            
        elif action_name == "move_around":
            plan_req.planner_name = action.planner[0]
            plan_req.inputs.targetBody = PlannerInputs().MOBILE_BASE
            
            # Set objects information in environment
            target_object = []
            target_object.append(action.parameters[2])
            plan_req.inputs.targetObject = target_object

            for obj in self.detected_objects:
                plan_req.inputs.obstacle_ids.append(obj.name.data)
                plan_req.inputs.obstacles.append(obj.bb3d)

        rospy.loginfo("Assigning is done.")
        return plan_req
