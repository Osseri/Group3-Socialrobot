#!/usr/bin/env python
import rospy
import rosparam
import math
from mongodb_store.message_store import MessageStoreProxy
from sensor_msgs import msg as sensor_msg
from diagnostic_msgs.msg import KeyValue
from geometry_msgs import msg as geo_msg
from socialrobot_actionlib import msg as sactlib_msg
from socialrobot_perception_msgs import msg as perception_msg
from socialrobot_behavior import msg as behavior_msg
from socialrobot_behavior import srv as behavior_srv
from rosjava_custom_srv import msg as rosjava_msg
from rosjava_custom_srv import srv as rosjava_srv
from socialrobot_perception_msgs.srv import *
from rearrange_node import msg as rearr_msg
from rearrange_node import srv as rearr_srv
from relocation_node import srv as reloc_srv
from interface import InterfaceBase


class KnowledgeInterface(InterfaceBase):
    # TODO: remove hard coding
    PREDICATES = [
        ["on_Physical", "Top", "Bottom", "0", "0"],
        ["empty_hand", "Hand", "0", "0", "0"],
        ["opened_hand", "Hand", "0", "0", "0"],
        ["graspedBy", "Object", "Hand", "0", "0"],
        ["detected_object", "Object", "0", "0", "0"],
        ["locatedAt", "Object", "Place", "0", "0"],
        ["in_ContGeneric", "Object1", "Object2", "0", "0"],
        ["aboveOf", "Object1", "Object2", "0", "0"],
        ["belowOf", "Object1", "Object2", "0", "0"],
        ["inFrontOf", "Object1", "Object2", "0", "0"],
        ["behind", "Object1", "Object2", "0", "0"],
        ["near", "Object1", "Object2", "0", "0"],
        ["empty_container", "Object", "0", "0", "0"],
    ]
    EFFECT_EXCEPTION = ['locatedat','rearranged','transferred','standby']
    ARM_LENGTH = 0.55 # meter

    def __init__(self):
        super(KnowledgeInterface, self).__init__()
        """
        Module for Knowledge manager
        """
        rospy.loginfo("Initializing KnowledgeInterface...")

        # set the data label name for DB and clear data
        self.use_mongodb = False
        self.msg_store = None
        self.current_state = None
        self.is_start =False
        if rospy.has_param("/use_mongodb"):
            self.use_mongodb = rospy.get_param("/use_mongodb")
        if self.use_mongodb:
            self.msg_store = MessageStoreProxy()
            self.msg_store.insert_named("/knowledge/current_state", sactlib_msg.Problem())
            self.msg_store.update_named("/knowledge/current_state", sactlib_msg.Problem())

        # subscriber
        obj_topic = "/perception/objects"
        state_topic = "/context_manager/monitor/reception"
        rospy.Subscriber(obj_topic, perception_msg.Objects, self._callback_objects)
        rospy.Subscriber(state_topic, rosjava_msg.MonitorServiceRequest, self._callback_states)

        # service
        self.context_srv = rospy.ServiceProxy("/context_manager/monitor/service", rosjava_srv.MonitorSimilarService)
        self.scene_srv = rospy.ServiceProxy("/motion_plan/get_scene_objects", GetObjects)

        self.is_context_manager = False
        self.constraint = 0
        self.detected_objects = []
        self.grasped_object = ""
        self.accessibility_map = {"obj_left_hand": {}, "obj_right_hand": {}}
        self.social_robot = {"height": 0.075,
                    # shoulder's x-y direction based on /base_footprint
                      "group": {'obj_left_hand': [0.05, 0.3],     
                                "obj_right_hand": [0.05, -0.3]}} 

    def reset(self):
        self.detected_objects = []
        self.grasped_object = ""
        self.accessibility_map = {"obj_left_hand": {}, "obj_right_hand": {}}
        self.current_state = None
        self.is_start =False
        while(not self.detected_objects):
            pass
        return True

    def get_current_states(self, target_object=None):
        problem = sactlib_msg.Problem()
        self.is_start = True
        if self.use_mongodb:            
            # get states from local DB
            msg_state = self.msg_store.query_named(
                self.current_state, sactlib_msg.Problem._type
            )
            return msg_state[0].facts
        else:                          
            # knowledge reasoner
            if self.update_states():
                problem = self.current_state

                # check workspace
                self.check_inworkspace(problem)

                # obstacle reasoner
                if target_object:
                    # relocate service for obstruct predicate of target object
                    if self.check_accessibility(target_object, self.detected_objects):
                        for robot_group in self.accessibility_map.keys():
                            for obj in self.accessibility_map[robot_group].keys():
                                if not self.accessibility_map[robot_group][obj][0]: 
                                    # if robot group can't accessible to object
                                    pred = sactlib_msg.Predicate()
                                    pred.name = 'obstruct'
                                    pred.args = [robot_group, target_object, self.accessibility_map[robot_group][obj][1]]
                                    pred.is_negative = False
                                    problem.facts.append(pred)                        
                    else:
                        return False, None
                return True, problem
            else:
                return False, None

    def check_inworkspace(self, problem):
        for part in self.social_robot['group'].keys():
            part_shoulder = self.social_robot['group'][part]
            print('arm length =', self.ARM_LENGTH)
            for obj in self.detected_objects:
                obj_center = [obj.bb3d.center.position.x, obj.bb3d.center.position.y]
                dist = math.sqrt((part_shoulder[0] - obj_center[0])**2 + (part_shoulder[1] - obj_center[1])**2)
                print(obj.name.data)
                print('object dist =', dist)
                if dist < self.ARM_LENGTH:
                    # if object in workspace of specific arm part
                    pred = sactlib_msg.Predicate()
                    pred.name = 'inWorkspace'
                    pred.args = [part, obj.name.data.replace('obj_','obj_')]
                    pred.is_negative = False
                    problem.facts.append(pred)

    def check_accessibility(self, target, obstacles):    
        """ calculate accessibility for target object with KIST relocation_node

        Args:
            target ([string]): ID of target
            obstacles ([socialrobot_perception_msgs/Objects]): detected object list
        """
            
        # request to relocation_node
        req = reloc_srv.relocate_env_srvRequest()
        req.robot_height = self.social_robot['height']

        objects = self.detected_objects
        req.N = len(objects)
        target_idx = None
        for i, obj in enumerate(objects):
            if target == obj.name.data:
                target_idx = i
            if obj.name.data == 'obj_table':    # except table
                req.N -= 1
                req.x_min = 0
                req.x_max = 0.5
                req.y_min = -0.5
                req.y_max = 0.5 
            else:
                req.R.append(max(obj.bb3d.size.x, obj.bb3d.size.y))
                req.H.append(obj.bb3d.size.z)
                req.X.append(obj.bb3d.center.position.x)
                req.Y.append(obj.bb3d.center.position.y)
        req.target_id = target_idx

        # workspace (consider table size)
        req.x_min = 0
        req.x_max = 0.5
        req.y_min = -0.5
        req.y_max = 0.5           

        rospy.loginfo("[Socialrobot Interface] wait for relocation service...")
        rospy.wait_for_service("relocation_srv")
        rospy.loginfo("[Socialrobot Interface] requesting for accesibility of target object...")
        try:
            f_check_srv = rospy.ServiceProxy("relocation_srv", reloc_srv.relocate_env_srv)
            for part in self.social_robot['group'].keys():
                req.robot_pose = self.social_robot['group'][part]
                resp = f_check_srv(req)
                # no relocate solution
                if resp.relocate_id == -1:
                    rospy.logwarn('No relocation solution.')
                else:
                    if resp.accessibility == 1:  # accessible
                        self.accessibility_map[part][target] = [True, '']
                    elif resp.accessibility == -1:   # not accessible
                        self.accessibility_map[part][target] = [False, objects[resp.relocate_id].name.data]
            return True

        except rospy.ServiceException, e:
            print("Relocation Service call failed: %s" % e)

    def is_detected(self, object_id):
        """[summary]

        Args:
            object_id ([string]): object ID

        Returns:
            [bool]: check object is already detected
        """
        # 
        for obj in self.detected_objects:
            if obj.name.data == object_id:
                return True
        return False

    def estimate_target(self, request):
        if request.actionName.lower() == 'grab':
            return "obj_" + request.actionParam1.lower()
        elif request.actionName.lower() == 'handover':
            return "obj_" + request.actionParam1.lower()
        else:
            return None

    def estimate_goal(self, request):
        ''' 
        string actionName
        string actionParam1
        string actionParam2
        string actionParam3
        string actionParam4
        string actionParam5
        string actionConstraint
        int32 actionID
        ---
        int32 result 
        '''
        goal_predicates = []
        predicate = sactlib_msg.Predicate()
        robot_part = None
        act_name = request.actionName.lower()
        act_params = [request.actionParam1.lower(),
                        request.actionParam2.lower(),
                        request.actionParam3.lower(),
                        request.actionParam4.lower(),
                        request.actionParam5.lower()]
        
        if (request.actionConstraint.lower() == 'left') or (request.actionParam1.lower() == 'left'):
            robot_part = 'obj_left_hand'
        elif (request.actionConstraint.lower() == 'right') or (request.actionParam1.lower() == 'right'):
            robot_part = 'obj_right_hand'

        if act_name == "move":
            predicate.name = 'locatedAt'
            predicate.args = ['robot_base', act_params[0]]            
        elif act_name == "handover":
            predicate.name = 'transferred'
            obj = 'obj_'+ act_params[0] 
            predicate.args = [robot_part, obj]
        elif act_name == "grab":
            predicate.name = 'graspedBy'
            obj = 'obj_'+ act_params[0]  
            predicate.args = [robot_part, obj]    
        elif act_name == "open":
            predicate.name = 'openedHand'
            predicate.args = [robot_part]  
        elif act_name == "close":
            predicate.name = 'openedHand'
            predicate.args = [robot_part] 
            predicate.is_negative = True
        elif act_name == "zeropose":
            predicate.name = 'standby'
            predicate.args = [robot_part]
        goal_predicates.append(predicate)
        return goal_predicates

    def create_subgoal(self, task):
        goal_predicates = []
        predicate = sactlib_msg.Predicate()

        if task == "rearrange":
            # set rearrange target from detected objects    
            target = "obj_red_gotica"        
            predicate.name = "rearranged"
            predicate.args = [target]
        goal_predicates.append(predicate)
        return goal_predicates, target
    
    def _callback_objects(self, data):
        # if tracking mode
        if rospy.has_param('/object_tracking') and rospy.get_param('/object_tracking')==True:
            self.detected_objects = list(data.detected_objects)
        else:
            # if not tracked, update objects only first time
            if not self.is_start:
                self.detected_objects = list(data.detected_objects)
            else:
                pass

    def _callback_states(self, data):
        # if not self.is_context_manager:
        #     self.is_context_manager = True
        self.is_context_manager = True

    def _convert_format(self, state):
        # convert data format
        pred = sactlib_msg.Predicate()
        pred.name = state.predicate
        if "_" in pred.name:
            pred.name = pred.name.replace("_", "")

        for i, j in enumerate([state.param1, state.param2, state.param3, state.param4]):
            if "'http://www.arbi.com/ontologies/arbi.owl#" in j:
                j = j.replace("'http://www.arbi.com/ontologies/arbi.owl#", "obj_")
            if "_1'" in j:
                j = j.replace("_1'", "")
            if "'" in j:
                j = j.replace("'", "")
            if j:
                pred.args.append(j)
        # exeption only for 'locatedAt'
        if pred.name == "locatedAt":
            pred.args[1] = pred.args[0].replace("obj", "pos")
        return pred

    def update_states(self):
        """
        update current states from context manager
        """
        try:
            rospy.wait_for_service("/context_manager/monitor/service", timeout=1)
            current_states = sactlib_msg.Problem()
            # query for current state predicates
            req = rosjava_srv.MonitorSimilarServiceRequest()
            req.status = 100
            req.manager = "TaskManager"
            for pred in self.PREDICATES:
                print("Reasoning for %s predicate." %pred[0])
                req.predicate = pred[0]
                req.param1 = pred[1]
                req.param2 = pred[2]
                req.param3 = pred[3]
                req.param4 = pred[4]
                res = self.context_srv(req)
                # check response is not empty
                for response in res.response:
                    state = self._convert_format(response)
                    if 'obj_bakey' not in state.args and 'pos_bakey' not in state.args:
                        current_states.facts.append(state)   

            # add additional facts manually            
            temp_type = sactlib_msg.Predicate()
            temp_type.name = 'type'
            temp_type.args = ['obj_right_hand','Gripper']
            current_states.facts.append(temp_type) 
            temp_type = sactlib_msg.Predicate()
            temp_type.name = 'type'
            temp_type.args = ['obj_left_hand','Gripper']
            current_states.facts.append(temp_type) 
            temp_type = sactlib_msg.Predicate()
            temp_type.name = 'type'
            temp_type.args = ['obj_socialrobot','Mobile']
            current_states.facts.append(temp_type) 
            # temp_type = sactlib_msg.Predicate()
            # temp_type.name = 'incontgeneric'
            # temp_type.args = ['obj_fridge','obj_milk']
            # current_states.facts.append(temp_type) 
            # temp_type = sactlib_msg.Predicate()
            # temp_type.name = 'locatedat'
            # temp_type.args = ['obj_fridge','pos_fridge']
            # current_states.facts.append(temp_type) 
            # temp_obj = KeyValue()
            # temp_obj.key = 'object'
            # temp_obj.value = 'obj_fridge'
            # current_states.objects.append(temp_obj) 
            # temp_obj = KeyValue()
            # temp_obj.key = 'position'
            # temp_obj.value = 'pos_fridge'
            # current_states.objects.append(temp_obj) 
                    
            if len(current_states.facts)>0:
                self.current_state = current_states
                # update data into DB
                if self.use_mongodb:
                    self.msg_store.update_named(self.current_state, current_states)
                return True
            else:
                return False
            
        except Exception:
            rospy.logerr("[Socialrobot Interface] service call to context manager is failed.")
            return False

    def compare_states(self, facts, effects):        
        inefficient_condition = sactlib_msg.Condition()
        fact_condition = sactlib_msg.Condition()

        # convert predicate to fluent
        for fact in facts:
            fluent = sactlib_msg.Fluent(predicate=fact.name.lower(), args=fact.args)
            fact_condition.positives.append(fluent)    

        for p in effects.positives:
            if p.predicate.lower() not in self.EFFECT_EXCEPTION:
                if p not in fact_condition.positives:                  
                    inefficient_condition.positives.append(p)

        for n in effects.negatives:
            if n.predicate.lower() not in self.EFFECT_EXCEPTION:
                if n in fact_condition.positives:
                    inefficient_condition.negatives.append(n)

        return inefficient_condition 

    def get_values(self, action):
        """
        Get metric data from knowledge
        """
        plan_req = behavior_srv.GetMotionRequest()

        action_name = action.name
        target_body = action.parameters[0]
        rospy.loginfo("Getting metric values from knowledge for %s..." % action_name)

        # update object data
        if rospy.has_param('/object_tracking') and rospy.get_param('/object_tracking')==False:
            rospy.loginfo("Getting object information from moveit scene.")
            req = GetObjectsRequest()
            scene_objects = self.scene_srv(req)
            if scene_objects.objects:
                self.detected_objects = list(scene_objects.objects)

        # Define target body
        if rospy.has_param("robot_name"):
            robot_name = rosparam.get_param("/robot_name")
        # Define environment
        if rospy.has_param("robot_hw"):
            hw_interface = rosparam.get_param("/robot_hw")

        # Set action parameter
        # TODO: assign metric values automatically. now hardcoding....
        if action_name in ("open_hand", "close_hand"):
            plan_req.planner_name = "openclose"
            if "left" in target_body:
                plan_req.inputs.targetBody = behavior_msg.PlannerInputs().LEFT_GRIPPER
            elif "right" in target_body:
                plan_req.inputs.targetBody = behavior_msg.PlannerInputs().RIGHT_GRIPPER

            # pose template
            gripper_behavior = [0.0 if action_name == "open_hand" else 1.0, 0.0]

            gripper_box = [0.0, 0.0]
            # gripper_circle = [0.0, 0.3]
            # gripper_hook = [0.0, 1.0]

            # set grasp pose: close and box shape
            plan_req.inputs.graspPose = map(
                lambda x, y: x + y, gripper_behavior, gripper_box
            )

        elif action_name == "approach_arm":
            # set approach direction
            plan_req.planner_name = "approach"

            if "left" in target_body:
                plan_req.inputs.targetBody = behavior_msg.PlannerInputs().LEFT_ARM
            elif "right" in target_body:
                plan_req.inputs.targetBody = behavior_msg.PlannerInputs().RIGHT_ARM

            plan_req.inputs.approachDirection = plan_req.inputs.APPROACH_SIDE
            if action.parameters[3] == 'objecttop':
                plan_req.inputs.approachPoint = plan_req.inputs.APPROACH_TOP
            elif action.parameters[3] == 'objectbottom':
                plan_req.inputs.approachPoint = plan_req.inputs.APPROACH_BOTTOM

            elif action.parameters[3] == 'objecthandover':
                plan_req.inputs.approachPoint = plan_req.inputs.APPROACH_BOTTOM
                if "left" in target_body:
                    plan_req.inputs.targetBody = behavior_msg.PlannerInputs().LEFT_ARM_WITHOUT_WAIST
                elif "right" in target_body:
                    plan_req.inputs.targetBody = behavior_msg.PlannerInputs().RIGHT_ARM_WITHOUT_WAIST
            elif action.parameters[3] == 'objecttakeover':
                plan_req.inputs.approachPoint = plan_req.inputs.APPROACH_TOP
                if "left" in target_body:
                    plan_req.inputs.targetBody = behavior_msg.PlannerInputs().LEFT_ARM_WITHOUT_WAIST
                elif "right" in target_body:
                    plan_req.inputs.targetBody = behavior_msg.PlannerInputs().RIGHT_ARM_WITHOUT_WAIST
                
            # Set objects information in environment
            plan_req.inputs.targetObject = [
                action.parameters[1],
            ]

            for obj in self.detected_objects:
                plan_req.inputs.obstacle_ids.append(obj.name.data)
                plan_req.inputs.obstacles.append(obj.bb3d)

            # TODO:fridge demo
            # handle grasp pose
            # target_pose.position.x = 0.378509
            # target_pose.position.y = -0.06253
            # target_pose.position.z = 0.836745
            # target_pose.orientation.x = 0
            # target_pose.orientation.y = 0.707107
            # target_pose.orientation.z = 0
            # target_pose.orientation.w = 0.707107

        elif action_name == "standby":
            plan_req.planner_name = "standby"
            target_joint_state = sensor_msg.JointState()
            if robot_name == "social_robot":
                if "left" in target_body:
                    plan_req.inputs.targetBody = behavior_msg.PlannerInputs().LEFT_ARM
                    target_joint_state.name = [
                        'Waist_Roll', 
                        'Waist_Pitch',
                        "LShoulder_Pitch",
                        "LShoulder_Roll",
                        "LElbow_Pitch",
                        "LElbow_Yaw",
                        "LWrist_Pitch",
                        "LWrist_Roll",
                    ]
                    target_joint_state.position = [0, 0.48, 0, -1.22, 0, 0, 0, 0]
                elif "right" in target_body:
                    plan_req.inputs.targetBody = behavior_msg.PlannerInputs().RIGHT_ARM
                    target_joint_state.name = [
                        'Waist_Roll', 
                        'Waist_Pitch',
                        "RShoulder_Pitch",
                        "RShoulder_Roll",
                        "RElbow_Pitch",
                        "RElbow_Yaw",
                        "RWrist_Pitch",
                        "RWrist_Roll",
                    ]
                    target_joint_state.position = [0, 0.48, 0, 1.22, 0, 0, 0, 0]
                                         
            # only consider table object
            for obj in self.detected_objects:
                if obj.name.data == 'obj_table':
                    plan_req.inputs.obstacle_ids.append(obj.name.data)
                    plan_req.inputs.obstacles.append(obj.bb3d)
                    
            plan_req.inputs.jointGoal = target_joint_state

        elif action_name in ("approach_base", "move_base", "move_around"):
            plan_req.planner_name = action.planner[0]
            plan_req.inputs.targetBody = behavior_msg.PlannerInputs().MOBILE_BASE

            # Set objects information in environment
            plan_req.inputs.targetObject = [
                action.parameters[2],
            ]

            for obj in self.detected_objects:
                plan_req.inputs.obstacle_ids.append(obj.name.data)
                plan_req.inputs.obstacles.append(obj.bb3d)


        elif action_name == 'relocate_obstacle':
            plan_req.planner_name = "relocate"
            
            if "left" in target_body:
                plan_req.inputs.targetBody = behavior_msg.PlannerInputs().LEFT_ARM
            elif "right" in target_body:
                plan_req.inputs.targetBody = behavior_msg.PlannerInputs().RIGHT_ARM

            # Set objects information in environment
            plan_req.inputs.targetObject = [
                action.parameters[1],
                action.parameters[2],
            ]
            for obj in self.detected_objects:
                plan_req.inputs.obstacle_ids.append(obj.name.data)
                plan_req.inputs.obstacles.append(obj.bb3d)

        elif action_name == 'rearrange_object':
            plan_req.planner_name = "relocate"
            
            if "left" in target_body:
                plan_req.inputs.targetBody = behavior_msg.PlannerInputs().LEFT_ARM
            elif "right" in target_body:
                plan_req.inputs.targetBody = behavior_msg.PlannerInputs().RIGHT_ARM

            # Set objects information in environment
            plan_req.inputs.targetObject = [
                '',
                action.parameters[1]
            ]
            for obj in self.detected_objects:
                plan_req.inputs.obstacle_ids.append(obj.name.data)
                plan_req.inputs.obstacles.append(obj.bb3d)

        elif action_name == 'transfer_object':
            plan_req.planner_name = "transfer"

            if "left" in target_body:
                plan_req.inputs.targetBody = behavior_msg.PlannerInputs().LEFT_ARM
            elif "right" in target_body:
                plan_req.inputs.targetBody = behavior_msg.PlannerInputs().RIGHT_ARM
            
            # set obstacles
            for obj in self.detected_objects:
                if not plan_req.inputs.targetObject == obj.name.data:
                    plan_req.inputs.obstacle_ids.append(obj.name.data)
                    plan_req.inputs.obstacles.append(obj.bb3d)

            plan_req.inputs.targetObject =[action.parameters[1]]

        elif action_name == 'handover_object':
            plan_req.planner_name = "handover"

            if "left" in target_body:
                plan_req.inputs.targetBody = behavior_msg.PlannerInputs().LEFT_ARM
            elif "right" in target_body:
                plan_req.inputs.targetBody = behavior_msg.PlannerInputs().RIGHT_ARM
            
            # set obstacles
            for obj in self.detected_objects:
                if not plan_req.inputs.targetObject == obj.name.data:
                    plan_req.inputs.obstacle_ids.append(obj.name.data)
                    plan_req.inputs.obstacles.append(obj.bb3d)

            plan_req.inputs.targetObject =[action.parameters[2]]

        elif action_name == 'move_arm':
            plan_req.planner_name = "movearm"
            target_joint_state = sensor_msg.JointState()
            if "left" in target_body:
                pass
            elif "right" in target_body:
                plan_req.inputs.targetBody = behavior_msg.PlannerInputs().RIGHT_ARM
                target_pose = geo_msg.Pose()
                target_pose.position.x = 0.378509
                target_pose.position.y = -0.06253
                target_pose.position.z = 0.836745
                target_pose.orientation.x = 0
                target_pose.orientation.y = 0.707107
                target_pose.orientation.z = 0
                target_pose.orientation.w = 0.707107
                plan_req.inputs.poseGoal = target_pose

        elif action_name == 'open_door':
            plan_req.planner_name = "opendoor"
            plan_req.inputs.targetBody = plan_req.inputs.MOBILE_BASE


        rospy.loginfo("Assigning is done.")
        return plan_req
