#!/usr/bin/env python
from numpy.core.numeric import require
import rospy
import rosparam
import tf
import math
from mongodb_store.message_store import MessageStoreProxy
from sensor_msgs import msg as sensor_msg
from diagnostic_msgs.msg import KeyValue
from geometry_msgs import msg as geo_msg
from socialrobot_actionlib import msg as sactlib_msg
from socialrobot_msgs import msg as social_robot_msg
from rosjava_custom_srv import msg as rosjava_msg
from rosjava_custom_srv import srv as rosjava_srv
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
    ARM_LENGTH = 0.599 # meter

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
        #obj_topic = "/perception/objects"
        state_topic = "/context_manager/monitor/reception"
        rospy.Subscriber(state_topic, rosjava_msg.MonitorServiceRequest, self._callback_states)

        # service
        self.context_srv = rospy.ServiceProxy("/context_manager/monitor/service", rosjava_srv.MonitorSimilarService)

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
        return True

    def update_current_states(self, current_task):
        problem = sactlib_msg.Problem()
        self.is_start = True
        if self.use_mongodb:            
            # get states from local DB
            msg_state = self.msg_store.query_named(
                "/knowledge/current_state", sactlib_msg.Problem._type
            )
            return msg_state[0].facts
        else:                          
            # knowledge reasoner
            print('Estimating current states.')
            problem = self.update_states()

            # check workspace predicate
            print('Calculating inworkspace predicates.')
            self.check_inworkspace(problem)

            # obstacle reasoner
            if current_task.target_object:
                if current_task.target_object != "SKIP":
                    print('Calculating obstruct predicates about %s.' %current_task.target_object)
                    # relocate service for obstruct predicate of target object
                    if self.check_accessibility(current_task.target_object):
                        for robot_group in self.accessibility_map.keys():
                            for obj in self.accessibility_map[robot_group].keys():
                                if not self.accessibility_map[robot_group][obj][0]: 
                                    # if robot group can't accessible to object
                                    #TODO: hard-coding for fridge demo
                                    if 'fridge' not in self.accessibility_map[robot_group][obj][1]:
                                        pred = sactlib_msg.Predicate()
                                        pred.name = 'obstruct'
                                        pred.args = [robot_group, current_task.target_object, self.accessibility_map[robot_group][obj][1]]
                                        pred.is_negative = False
                                        problem.facts.append(pred)  
                else:   # no solution or response
                    print('No obstruct predicates.')
            else:
                rospy.logwarn('No target object for obstruct predicate.')

            current_task.problem = problem
            return

    def check_inworkspace(self, problem):
        for part in self.social_robot['group'].keys():
            part_shoulder = self.social_robot['group'][part]
            print(part)
            print('arm length =', self.ARM_LENGTH)
            for obj in self.detected_objects:
                obj_center = [obj.bb3d.center.position.x, obj.bb3d.center.position.y]
                dist = math.sqrt((part_shoulder[0] - obj_center[0])**2 + (part_shoulder[1] - obj_center[1])**2)
                print(obj.id)
                print('object dist =', dist)
                #TODO: check distance between object's affordance and robot shoulder
                #for aff in obj.affordance:
                
                if dist < self.ARM_LENGTH:
                    # if object in workspace of specific arm part
                    pred = sactlib_msg.Predicate()
                    pred.name = 'inWorkspace'
                    pred.args = [part, obj.id.replace('obj_','obj_')]
                    pred.is_negative = False
                    problem.facts.append(pred)                
            
                if self.check_predicate(problem, 'inWorkspace', ['obj_right_hand', 'obj_tray']) and self.check_predicate(problem, 'inWorkspace', ['obj_right_hand', 'obj_tray']):
                    self.add_predicate(problem, 'inWorkspace', ['obj_dual_hand', 'obj_tray'])
                # if self.check_predicate(problem, 'inWorkspace', ['obj_right_hand', 'obj_courier_box']) and self.check_predicate(problem, 'inWorkspace', ['obj_right_hand', 'obj_courier_box']):
                #     self.add_predicate(problem, 'inWorkspace', ['obj_dual_hand', 'obj_courier_box'])

        return

    def check_accessibility(self, target):    
        """ calculate accessibility for target object with KIST relocation_node

        Args:
            target ([string]): ID of target
        """
            
        # request to relocation_node
        req = reloc_srv.relocate_env_srvRequest()
        req.robot_height = self.social_robot['height']

        objects = self.detected_objects
        req.N = len(objects)
        target_idx = None
        
        for i, obj in enumerate(objects):
            if target == obj.id:
                target_idx = i
            if obj.id == 'obj_table':    # except table
                req.N -= 1
                req.x_min = 0
                req.x_max = 0.9
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
        req.x_max = 0.9
        req.y_min = -0.5
        req.y_max = 0.5           

        rospy.loginfo("[Socialrobot Knowledge] requesting for accesibility of %s", target)
        try:
            f_check_srv = rospy.ServiceProxy("relocation_srv", reloc_srv.relocate_env_srv)
            for part in self.social_robot['group'].keys():
                req.robot_pose = self.social_robot['group'][part]
                print(req)
                resp = f_check_srv(req)
                print(resp)

                # no relocate solution
                if resp.relocate_id == -1:
                    rospy.logwarn('No relocation solution.')
                else:
                    if resp.accessibility == 1:  # accessible
                        self.accessibility_map[part][target] = [True, '']
                    elif resp.accessibility == -1:   # not accessible
                        obstacle_id = objects[resp.relocate_id].id
                        rospy.logwarn('%s is obstructed by %s for %s grasp', target, obstacle_id, part)
                        self.accessibility_map[part][target] = [False, obstacle_id]  
            return True

        except rospy.ServiceException, e:
            rospy.logerr("Relocation Service call failed: %s" % e)
            return False

    def is_detected(self, object_id):
        """[summary]

        Args:
            object_id ([string]): object ID

        Returns:
            [bool]: check object is already detected
        """        
        # 
        print('target:', object_id)
        for obj in self.detected_objects:
            print(obj.id)
            if obj.id == object_id:
                return True
        return False

    def estimate_target(self, request):
        if request.actionName.lower() == 'grab':
            return "obj_" + request.actionParam1.lower()
        elif request.actionName.lower() == 'handover':
            return "obj_" + request.actionParam1.lower()
        elif request.actionName.lower() == 'open' or request.actionName.lower() == 'close':
            return "SKIP"
        else:
            return None

    def estimate_task(self, request):
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

        # demo 1
        req.actionName = "HandOver"          
        req.actionParam1 = "Gotica"           
        req.actionConstraint = "Left" 
        # demo 2        
        req.actionName = "Grab"              
        req.actionParam1 = "Gotica"             
        req.actionConstraint = "Left"  
        # demo 3       
        req.actionName = Open / Close
        req.actionConstraint = Left / Right / Dual
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
        elif (request.actionConstraint.lower() == 'dual') or (request.actionParam1.lower() == 'dual'):
            robot_part = 'obj_dual_hand'

        if act_name == "handover":
            predicate.name = 'handedover'
            obj = 'obj_'+ act_params[0] 
            predicate.args = [robot_part, obj]
        elif act_name == "grab":
            predicate.name = 'graspedBy'
            obj = 'obj_'+ act_params[0]  
            predicate.args = [robot_part, obj]    
        elif act_name == "open":
            predicate.name = 'openhand'
            predicate.args = [robot_part]  
        elif act_name == "close":
            predicate.name = 'closehand'
            predicate.args = [robot_part] 
            predicate.is_negative = True
        elif act_name == "standby":
            predicate.name = 'standby'
            predicate.args = [robot_part]
        goal_predicates.append(predicate)
        return goal_predicates

    def create_subgoal(self, task):
        goal_predicates = []
        predicate = sactlib_msg.Predicate()

        #TODO:
        if task == "rearrange":
            # set rearrange target from detected objects    
            target = "obj_red_gotica"        
            predicate.name = "rearranged"
            predicate.args = [target]
        goal_predicates.append(predicate)
        return goal_predicates, target

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

    def add_object(self, problem, instance_name, obj_type):
        key_value = KeyValue()
        key_value.key = obj_type		    
        key_value.value = instance_name		
        problem.objects.append(key_value) 

    def add_predicate(self, problem, name, args):
        """
        manually add predicate into fact if don't use knowledge
        """
        predicate = sactlib_msg.Predicate()
        predicate.name = name
        predicate.args = args
        problem.facts.append(predicate) 

    def remove_predicate(self, problem, name, args):
        """
        manually remove predicate from states
        """
        for i, pred in enumerate(problem.facts):
            if pred.name.lower() == name.lower() and pred.args == args:
                problem.facts.pop(i)

    def check_predicate(self, problem, name, args):
        """
        find problem
        """
        for i, pred in enumerate(problem.facts):
            if pred.name.lower() == name.lower() and pred.args == args:
                return True
        return False

    def update_states(self):
        """
        update current states from context manager
        """
        problem = sactlib_msg.Problem()
        try:
            rospy.wait_for_service("/context_manager/monitor/service", timeout=1)            
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
                    print(state)
                    problem.facts.append(state)   
   
            # object type
            rospy.loginfo("[Knowledge Interface] manually add facts.")
            robot_group = ['obj_right_hand', 'obj_left_hand', 'obj_dual_hand', 'obj_socialrobot']
            static_group = ['obj_table','obj_fridge']                 
            dynamic_group = []      

            self.add_predicate(problem, 'type', ['obj_right_hand','Arm'])
            self.add_predicate(problem, 'type', ['obj_left_hand','Arm'])
            self.add_predicate(problem, 'type', ['obj_dual_hand','DualArm'])
            self.add_predicate(problem, 'type', ['obj_socialrobot','Mobile'])

            #hard-coding for dual grasp 
            self.add_predicate(problem, 'locatedAt', ['obj_dual_hand','pos_dual_hand'])
            self.add_predicate(problem, 'locatedAt', ['obj_left_hand','pos_left_hand'])
            self.add_predicate(problem, 'locatedAt', ['obj_right_hand','pos_right_hand'])
                
            #hard-coding for on-physical predicate
            # for obj in self.detected_objects:
            #     if obj.id not in ['obj_right_hand', 'obj_left_hand', 'obj_dual_hand', 'obj_socialrobot', 'obj_table']:
            #         self.add_predicate(problem, 'onPhysical', [obj.id, 'obj_table'])

            has_workspace = False
            detected_objects = []
            for obj in self.detected_objects:
                detected_objects.append(obj.id)
            for obj in self.detected_objects:
                # create dynamic group
                if obj.id in static_group:
                    has_workspace = True   
                if obj.id not in static_group + robot_group:
                    dynamic_group.append(obj.id)
                
                # HARD-CODING for fridge demo
                if obj.id == 'obj_human':                      
                    self.add_object(problem, 'obj_human', 'Object')    
                    self.add_object(problem, 'pos_human', 'Position')
                    self.add_predicate(problem, 'locatedAt', ['obj_human','pos_human'])
                if obj.id == 'obj_fridge':
                    self.add_predicate(problem, 'openedhand', ['obj_dual_hand'])
                    self.add_predicate(problem, 'largeSized', ['obj_fridge'])
                    self.add_predicate(problem, 'emptyhand', ['obj_right_hand'])
                    self.add_predicate(problem, 'emptyhand', ['obj_left_hand'])
                    self.add_predicate(problem, 'detectedobject', ['obj_white_gotica'])
                    self.add_predicate(problem, 'locatedAt', ['obj_white_gotica','pos_white_gotica'])
                    if 'obj_red_gotica' in detected_objects:
                        self.add_predicate(problem, 'largeSized', ['obj_tray'])   
                        self.add_predicate(problem, 'inContGeneric', ['obj_red_gotica','obj_fridge'])
                        self.add_predicate(problem, 'inContGeneric', ['obj_white_gotica','obj_fridge'])
                        self.add_predicate(problem, 'obstruct', ['obj_right_hand','obj_white_gotica','obj_red_gotica'])
                        self.add_predicate(problem, 'locatedAt', ['obj_fridge','pos_fridge'])
                        self.add_predicate(problem, 'locatedAt', ['obj_red_gotica','pos_red_gotica'])
                        self.add_predicate(problem, 'detectedobject', ['obj_red_gotica'])
                        self.add_predicate(problem, 'inWorkspace', ['obj_right_hand', 'obj_white_gotica'])
                    self.remove_predicate(problem, 'locatedAt', ['obj_left_hand', 'pos_left_hand'])
                    if rospy.has_param('fridge_isopen'):
                        if rospy.get_param('fridge_isopen') == True:
                            self.add_predicate(problem, 'openedContainer', ['obj_fridge'])
                if obj.id == 'obj_courier_box':
                    if rospy.has_param('user_age'):
                        if rospy.get_param('user_age') == 'young':
                            self.add_predicate(problem, 'largeSized', ['obj_courier_box'])
                    else:
                        self.add_predicate(problem, 'largeSized', ['obj_courier_box'])
                    self.add_predicate(problem, 'locatedAt', ['obj_courier_box','pos_courier_box'])
            
            # if workspace is not detected, add virtual table
            if not has_workspace:
                self.add_predicate(problem, 'detectedobject', ['obj_table'])
                self.add_predicate(problem, 'locatedAt', ['obj_table','pos_table'])
                for dym_obj in dynamic_group:
                    self.add_predicate(problem, 'onPhysical', [dym_obj, 'obj_table'])

            if len(problem.facts)>0:
                # update data into DB
                if self.use_mongodb:                    
                    self.msg_store.update_named("/knowledge/current_state", problem)
                return problem
            else:
                rospy.logerr("[Socialrobot Interface] current facts.")
                return problem
            
        except Exception:
            rospy.logerr("[Socialrobot Interface] service call to context manager is failed.")
            return problem

    def compare_states(self, facts, effects):        
        '''
        compare action effect and current states
        '''
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

    def _get_object_value(self, requirements=None, symbolic_parameters=None):   
        """
        input: symbolic_parameters (list)
        """  
        detected_objects = self.detected_objects
        obj_ids = []
        metric_values = []
        for obj in detected_objects: 
            obj_ids.append(obj.id)

        # find object from symbolic parameters
        if symbolic_parameters:
            for param in symbolic_parameters:     
                # detected from perception manager       
                if param in obj_ids:
                    idx = obj_ids.index(param)                
                    metric_values.append(detected_objects[idx])

                # robot part
                elif 'left' in param:
                    metric_values.append(social_robot_msg.Behavior().LEFT_ARM)
                elif 'right' in param:
                    metric_values.append(social_robot_msg.Behavior().RIGHT_ARM)
                elif 'dual' in param:
                    metric_values.append(social_robot_msg.Behavior().BOTH_ARM)
                elif 'robot' in param:
                    metric_values.append(social_robot_msg.Behavior().MOBILE_BASE)
                else:
                    rospy.logwarn("cannot find object metric value %s", param)
                    metric_values.append(social_robot_msg.Object())

        # find object from object type
        if requirements == 'dynamic_object':
            #metric_values = detected_objects
            for obj in detected_objects:
                # TODO: remove
                if 'obj_human' not in obj.id:
                    metric_values.append(obj)
            
        elif requirements == 'static_object':
            pass                   
        return metric_values

    def _get_position_value(self, requirements=None, symbolic_parameters=None):
        detected_objects = self.detected_objects
        obj_ids = []
        metric_values = []
        for obj in detected_objects: 
            obj_ids.append(obj.id)

        # get object position values
        if symbolic_parameters:
            for pos_id in symbolic_parameters:
                if 'pos_' in pos_id:
                    # get position from object   
                    obj_id = pos_id.replace('pos_','obj_')  
                    if obj_id in obj_ids:
                        idx = obj_ids.index(obj_id)           
                        obj = detected_objects[idx]
                        # convert object to position
                        pos = social_robot_msg.Position()
                        pos.waypoint = obj_id
                        pos.pose = obj.bb3d.center
                        metric_values.append(pos)
                else: 
                    # convert string value into float
                    pose_2d = [float(x) for x in pos_id[1:-1].split(',')] #[x,y,theta]
                    quat = tf.transformations.quaternion_from_euler(0, 0, pose_2d[2])
                    pos = social_robot_msg.Position()
                    pos.pose.position.x = pose_2d[0]
                    pos.pose.position.y = pose_2d[1]
                    pos.pose.orientation.x = quat[0]
                    pos.pose.orientation.y = quat[1]
                    pos.pose.orientation.z = quat[2]
                    pos.pose.orientation.w = quat[3]
                    metric_values.append(pos)


        return metric_values

    def assign_metric_value(self, requirements):
        """
        'requirements':{'parameter':[],
                        'type':[],
                        'symbolic':[],
                        'metric':[]}}
        """          

        # find metric value of symbolic value
        for i, req in enumerate(requirements['parameter']):
            param_type = param_sym = None
            if requirements['type'][i]:
                param_type = requirements['type'][i][0]
            if requirements['symbolic'][i]:
                param_sym = requirements['symbolic'][i]
                
            if param_type == 'object':
                metric_values = self._get_object_value(symbolic_parameters=param_sym)
                requirements['metric'][i] = metric_values

            elif param_type == 'position':
                metric_values = self._get_position_value(symbolic_parameters=param_sym)
                requirements['metric'][i] = metric_values

            elif req == 'dynamic_object' or req == 'static_object':
                metric_values = self._get_object_value(requirements=req)
                requirements['metric'][i] = metric_values

    def assign_social_constraints(self, task_plan):
        """ append constraints to each action

        Args:
            task_plan ([socialrobot_actionlib/Action[]]): task plan sequence
        """
        # TODO: remove hard-coding
        #for act in task_plan:
            


        return True
