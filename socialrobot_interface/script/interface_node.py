#!/usr/bin/env python
# [[Python Style Guide]]
# @ RISE LAB
# https://rise-lab.atlassian.net/wiki/spaces/ENG/pages/118358105/RISE+Python+3+Style+Guide
# @ Google
# https://google.github.io/styleguide/pyguide.html

# Python libs
import six
import logging

# ROS Frameworks
import rospy
import rosparam
import actionlib
from actionlib_msgs import msg as actionlib_msg
from socialrobot_actionlib.msg import *
from std_msgs.msg import String
from std_srvs.srv import *

# Third-party libs

# Socialrobot modules
import perception
import actionlibrary
import task
import behavior
import knowledge
from socialrobot_interface import msg as interface_msg
from socialrobot_interface import srv as interface_srv
from socialrobot_task import srv as task_srv
from socialrobot_msgs import msg as social_robot_msg
from socialrobot_msgs import srv as social_robot_srv
from socialrobot_actionlib import msg as sactlib_msg
from socialrobot_behavior import srv as behavior_srv

class CustomFormatter(logging.Formatter):
    """Logging Formatter to add colors and count warning / errors"""
    """Usage: 
        logger.debug("debug message")
        logger.info("info message")
        logger.warning("warning message")
        logger.error("error message")
        logger.critical("critical message")"""

    cyan = "\x1b[36m"
    grey = "\x1b[38;21m"
    green = "\x1b[32m"
    yellow = "\x1b[33;21m"
    red = "\x1b[31;21m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    #format = "%(asctime)s - %(name)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)"
    format = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    FORMATS = {
        logging.DEBUG: grey + format + reset,
        logging.INFO: green + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)
logger = logging.getLogger("SocialrobotInterface")

class Singleton(type):

    """Singleton pattern meta"""

    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class Task():
    """ Task class"""
    def __init__(self):
        self.is_finished = False
        self.is_failed = False
        self.plan = []
        self.task_states = []
        self.goal_predicates = []
        self.problem = []
        self.command = None
        self.current_action = None
        self.target_object = None
   
    def set_finish(self):
        '''
        set task execution result as FINISH
        '''
        self.is_finished = True

    def set_fail(self):
        '''
        set task execution result as FAIL
        '''
        self.is_failed = True

    def set_next_action(self):
        '''
        set current action as first queue of primitive action list
        '''
        if len(self.plan[0]['primitive_actions'])>0:
            self.current_action = {'action':self.plan[0]['primitive_actions'].pop(0), 
                                'behavior':social_robot_msg.Behavior(),
                                'requirements':{'parameter':[],
                                                'type':[],
                                                'symbolic':[],
                                                'metric':[]}}
            return True
        else:
            return False


class SocialrobotInterface(six.with_metaclass(Singleton)):

    """Socialrobot Interface Definition

    This is a singleton class.
    """
    MAX_TASK_ITER = 0

    def __init__(self):
        # create logger with 'spam_application'
        logger.setLevel(logging.DEBUG)

        # create console handler with a higher log level
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        ch.setFormatter(CustomFormatter())
        logger.addHandler(ch)

        # Init ROS node
        rospy.init_node("socialrobot_interface")
        logger.info("Service Started!")        

        #
        self.is_cmd_received = False
        self.is_sim = False
        self.task_string = None        
        self.task_list = []

        self.motion = None
        self.retry_count = 0
        self.object_tracking = False
        self.use_knowledge = False
        self.use_reasoner = True
        
        self.command_dict = {
            # task prepairing step
            "Setup": self._cmd_setup,
            "GetCommand": self._cmd_get_command,
            "TaskSuccess": self._cmd_set_success,
            "TaskFail": self._cmd_set_fail,
            "TaskRetry": self._cmd_set_retry,
            "GetTask": self._cmd_get_task,
            "ActivePerception": self._cmd_active_perception,
            "GetState": self._cmd_get_state,
            "TaskPlan": self._cmd_get_plan,
            "GetConst": self._cmd_get_constraints,
            # task performance step
            "DecodePlan": self._cmd_decode_plan,
            "UpdateStates": self._cmd_update_states,
            #
            "CheckAction": self._cmd_check_action,
            "GetSymbolicValues": self._cmd_get_symbolic_values,
            "GetMetricValues": self._cmd_get_metric_values,
            "UpdateValues": self._cmd_update_values,
            "CheckPrecond": self._cmd_check_precond,
            "GetMotion": self._cmd_get_motion,
            "ExecuteAction": self._cmd_execute_action,
            "CheckEffect": self._cmd_check_effect,
        }

        # get params
        self.object_tracking = rospy.get_param("/object_tracking", default=True)
        self.moveit_start = rospy.get_param("/moveit_status")
        if rospy.get_param("/robot_hw", default='hardware') == 'vrep':
            self.is_sim = True

        self.sub_moveit = rospy.Subscriber(
            "/move_group/status",
            actionlib_msg.GoalStatusArray,
            self._callback_moveit_status,
        )

        # wait if moveit is initializing
        logger.info("Waiting moveit initialization...")
        if rospy.has_param("/moveit_status"):
            while not self.moveit_start:
                self.moveit_start = rosparam.get_param("/moveit_status")
                rospy.sleep(2.0)
        logger.info("Moveit is connected.")

        # load interface modules
        self.perception_interface = perception.PerceptionInterface()
        self.actionlib_interface = actionlibrary.ActionLibInterface()
        self.task_interface = task.TaskInterface()
        self.behavior_interface = behavior.BehaviorInterface()
        self.knowledge_interface = knowledge.KnowledgeInterface()
        logger.info("Interface modules are initialized.")

        # action server
        self._action_server = actionlib.SimpleActionServer(
            "/socialrobot/system",
            interface_msg.SystemAction,
            execute_cb=self._system_action_callback,
            auto_start=False,
        )
        #self._action_server.register_goal_callback(self._action_goal_callback)
        self._action_server.register_preempt_callback(self._action_preempt_callback)
        self._action_server.start()
        self._feedback = interface_msg.SystemFeedback()
        self._result = interface_msg.SystemResult()

        # service server
        rospy.Service(
            "/socialrobot/set_command",
            interface_srv.SetCommand,
            self._interface_task_callback,
        )
        rospy.Service(
            "/socialrobot/set_goal_predicates",
            social_robot_srv.SetGoalPredicates,
            self._goal_callback,
        )
        rospy.Service(
            "/socialrobot/set_task",
            social_robot_srv.SetTask,
            self._task_callback,
        )
        rospy.Service(
            "/socialrobot/reset_task",
            Empty,
            self._reset_callback,
        )

        # publisher
        self.arbi_publisher = rospy.Publisher('arbi_console', String, queue_size=10)

        # 
        self.update()

    # action server callback
    def _action_goal_callback(self):
        self._goal = self._action_server.accept_new_goal()

    def _action_preempt_callback(self):
        self._action_server.set_preempted()

    def _system_action_callback(self, goal):
        #rospy.Rate(1)
        try:            
            success = self.command_dict[goal.command]()
            
            # check that preempt has not been requested by the client
            for i in range(1, 50):
                if self._action_server.is_preempt_requested():
                    logger.error("Task is preempt requested.")
                    self._action_server.set_preempted()
                    success = False
                    break
            if success:
                logger.info("%s State is finished", goal.command)
                self._action_server.set_succeeded(self._result)
            else:
                logger.error("%s State is failed", goal.command)
                self._action_server.set_preempted(self._result)
                self.task_list[0].set_fail()
        except:
            logger.warning("%s State is failed", goal.command)
            self._action_server.set_preempted(self._result)
            self.task_list[0].set_fail()

    #############################     State callback    ###########################
    def _cmd_setup(self):
        return True

    def _cmd_get_command(self): 
        """STANDBY state"""
        self.is_cmd_received = False
        
        rate = rospy.Rate(10)
        while not self.is_cmd_received:
            # wait until user's command is received
            if not self._action_server.is_active():
                logger.error("Task is preempt requested.")
                self._action_server.set_preempted()
                self.is_cmd_received = False
                break
            rate.sleep()
        return self.is_cmd_received

    def _cmd_set_success(self):
        # current task is done        
        if not self.task_list[-1].is_failed:
            logger.info('Current task is finished.')
            self.task_list[-1].set_finish()

            if len(self.task_list)>1:
                # remove task from queue
                self.task_list.pop(-1)                
                logger.info('Next task will be started.')
                return False
            elif len(self.task_list)==1:
                return True
        else:
            logger.info('Current task is failed.')  
            return False          

    def _cmd_set_fail(self):
        logger.error('Task is failed.')
        self.knowledge_interface.is_start = False
        self.behavior_interface.clear_objects()
        self.is_task_failed = True
        return True

    def _cmd_set_retry(self):
        logger.warning("Retrying the task...")
        self.knowledge_interface.reset()
        self.behavior_interface.clear_objects()
        self.retry_count += 1
        if self.retry_count > self.MAX_TASK_ITER:
            self.retry_count = 0
            return False
        else:
            return True

    def _cmd_execute_action(self):
        current_action = self.task_list[-1].current_action['action']
        return self.behavior_interface.execute_action(current_action, self.motion)

    def _cmd_check_effect(self):   
        if self.is_sim:
            #wait seconds for vrep update
            rospy.sleep(rospy.Duration(3.0))
            
        curent_action_name = self.task_list[-1].current_action['action'].name
        scene_need_action = ['relocate_obstacle', 'hold_object', 'hold_object_dualarm', 'standby_with_object', 'putup_object', 'transfer_object','pour_object']
        # after behavior is done, get object info from moveit scene  
        if curent_action_name in scene_need_action:  
            self.perception_interface.update_objects_from_scene()

        # realtime object tracking mode
        if self.object_tracking == True:
            # get current states 
            # TODO: fix 'emptyhand' predicate estimation error
            rospy.sleep(rospy.Duration(5))
            self.knowledge_interface.update_current_states(self.task_list[-1])
            problem = self.task_list[-1].problem

            # get action effect
            current_effect = self.current_action.effect            
            if problem:            
                # compare predicates
                cond = self.knowledge_interface.compare_states(problem.facts, current_effect)
                if cond.negatives or cond.positives:
                    logger.error("Current action is failed as following condition.")
                    print(cond)
                    return False
                else:
                    return True
            else:
                logger.error("Getting current state from context_manager is failed.")
                return False
        # recognize objects only once before the task
        else:
            logger.warning('objects are not tracked. pass this state.')
            return True

    def _cmd_check_action(self):
        '''
        Setup current action and get requirements for motion planning
        '''
        try: 
            if self.task_list[-1].set_next_action():
                # get requirements of behavior
                print("current action: ", self.task_list[-1].current_action['action'])
                action_name = self.task_list[-1].current_action['action'].name
                behavior_name = self.task_list[-1].current_action['action'].planner[0] 
                res = self.behavior_interface.get_requirements(behavior_name)    
                print("requirements: ", res.requirements)
                if res.result:           
                    for req in res.requirements:
                        self.task_list[-1].current_action['requirements']['parameter'].append(req)
                        self.task_list[-1].current_action['requirements']['type'].append([])
                        self.task_list[-1].current_action['requirements']['symbolic'].append([])
                        self.task_list[-1].current_action['requirements']['metric'].append([])
                    print('requirements= ', self.task_list[-1].current_action['requirements'])
                    return True
            else:
                logger.error("[CHECK_ACTION] can't set next action")
                return False
        except:
            logger.error("[CHECK_ACTION] can't get the behavior information")
            return False

    def _cmd_get_symbolic_values(self):
        current_action = self.task_list[-1].current_action
        action = current_action['action']
        behavior_requirements = current_action['requirements']['parameter']

        # find requirement from action parameters     
        for i, pddl_param in enumerate(action.parameters):
            for j, req in enumerate(behavior_requirements):            
                if req in pddl_param:
                    sym_val = action.values[i]
                    type_val = action.type[i]
                    current_action['requirements']['symbolic'][j].append(sym_val)
                    current_action['requirements']['type'][j].append(type_val)                
        print(current_action['requirements'])
        return True

    def _cmd_get_metric_values(self):
        action = self.task_list[-1].current_action['action']
        requirements = self.task_list[-1].current_action['requirements']
        self.knowledge_interface.assign_metric_value(requirements)

        # fill Behavior.msg
        #inputs = social_robot_msg.Behavior().__slots__
        self.task_list[-1].current_action['behavior'].name = action.name.replace('_','')
        try:
            for i, param_key in enumerate(requirements['parameter']):            
               self.task_list[-1].current_action['behavior'].__setattr__(param_key, requirements['metric'][i])
        except:
            logger.error("[GET_METRIC] cannot assign metric values.")
            return False
        #joint states
        self.task_list[-1].current_action['behavior'].current_position.joint_state = self.behavior_interface.joint_states
        print(self.task_list[-1].current_action['behavior'])
        return True
        
    def _cmd_update_values(self):


        return True

    def _cmd_check_precond(self):
        # before action execution, update object
        self._update_objects()
        return True

    def _cmd_get_motion(self):
        # requirements for current behavior
        behavior_req = self.task_list[-1].current_action['behavior'] 
        res = self.behavior_interface.get_motion(behavior_req)
        if res.result is True:
            self.motion = res.motion
            return True
        logger.error("[GET_MOTION] cannot calulate motion trajectory.")
        return False

    def _cmd_get_constraints(self):
        # knowledge reasoning for social behaviors
        plan = self.task_list[-1].plan        

        return self.knowledge_interface.assign_social_constraints(plan)

    def _cmd_decode_plan(self):
        '''
        decode compound action if there is current compound action
        '''
        try:
            # decode first plan of last task
            self.task_interface.decode_plan([self.task_list[-1].plan[0]],
                                            self.task_list[-1].problem.facts)

            # transfer the action sequence to state machine iterator
            primitive_action_seq = self.task_list[-1].plan[0]['primitive_actions']
            if len(primitive_action_seq)>0:                  
                # return primitive action sequence
                self._result.action_sequence = primitive_action_seq
                self._result.action_idx = 0
                self._feedback.action_sequence = primitive_action_seq
                self._feedback.action_idx = 0

                return True
        except:
            return False

    def _cmd_update_states(self):          
        if self.is_sim:
            #wait seconds for vrep update
            rospy.sleep(rospy.Duration(3.0))

        rospy.loginfo("Checking gripper status...")
        print('left_gripper:', self.behavior_interface.check_gripper_opened("left"))
        print('right_gripper:', self.behavior_interface.check_gripper_opened("right"))
        
        # compound action is finished and grippers are not grasped state
        if self.behavior_interface.check_gripper_opened("left") and self.behavior_interface.check_gripper_opened("right"):
            # set robot's detect pose
            rospy.logwarn("Setting detecting pose...")
            self.behavior_interface.set_detect_pose(self.perception_interface.detected_objects)

        # update knowledge facts
        if self.use_knowledge:
            self._cmd_get_state()
        else:
            rospy.logwarn("parameter /use_knowledge is FALSE. Cannot get current states from knowledge manager.")

        # pop and setup for next compound action
        if len(self.task_list[-1].plan)>0:
            self.task_list[-1].plan.pop(0)
        if len(self.task_list[-1].plan)>0:
            print('setting next compound action %s', self.task_list[-1].plan[0]['compound_action'].name)
        
        return True

    def _cmd_get_plan(self):
        res = self.task_interface.get_plan()
        if res.plan_result == task_srv.GetActionSeqResponse.SUCCESS:
            for action in res.action_sequence:
                action_seq = {'compound_action':None, 'primitive_actions':[],'constraints':[]}
                action_seq['compound_action'] = action
                self.task_list[-1].plan.append(action_seq)

            # return compound action sequence
            self._result.action_sequence = res.action_sequence
            self._result.action_idx = 0
            self._feedback.action_sequence = res.action_sequence
            self._feedback.action_idx = 0

            return True
        else:
            return False

    def _cmd_get_task(self):
        '''
        estimate task object and goals
        '''        
        if len(self.task_list[-1].goal_predicates)==0: # if there is no task, estimate from command
            # get manipulation object
            target_object = self.knowledge_interface.estimate_target(self.task_list[-1].command)
            self.task_list[-1].target_object = target_object
            
            # goal state estimation
            goal_predicates = self.knowledge_interface.estimate_task(self.task_list[-1].command) 
            self.task_list[-1].goal_predicates = goal_predicates
        else:
            goal_predicates = self.task_list[-1].goal_predicates
            target_object = self.task_list[-1].target_object

        if goal_predicates:
            print("Task list: ")
            for task in self.task_list:
                print(task.goal_predicates, task.target_object)
            return True
        else:
            return False     

    def _cmd_active_perception(self):
        '''
        if target object is not detected, add subtask
        '''
        # update perception
        self._update_objects()
        
        if self.task_list:
            current_task = self.task_list[-1]
        else:
            logger.error("ACTIVE_PERCEPTION: no current task.")
            return False   

        if current_task.target_object == "SKIP":
            logger.info("Skip target object.")
            return True
        elif current_task.target_object: # if task needs manipulation object  

            if self.perception_interface.find_object(current_task.target_object)>-1: # if object is already detected
                logger.info("Target %s is already detected.", current_task.target_object)
                return True
            else:
                logger.error("Target %s is not detected.", current_task.target_object)

                # Do active perception task
                # 1. navigate waypoints
                # 2. relocate objects
                # TODO: adding sub task method
                # sub_task = Task()
                # sub_task.goal_predicate, sub_task.target_object = self.knowledge_interface.create_subgoal("rearrange")
                # self.task_list.append(sub_task)
                return False
        else:
            logger.error("Target %s is not detected.", current_task.target_object)
            return False

    def _cmd_get_state(self):
        """ create pddl problem file based on reasoning
        Returns:
            [bool]: state result
        """

        rospy.loginfo("Checking gripper status...")
        print('left_gripper:', self.behavior_interface.check_gripper_opened("left"))
        print('right_gripper:', self.behavior_interface.check_gripper_opened("right"))

        # update perception
        self._update_objects()

        # get current states
        if self.task_list:
            if self.use_knowledge:  # facts from knowledge manager
                # create facts from reasoning
                self.knowledge_interface.update_current_states(self.task_list[-1])
                if len(self.task_list[-1].problem.facts)>0:
                    # generate pddl files
                    is_pddl_created = self.task_interface.generate_problem_pddl(self.task_list[-1],
                                                                file_name="")
                    if is_pddl_created:
                        self._result.problem = self.task_list[-1].problem
                        return True
                    else:
                        logger.error("GET_STATES: pddl files are not created.")
                        return False 
                else:
                    logger.error("GET_STATES: problem and goal are needed.")
                    return False
            else:   # facts from manual scripts
                rospy.logwarn("parameter /use_knowledge is FALSE. Cannot get current states from knowledge manager.")
                return False
        logger.error("GET_STATES: no current task.")
        return False

    def _goal_callback(self, request):
        """[set goal predicates]
        Args:
            socialrobot_actionlib/Predicate[] goal_predicates    
            socialrobot_actionlib/Predicate[] social_constraints        
            int32 actionID 
        """
        logger.info("Goal predicate is requested.")
        self.is_cmd_received = True

        response = social_robot_srv.SetGoalPredicatesResponse()
        if len(request.goal_predicates)==0:
            logger.error("Goal predicate is invalid.")
            response.result = 0
            return response
        
        #print(request)
        new_task = Task() 
        new_task.goal_predicates = request.goal_predicates
        # set manipulation object
        new_task.target_object = request.goal_predicates[0].args[1]

        # set task 
        self.task_list = []
        self.task_list.append(new_task)

        # wait until task is finished
        rate = rospy.Rate(10)
        while(not self.task_list[0].is_finished):
            if self.task_list[0].is_failed:
                response.result = social_robot_srv.SetGoalPredicatesResponse().FAIL
                logger.error("Task is failed.")    
                return response            
            rate.sleep()

        # task is finished
        logger.info("All the task is finished.")   
        self.task_list.pop(0)     
        response.result = social_robot_srv.SetGoalPredicatesResponse().SUCCESS
        return response

    def _task_callback(self, request):
        '''
        req = social_robot_srv.behaviorRequest()
        '''
        logger.info("Task command is requested.")

        # command from ARBI framework
        if rospy.has_param('arbi_time'):
            cur_time = rospy.get_param('arbi_time')
            self.arbi_publisher.publish("service call receive : " + cur_time)

        response = social_robot_srv.SetTaskResponse()
        if not request.actionName:
            logger.error("Task input is invalid.")
            response.result = 0
            return response

        # get manipulation object
        target_object = self.knowledge_interface.estimate_target(request)

        # goal state estimation
        goal_predicates = self.knowledge_interface.estimate_task(request)   
        
        
        if target_object == "SKIP":
            # task is gripper action
            plan_req = behavior_srv.GetMotionRequest()

            # fill behavior requirements
            plan_req.requirements.name = goal_predicates[0].name
            group = goal_predicates[0].args[0]
            if group == "obj_left_hand":
                plan_req.requirements.robot_group = [plan_req.inputs.LEFT_GRIPPER] 
            elif group == "obj_right_hand":
                plan_req.requirements.robot_group = [plan_req.inputs.RIGHT_GRIPPER] 
            elif group == "obj_dual_hand":
                plan_req.requirements.robot_group = [plan_req.inputs.BOTH_GRIPPER] 

            # get motion
            motion_srv = rospy.ServiceProxy('/behavior/get_motion',behavior_srv.GetMotion)
            res = motion_srv(plan_req)

            # set behavior
            if res.result:
                demo_srv = rospy.ServiceProxy('/behavior/set_behavior', behavior_srv.SetBehavior)
                req = behavior_srv.SetBehaviorRequest()
                req.behavior_name = plan_req.requirements.name
                req.trajectory = res.motion.jointTrajectory
                demo_res = demo_srv(req)

        elif goal_predicates[0].name == 'handedover':
            # task is arm action
            plan_req = behavior_srv.GetMotionRequest()

            #current joints
            current_joints = self.perception_interface.joint_states
            plan_req.requirements.current_position.joint_state = current_joints

            # fill behavior requirements
            plan_req.requirements.name = 'transferobject'
            group = goal_predicates[0].args[0]
            target_object =goal_predicates[0].args[1]
            goal_position = 'obj_human'
            if group == "obj_left_hand":
                plan_req.requirements.robot_group = [plan_req.inputs.LEFT_ARM] 
            elif group == "obj_right_hand":
                plan_req.requirements.robot_group = [plan_req.inputs.RIGHT_ARM] 
            elif group == "obj_dual_hand":
                plan_req.requirements.robot_group = [plan_req.inputs.BOTH_ARM] 

            # get motion
            motion_srv = rospy.ServiceProxy('/behavior/get_motion',behavior_srv.GetMotion)
            res = motion_srv(plan_req)

            # set behavior
            if res.result:
                demo_srv = rospy.ServiceProxy('/behavior/set_behavior', behavior_srv.SetBehavior)
                req = behavior_srv.SetBehaviorRequest()
                req.behavior_name = plan_req.requirements.name
                req.trajectory = res.motion.jointTrajectory
                demo_res = demo_srv(req)

        else:              
            self.is_cmd_received = True  

            # create task instance
            new_task = Task()
            new_task.command = request   
            new_task.target_object = target_object
            new_task.goal_predicates = goal_predicates
            self.task_list = []
            self.task_list.append(new_task)

            # wait until task is finished
            rate = rospy.Rate(10)
            while(not self.task_list[0].is_finished):
                if self.task_list[0].is_failed:
                    response.result = social_robot_srv.SetTaskResponse().FAIL
                    logger.error("Task is failed.")    
                    return response            
                rate.sleep()

            # task is finished
            logger.info("All the task is finished.")   
            self.task_list.pop(0)    
        
        response.result = social_robot_srv.SetTaskResponse().SUCCESS

        if rospy.has_param('arbi_time'):
            cur_time = rospy.get_param('arbi_time')
            self.arbi_publisher.publish("service response send : " + cur_time)
        return response

    def _interface_task_callback(self, request):
        logger.info("Command is received.")
        if request.command: # get specific scenario name
            self.is_cmd_received = True
            self.task_string = request.command
        elif request.problem.goals: # get pddl problem format
            self.is_cmd_received = True
            self.problem = request.problem
        else:
            logger.error("Set invalid command.")
        # Warn: Response is empty.
        response = interface_srv.TaskResponse()
        return response

    def _callback_moveit_status(self, moveit_status):
        if not self.moveit_start:
            logger.info("[BehaviorInterface] Moveit is initialized.")
            rosparam.set_param("/moveit_status", "True")
            self.moveit_start = True

    def _reset_callback(self, req):
        logger.info("Task reset is requested.")
        res = EmptyResponse()
        self.knowledge_interface.reset()
        self.behavior_interface.clear_objects()
        return res
    
    def _update_feedback(self):
        # if self.problem:
        #    self._feedback.problem = self.problem
        # if self.current_action:
        #     self._feedback.current_action = self.current_action
        # if self.current_task:
        #     self._feedback.current_goal_predicates = []            
        #     self._feedback.current_goal_predicates = self.current_task[0]
        # self._action_server.publish_feedback(self._feedback)
        pass

    def _update_objects(self):
        '''
        update perception info to knowledge
        '''              
        self.perception_interface.update_objects()
        self.knowledge_interface.detected_objects = self.perception_interface.get_detected_objects()

    def update(self):   
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            # check module status
            if rospy.has_param("/use_knowledge"):
                self.use_knowledge = rospy.get_param("/use_knowledge")
            if rospy.has_param("/use_reasoner"):
                self.use_reasoner = rospy.get_param("/use_reasoner")

            # update perception
            self.perception_interface.publish_objects()

            # check feedback command
            if self.is_cmd_received:
                self._update_feedback()

            rate.sleep()             
        return

if __name__ == "__main__":
    interface = SocialrobotInterface()
    #rospy.spin()
