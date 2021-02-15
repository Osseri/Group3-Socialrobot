#!/usr/bin/env python
# [[Python Style Guide]]
# @ RISE LAB
# https://rise-lab.atlassian.net/wiki/spaces/ENG/pages/118358105/RISE+Python+3+Style+Guide
# @ Google
# https://google.github.io/styleguide/pyguide.html

# Python libs
import six

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
from socialrobot_msgs import srv as social_robot_srv
class Singleton(type):

    """Singleton pattern meta"""

    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class SocialrobotInterface(six.with_metaclass(Singleton)):

    """Socialrobot Interface Definition

    This is a singleton class.
    """

    def __init__(self):
        rospy.init_node("socialrobot_interface")
        rospy.loginfo("[SocialrobotInterface] Service Started!")
        
        #
        self.is_cmd_received = False
        self.is_task_finished = False
        self.is_task_failed = False
        self.task_string = None
        self.compound_action = []
        self.primitive_action_seq = []
        self.current_action = []
        self.current_task = []
        self.action_queue = []
        self.action_status = []
        self.planner_inputs = None
        self.motion = None
        self.task_command = None
        self.task_list = []         # [goal predicate, target object]
        self.problem = None
        self.retry_count = 0
        self.object_tracking = False
        
        self.command_dict = {
            "Setup": self._cmd_setup,
            "GetCommand": self._cmd_get_command,
            "TaskSuccess": self._cmd_set_success,
            "TaskFail": self._cmd_set_fail,
            "TaskRetry": self._cmd_set_retry,
            # estimate goal state and current state
            "GetTask": self._cmd_get_task,
            "ActivePerception": self._cmd_active_perception,
            "GetState": self._cmd_get_state,
            "TaskPlan": self._cmd_get_plan,
            "DecodePlan": self._cmd_decode_plan,
            # execute primitive actions
            "CheckAction": self._cmd_check_action,
            "GetValues": self._cmd_get_values,
            "CheckPrecond": self._cmd_check_precond,
            "GetMotion": self._cmd_get_motion,
            "ExecuteAction": self._cmd_execute_action,
            "CheckEffect": self._cmd_check_effect,
            "UpdateStates": self._cmd_update_states,
        }

        # get params
        self.object_tracking = rosparam.get_param("/object_tracking")
        self.moveit_start = rosparam.get_param("/moveit_status")
        self.sub_moveit = rospy.Subscriber(
            "/move_group/status",
            actionlib_msg.GoalStatusArray,
            self._callback_moveit_status,
        )
        # wait if moveit is initializing
        rospy.loginfo("[Socialrobot Interface] Waiting moveit initialization...")
        if rospy.has_param("/moveit_status"):
            while not self.moveit_start:
                self.moveit_start = rosparam.get_param("/moveit_status")
                rospy.sleep(2.0)
        rospy.loginfo("[Socialrobot Interface] Moveit is connected.")

        # load managers
        self.perception_interface = perception.PerceptionInterface()
        self.actionlib_interface = actionlibrary.ActionLibInterface()
        self.task_interface = task.TaskInterface()
        self.behavior_interface = behavior.BehaviorInterface()
        self.knowledge_interface = knowledge.KnowledgeInterface()
        rospy.loginfo("[Socialrobot Interface] Interface modules are initialized.")

        # action server
        self._action_server = actionlib.SimpleActionServer(
            "/socialrobot/system",
            interface_msg.SystemAction,
            execute_cb=self._system_action_callback,
            auto_start=False,
        )
        #self._action_server.register_goal_callback(self._goal_callback)
        self._action_server.register_preempt_callback(self._preempt_callback)
        self._action_server.start()
        self._feedback = interface_msg.SystemFeedback()
        self._result = interface_msg.SystemResult()

        # service server
        rospy.Service(
            "/socialrobot/set_command",
            interface_srv.Task,
            self._interface_task_callback,
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
    def _goal_callback(self):
        self._goal = self._action_server.accept_new_goal()

    def _preempt_callback(self):
        self._action_server.set_preempted()

    def _system_action_callback(self, goal):
        rospy.Rate(1)
        success = self.command_dict[goal.command]()
        # ######################## Set action status
        # check that preempt has not been requested by the client
        for i in range(1, 50):
            if self._action_server.is_preempt_requested():
                rospy.logerr("[Socialrobot Interface] Task is preempt requested.")
                self._action_server.set_preempted()
                success = False
                break
        if success:
            rospy.loginfo("[Socialrobot Interface]  %s State is finished", goal.command)
            self._action_server.set_succeeded(self._result)

    # ####################################
    def _cmd_get_values(self):
        self.planner_inputs = self.knowledge_interface.get_values(self.current_action)
        return True

    def _cmd_check_precond(self):
        return True

    def _cmd_get_motion(self):
        res = self.behavior_interface.get_motion(self.planner_inputs)
        if res.result is True:
            self.motion = res.motion
            return True
        return False

    def _cmd_execute_action(self):
        return self.behavior_interface.execute_action(self.current_action, self.motion)

    def _cmd_check_effect(self):
        # get current states 
        # TODO: fix 'emptyhand' predicate estimation error
        rospy.sleep(rospy.Duration(5))
        res, problem = self.knowledge_interface.get_current_states()
        self.problem = problem

        # if using object tracking algorithms
        if self.object_tracking == True:
            # get action effect
            current_effect = self.current_action.effect            
            if res:            
                # compare predicates
                cond = self.knowledge_interface.compare_states(problem.facts, current_effect)
                if cond.negatives or cond.positives:
                    rospy.logerr("Current action is failed as following condition.")
                    print(cond)
                    return False
                else:
                    return True
            else:
                rospy.logerr("Getting current state from context_manager is failed.")
                return False
        else:
            return True

    def _cmd_update_states(self):
        # update scenes
        
        return True

    def _cmd_check_action(self):
        self.current_action = self.action_queue.pop(0)
        res = self.task_interface.get_action_info(self.current_action)
        if res.result:
            self.current_action = res.action
            return True
        else:
            return False

    def _cmd_decode_plan(self):
        self.primitive_action_seq = self.task_interface.decode_plan(
            self.compound_action
        )    

        self.action_queue = self.primitive_action_seq
        if len(self.primitive_action_seq) > 0:
            self._result.action_sequence = self.primitive_action_seq
            self._result.action_idx = 0
            self._feedback.action_sequence = self.primitive_action_seq
            self._feedback.action_idx = 0
            return True
        return False

    def _cmd_get_plan(self):
        res = self.task_interface.get_plan()
        if res.plan_result == task_srv.GetActionSeqResponse.SUCCESS:
            self.compound_action = res.action_sequence
            return True
        return False

    def _cmd_get_task(self):
        '''
        estimate task object and goals
        '''
        # get manipulation object
        target_object = self.knowledge_interface.estimate_target(self.task_command)
        
        # goal state estimation
        goal_predicates = self.knowledge_interface.estimate_goal(self.task_command)   
        if goal_predicates:
            self.task_list = [[goal_predicates, target_object]]
            print("Task list: ")
            for task in self.task_list:
                print(task)
            return True
        else:
            return False     

    def _cmd_active_perception(self):
        '''
        if target object is not detected, add subtask
        '''
        if self.task_list:
            self.current_task = self.task_list[-1]
        
        current_task = self.current_task
        current_target_object = current_task[1]

        if current_target_object: # if task need manipulation object
            if self.knowledge_interface.is_detected(current_target_object): # if object is already detected
                rospy.loginfo("Target %s is already detected.", current_target_object)
                return True
            else:
                rospy.logerr("Target %s is not detected.", current_target_object)
                print("Detected object: ")
                for obj in self.knowledge_interface.detected_objects:
                    print(obj.name.data)

                # Do active perception task
                # 1. navigate waypoints
                # 2. relocate objects
                # add sub task : rearrange
                goal, target_object = self.knowledge_interface.create_subgoal("rearrange")
                self.task_list.append([goal, target_object])
                return True
        else:
            return True

    def _cmd_get_state(self):
        """ create pddl problem file based on knowledge reasoning
        Returns:
            [bool]: Task result
        """
        is_pddl = False
        problem = None
        if self.task_list:
            self.current_task = self.task_list[-1]
        else:
            rospy.logerr("[Socialrobot Interface] GET_STATES: no current task.")
            return False

        current_goal, current_target_object = self.current_task
        
        # get current states
        if self.task_command.actionName:  
            # create facts from knowledge manager
            res, problem = self.knowledge_interface.get_current_states(target_object=current_target_object)
            
            if problem:
                # generate problem
                is_pddl, problem = self.task_interface.generate_problem_pddl(
                    file_name="", pddl_problem=problem, goal=current_goal
                )
                if is_pddl:
                    self.problem = problem
                    self._result.problem = self.problem
                    return True
            else:
                rospy.logerr("[Socialrobot Interface] GET_STATES: problem and goal are needed.")
                return False        
        return False

    def _cmd_setup(self):
        return True

    def _cmd_set_fail(self):
        rospy.logerr('Task is failed.')
        self.knowledge_interface.is_start = False
        self.behavior_interface.clear_objects()
        self.is_task_failed = True
        return True

    def _cmd_set_retry(self):
        self.knowledge_interface.reset()
        self.behavior_interface.clear_objects()
        self.retry_count += 1
        if self.retry_count > 3:
            self.retry_count = 0
            self.is_task_failed = True
            return False
        else:
            return True

    def _cmd_set_success(self):
        self.task_list.pop()
        if not self.task_list:
            rospy.logerr('Task is succeeded.')
            self.is_task_finished = True
            return True
        elif len(self.task_list)>1:
            rospy.logerr('Next task will be started.')
            return False

    def _cmd_get_command(self): 
        """STANDBY state"""
        self.is_cmd_received = False
        self.task_command = None
        rate = rospy.Rate(10)
        while not self.is_cmd_received:
            # wait until user's command is received
            if not self._action_server.is_active():
                rospy.logerr("[Socialrobot Interface] Task is preempt requested.")
                self._action_server.set_preempted()
                self.is_cmd_received = False
                break
            rate.sleep()
        return self.is_cmd_received

    def _task_callback(self, request):
        '''req = social_robot_srv.behaviorRequest()
        # demo 1
        req.actionName = "Move"
        req.actionParam1 = "table"
        # demo 2
        req.actionName = "HandOver"          
        req.actionParam1 = "Gotica"              
        req.actionParam2 = "Person"           
        req.actionConstraint = "Left" 
        demo 3         
        req.actionName = "Grab"              
        req.actionParam1 = "Gotica"             
        req.actionConstraint = "Left"  
        demo 4         
        req.actionName = "Open" or "Close"
        req.actionConstraint = "Left"  
        '''

        # command from ARBI framework
        if rospy.has_param('arbi_time'):
            cur_time = rospy.get_param('arbi_time')
            self.arbi_publisher.publish("service call receive : " + cur_time)
        rospy.loginfo("[Socialrobot Interface] Task is received.")

        response = social_robot_srv.SetTaskResponse()
        if not request.actionName:
            rospy.logerr("[Socialrobot Interface] Task input is invalid.")
            response.result = 0
            return response

        self.task_command = request
        self.is_cmd_received = True

        # wait until task is finished
        while(not self.is_task_finished):
            rospy.sleep(rospy.Duration(1))
            if self.is_task_failed:
                response.result = 0
                self.is_task_failed = False
                return response
        rospy.loginfo("[Socialrobot Interface] Task is finished.")
        response.result = 1
        self.is_task_finished = False
        if rospy.has_param('arbi_time'):
            cur_time = rospy.get_param('arbi_time')
            self.arbi_publisher.publish("service response send : " + cur_time)
        return response

    def _interface_task_callback(self, request):
        rospy.loginfo("[Socialrobot Interface] Command is received.")
        if request.command: # get specific scenario name
            self.is_cmd_received = True
            self.task_string = request.command
        elif request.problem.goals: # get pddl problem format
            self.is_cmd_received = True
            self.problem = request.problem
        else:
            rospy.logerr("[Socialrobot Interface] Set command.")
        # Warn: Response is empty.
        response = interface_srv.TaskResponse()
        return response

    def _callback_moveit_status(self, moveit_status):
        if not self.moveit_start:
            rospy.loginfo("[BehaviorInterface] Moveit is initialized.")
            rosparam.set_param("/moveit_status", "True")
            self.moveit_start = True

    def _reset_callback(self, req):
        res = EmptyResponse()
        self.knowledge_interface.reset()
        self.behavior_interface.clear_objects()
        return res
    
    def _update_feedback(self):
        if self.problem:
           self._feedback.problem = self.problem
        if self.current_action:
            self._feedback.current_action = self.current_action
        if self.current_task:
            self._feedback.current_goal_predicates = []            
            self._feedback.current_goal_predicates = self.current_task[0]
        self._action_server.publish_feedback(self._feedback)

    def update(self):   
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #if self.is_cmd_received:
            self._update_feedback()
            rate.sleep() 
        return

if __name__ == "__main__":
    interface = SocialrobotInterface()
    rospy.spin()
