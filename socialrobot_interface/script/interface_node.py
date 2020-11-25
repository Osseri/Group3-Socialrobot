#!/usr/bin/env python
import abc
from abc import ABCMeta
from six import with_metaclass

import rospy
import rospkg
import rosparam
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from socialrobot_interface.msg import *
from socialrobot_interface.srv import *
from socialrobot_task.srv import *

import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util

#load modules
from perception import PerceptionInterface
from actionlibrary import ActionLibInterface
from task import TaskInterface
from behavior import BehaviorInterface
from knowledge import KnowledgeInterface

class Singleton(type):
    '''
    for singleton pattern
    '''
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class SocialrobotInterface(with_metaclass(Singleton)):
    def __init__(self):        
        rospy.init_node("socialrobot_interface")
        rospy.loginfo('[SocialrobotInterface] Service Started!')
        
        # action server
        self._as = actionlib.SimpleActionServer("/socialrobot/system", SystemAction, execute_cb=self._callback_action, auto_start = False)
        self._as.start()
        self._feedback = SystemFeedback()
        self._result = SystemResult()

        # service server
        rospy.Service('/socialrobot/set_command', Task, self._callback_command)

        # wait for moveit group
        self.moveit_start = rosparam.get_param("/moveit_status")
        self.sub_moveit = rospy.Subscriber("/move_group/status", GoalStatusArray, self._callback_moveit_status)
        self._init()

        # load managers
        self.pm = PerceptionInterface()
        self.al = ActionLibInterface()
        self.tm = TaskInterface()
        self.bm = BehaviorInterface()
        self.km = KnowledgeInterface()
        rospy.loginfo("Interface modules are initialized.")   

        #        
        self.isCommand = None
        self.task = None
        self.compound_action = []
        self.primitive_action = []
        self.current_action = []
        self.queue_action = []
        self.action_status = []
        self.planner_inputs = None
        self.motion = None
        self.problem = []
    
    def _callback_action(self, goal):  
        r = rospy.Rate(1)
        success = False

        if goal.command == "Setup":
            success = True
        # wait until user's command is received
        elif goal.command == 'GetCommand':
            self.isCommand = None
            self._get_command()
            if self.isCommand:
                success = True
        # estimate goal state and current state
        elif goal.command == "GetState":
            if self._get_state():
                success = True
            
        # 
        elif goal.command == 'TaskPlan':
            if self._get_plan():
                success = True
        # 
        elif goal.command == 'DecodePlan':
            if self._decode_plan():
                success = True

        ######################### Primitive action ###########################
        # 
        elif goal.command == 'CheckAction':
            if self._check_action():
                success = True

        # 
        elif goal.command == 'GetValues':
            if self._get_values():
                success = True

        # Check precondition predicates
        elif goal.command == 'CheckPrecond':
            if self._check_precond():
                success = True

        # 
        elif goal.command == 'GetMotion':
            if self._get_motion():
                success = True

        # 
        elif goal.command == 'ExecuteAction':
            if self._execute_action():
                success = True

        # 
        elif goal.command == 'CheckEffect':
            if self._check_efffect():
                success = True

        # 
        elif goal.command == 'UpdateStates':
            if self._update_states():
                success = True

        ######################### Set action status ###########################
        # check that preempt has not been requested by the client
        for i in range(1, 50):
            if self._as.is_preempt_requested():
                rospy.logerr('Task is preempt requested.')
                self._as.set_preempted()
                success = False
                break

        if success:
            rospy.loginfo("%s is succeeded", goal.command)
            self._as.set_succeeded(self._result)

#####################################
    def _get_values(self):
        self.planner_inputs = self.km.get_values(self.current_action)
        return True

    def _check_precond(self):
        return True

    def _get_motion(self):
        res = self.bm.get_motion(self.planner_inputs)
        if res.result == True:
            self.motion = res.motion
            return True
        else:
            return False

    def _execute_action(self):
        return self.bm.execute_action(self.current_action, self.motion)

    def _check_efffect(self):
        return True

    def _update_states(self):
        return True         

    def _check_action(self):
        self.current_action = self.queue_action.pop(0)
        return True

    def _decode_plan(self):
        self.primitive_action = self.tm.decode_plan(self.compound_action)
        self.queue_action = self.primitive_action
        if len(self.primitive_action)>0:  
            self._result.action_sequence = self.primitive_action  
            self._result.action_idx = 0
            self._feedback.action_sequence = self.primitive_action  
            self._feedback.action_idx = 0        
            return True
        else:
            return False

    def _get_plan(self):
        res = self.tm.get_plan()
        if res.plan_result == GetActionSeqResponse.SUCCESS:
            self.compound_action = res.action_sequence
            return True
        else:
            return False

    def _get_state(self):            
        #load states from script
        if self.task:
            res = self.tm.generate_problem_pddl(file_name=self.task)
    
        #load states from knowledge  
        elif self.problem.goals:   
            self.problem.facts += self.km.get_current_states()
            self._feedback.problem = self.problem
            self._result.problem = self.problem
            #generate problem
            res = self.tm.generate_problem_pddl(file_name="", problem=self.problem)
        else:
            return False

        if res:
            self._as.publish_feedback(self._feedback)
            return True
        else:
            return False

    def _get_command(self):
        '''
        wait commad service 
        '''
        r = rospy.Rate(10)    
        while(not self.isCommand):
            if not self._as.is_active() or not self._as.is_active:
                rospy.logerr('Task is preempt requested.')
                self._as.set_preempted()
                self.isCommand = False
                break
            r.sleep()

    def _callback_command(self,req):
        rospy.loginfo("Command is received.")
        if req.command:
            self.isCommand = True
            self.task = req.command
        elif req.problem.goals:
            self.isCommand = True
            self.problem = req.problem             
        else:
            rospy.logerr("Set command.")      
        return 

    def _callback_moveit_status(self, moveit_status):
        if not self.moveit_start:
            rospy.loginfo("[BehaviorInterface] Moveit is initialized.")
            rosparam.set_param("/moveit_status", "True")
            self.moveit_start = True

    def _init(self):                
        # wait if moveit is initializing
        rospy.loginfo("Waiting moveit initialization...")
        if rospy.has_param("/moveit_status"):
            while not self.moveit_start:
                self.moveit_start = rosparam.get_param("/moveit_status")
                rospy.sleep(2.0)
        
        rospy.loginfo("Moveit is connected.")
    
    def update(self):
        self.km.update()
        return

if __name__ == '__main__':       
    interface = SocialrobotInterface()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        interface.update()
        r.sleep()