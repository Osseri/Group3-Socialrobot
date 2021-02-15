#!/usr/bin/env python
import rospy
import rospkg
import rosparam
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import Empty, Header, String
from std_srvs.srv import *
from socialrobot_interface.msg import *
from socialrobot_interface.srv import *

import smach
import smach_ros
from smach import Iterator, StateMachine, CBState
from smach_ros import ConditionState, IntrospectionServer

# class setup(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['setup_done','setup_fail'])

#     def execute(self, userdata):
#         rospy.sleep(5.0)

#         return 'setup_done'

# class standby(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['comm_confirmed'])

#     def execute(self, userdata):
        
#         rospy.sleep(5)
#         return 'comm_confirmed'

# class catch_intent(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'fail', 'request'])

#     def execute(self, userdata):
#         rospy.sleep(5)
#         return 'success'

# class get_states(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['done', 'fail'])
        
#     def execute(self, userdata):
#         rospy.loginfo("[State Machine] Getting initial & goal states...")
#         rospy.sleep(5)
#         return 'done'

# class plan_task(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['plan_success', 'plan_fail', 'no_solution'],  
#                                     input_keys=['result', 'action_plan'], 
#                                     output_keys = ['result', 'action_plan'])

#     def execute(self, userdata):
#         rospy.loginfo("[State Machine] Task planning...")
        
        
#         if True:
#             rospy.sleep(5)
#             return 'plan_success'
#         else:
#             rospy.logerr("[State Machine] Task planning is failed.")
#             return 'plan_fail'

# class decode_plan(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['decode_success', 'decode_fail'],  
#                                     input_keys=['result', 'action_plan'], 
#                                     output_keys = ['result', 'action_sequence'])

#     def execute(self, userdata):
#         rospy.loginfo("[State Machine] Decoding compound task plan...")
#         if True:
#             rospy.sleep(5)
#             return 'decode_success'
#         else:
#             rospy.logerr("[State Machine] Decoding plan is failed.")
#             return 'decode_fail'

# class check_action(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'fail'], 
#                 input_keys=['result', 'action', 'index', 'action_sequence', 'state', 'status'],output_keys = ['result', 'action', 'requirements'])

#     def execute(self, userdata):
        
#         if True:     
#             rospy.sleep(5)
#             return 'success'
#         else:
#             rospy.logerr("[State Machine] Checking action: is failed" )
#             return 'fail'

# class get_values(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'fail'], input_keys=['result', 'action', 'index', 'action_sequence', 'requirements', 'state', 'status'], output_keys = ['result', 'action', 'values'])

#     def execute(self, userdata):
#         rospy.loginfo('[State Machine] Getting action: parameter.')
       
#         if True:   
#             rospy.sleep(5)
#             return 'success'        
#         else:
#             rospy.logerr("[State Machine] Cannot get action: parameter.")
#             return 'fail'

# class get_motion(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'fail'], input_keys=['result', 'action', 'index', 'action_sequence', 'values', 'state', 'status'], output_keys = ['result', 'action', 'motion'])

#     def execute(self, userdata):  
#         rospy.loginfo("[State Machine] Getting action: motion.")       
        
        
#         if True:     
#             rospy.sleep(5)
#             return 'success'
        
#         else:
#             rospy.logerr("[State Machine] Getting action: motion is failed." )  
#             return 'fail'

# class execute_action(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'fail'],  input_keys=['result', 'action', 'index', 'action_sequence', 'motion', 'state', 'status'], output_keys = ['result', 'action'])

#     def execute(self, userdata):
#         # Execute each action 
#         rospy.loginfo("[StateMachine] Waiting until the action is finished.")
        
#         if True:
#             rospy.loginfo("[StateMachine] The action is finished.")
#             rospy.sleep(5)
#             return 'success'
#         else:
#             rospy.logerr("[State Machine] Waiting until the action: is failed." )  
#             return 'fail'

# class check_precondition(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'fail', 'update'],  input_keys=['result', 'action', 'index', 'action_sequence', 'motion', 'preconditions', 'state', 'status'], output_keys = ['result', 'action'])

#     def execute(self, userdata):
#         rospy.loginfo("[State Machine] Checking  action: precondition." )  
#         rospy.sleep(5)
#         return 'success'

# class check_effect(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'fail', 'update'],  input_keys=['result', 'action', 'index', 'action_sequence', 'motion', 'effects', 'state', 'status'], output_keys = ['result', 'action'])

#     def execute(self, userdata):
#         rospy.loginfo("[State Machine] The  action:  effect." ) 
#         rospy.sleep(5)
#         return 'success'

# class update_states(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'fail'],  input_keys=['result', 'action', 'index', 'action_sequence', 'motion', 'effects', 'state', 'status'], output_keys = ['result', 'action'])

#     def execute(self, userdata):
#         rospy.loginfo("[State Machine] Updating  action: predicates.") 
#         rospy.sleep(5)
#         return 'success'

class SocialrobotSM():
    def __init__(self):
        rospy.init_node("socialrobot_machine_state")
        rospy.loginfo('[SocialrobotSystem] Service Started!')        

        #main container
        self.sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
        self.sm.userdata.result = ""
        self.sm.userdata.action_plan = []
        self.sm.userdata.action_sequence = []
        self.sm.userdata.action = ""
        self.sm.userdata.requirements = []
        self.sm.userdata.preconditions = []
        self.sm.userdata.effects = []
        self.sm.userdata.motion = []
        self.sm.userdata.values = []
        self.sm.userdata.status = False
        self.sm.userdata.state = ""

        self.run()

    def run(self):
        with self.sm:
            #initialize system
            smach.StateMachine.add('SETUP',
                smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                    goal = SystemGoal(command="Setup")),
                {'succeeded':'STANDBY', 'preempted':'TASK_FAIL', 'aborted':'TASK_FAIL'})
            #Wait for command
            smach.StateMachine.add('STANDBY',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                    goal = SystemGoal(command="GetCommand")),
                    {'succeeded':'GET_TASK', 'preempted':'TASK_FAIL', 'aborted':'TASK_FAIL'})
            #Goal reasoning
            smach.StateMachine.add('GET_TASK',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                    goal = SystemGoal(command="GetTask")),
                    {'succeeded':'ACTIVE_PERCEPTION', 'preempted':'TASK_FAIL', 'aborted':'TASK_FAIL'})
            #Active perception
            smach.StateMachine.add('ACTIVE_PERCEPTION',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                    goal = SystemGoal(command="ActivePerception")),
                    {'succeeded':'GET_STATES', 'preempted':'TASK_FAIL', 'aborted':'TASK_FAIL'})
            #State reasoning
            smach.StateMachine.add('GET_STATES',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="GetState")),
                    {'succeeded':'TASK_PLAN', 'preempted':'TASK_FAIL',  'aborted':'TASK_FAIL'})
            #Task planning
            smach.StateMachine.add('TASK_PLAN',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="TaskPlan")),
                    {'succeeded':'DECODE_PLAN', 'preempted':'TASK_FAIL',  'aborted':'TASK_FAIL'})
            #Action decoding
            smach.StateMachine.add('DECODE_PLAN',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="DecodePlan"), result_key='action_sequence'),
                    {'succeeded':'TASK_IT', 'preempted':'TASK_FAIL',  'aborted':'TASK_FAIL'})            
            
            #
            smach.StateMachine.add('TASK_SUCCESS',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="TaskSuccess"), result_key='action_sequence'),
                    {'succeeded':'STANDBY', 'preempted':'STANDBY',  'aborted':'ACTIVE_PERCEPTION'})
            #
            smach.StateMachine.add('TASK_FAIL',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="TaskFail"), result_key='action_sequence'),
                    {'succeeded':'STANDBY', 'preempted':'STANDBY',  'aborted':'STANDBY'})
            #
            smach.StateMachine.add('TASK_RETRY',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="TaskRetry"), result_key='action_sequence'),
                    {'succeeded':'ACTIVE_PERCEPTION', 'preempted':'STANDBY',  'aborted':'STANDBY'})

            #action iteration container
            task_it = Iterator(outcomes = ['TASK_SUCCESS', 'TASK_FAIL', 'TASK_RETRY'],
                                input_keys = ['result', 'action_sequence', 'action'],
                                it = lambda: range(0, len(self.sm.userdata.action_sequence.action_sequence)),
                                output_keys = ['result', 'action_sequence', 'action'],
                                it_label = 'index',
                                exhausted_outcome = 'TASK_SUCCESS')
                                
            with task_it:
                action_sm = StateMachine(
                    outcomes = ['TASK_SUCCESS', 'TASK_FAIL','TASK_CONTINUE', 'TASK_RETRY'], input_keys = ['result', 'action_sequence', 'index', 'action'], output_keys = ['result', 'action_sequence', 'action'])

                with action_sm:
                    #Initialize
                    smach.StateMachine.add('CHECK_ACTION',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="CheckAction")),
                    {'succeeded':'GET_VALUES', 'preempted':'TASK_RETRY',  'aborted':'TASK_RETRY'})
                    #
                    smach.StateMachine.add('GET_VALUES',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="GetValues")),
                    {'succeeded':'CHECK_PRECONDITION', 'preempted':'TASK_RETRY',  'aborted':'TASK_RETRY'})
                    #
                    smach.StateMachine.add('CHECK_PRECONDITION',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="CheckPrecond")),
                    {'succeeded':'GET_MOTION', 'preempted':'TASK_RETRY',  'aborted':'TASK_RETRY'})
                    #
                    smach.StateMachine.add('GET_MOTION',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="GetMotion")),
                    {'succeeded':'EXECUTE_ACTION', 'preempted':'TASK_RETRY',  'aborted':'TASK_RETRY'})
                    #
                    smach.StateMachine.add('EXECUTE_ACTION',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="ExecuteAction")),
                    {'succeeded':'CHECK_EFFECT', 'preempted':'TASK_RETRY',  'aborted':'TASK_RETRY'})
                    #
                    smach.StateMachine.add('CHECK_EFFECT',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="CheckEffect")),
                    {'succeeded':'UPDATE_STATES', 'preempted':'TASK_RETRY',  'aborted':'TASK_RETRY'})
                    #
                    smach.StateMachine.add('UPDATE_STATES',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="UpdateStates")),
                    {'succeeded':'TASK_CONTINUE', 'preempted':'TASK_RETRY',  'aborted':'TASK_RETRY'})              

                #close action_sm
                Iterator.set_contained_state('ACTION_STATE', 
                                            action_sm, 
                                            loop_outcomes=['TASK_CONTINUE'])
            #close the task_it
            StateMachine.add('TASK_IT',task_it,
                        {'TASK_SUCCESS':'TASK_SUCCESS',
                        'TASK_FAIL':'TASK_FAIL', 
                        'TASK_RETRY':'TASK_RETRY'})

        sis = smach_ros.IntrospectionServer('smach_server', self.sm, '/SM_ROOT')
        sis.start()
        self.sm.execute()
        rospy.spin()
        sis.stop()

if __name__=="__main__":
    state_machine = SocialrobotSM()
