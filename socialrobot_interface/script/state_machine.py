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

class SocialrobotSM():
    def __init__(self):
        rospy.init_node("socialrobot_machine_state")
        rospy.loginfo('[SocialrobotSystem] State machine is started!')        

        #main container
        self.sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
        self.sm.userdata.result = ""
        self.sm.userdata.action_plan = []
        #self.sm.userdata.action_sequence = []
        self.sm.userdata.com_act_seq = None
        self.sm.userdata.prim_act_seq = None
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
            def task_cb(userdata, status, result):
                self.sm.userdata.com_act_seq = result
            smach.StateMachine.add('TASK_PLAN',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="TaskPlan"), result_cb=task_cb),
                    {'succeeded':'GET_CONSTRAINTS', 'preempted':'TASK_FAIL',  'aborted':'TASK_FAIL'})
            #Get social constraints
            smach.StateMachine.add('GET_CONSTRAINTS',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="GetConst")),
                    {'succeeded':'COM_IT', 'preempted':'TASK_FAIL',  'aborted':'TASK_FAIL'})

            smach.StateMachine.add('TASK_SUCCESS',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="TaskSuccess")),
                    {'succeeded':'STANDBY', 'preempted':'STANDBY',  'aborted':'ACTIVE_PERCEPTION'})
            #
            smach.StateMachine.add('TASK_FAIL',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="TaskFail")),
                    {'succeeded':'STANDBY', 'preempted':'STANDBY',  'aborted':'STANDBY'})
            #
            smach.StateMachine.add('TASK_RETRY',
                    smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                        goal = SystemGoal(command="TaskRetry")),
                    {'succeeded':'ACTIVE_PERCEPTION', 'preempted':'STANDBY',  'aborted':'STANDBY'})

            # #DEBUG function
            # @smach.cb_interface(input_keys=['prim_act_seq'],
            #                     output_keys=['prim_act_seq'], 
            #                     outcomes=['succeeded'])
            # def even_cb(ud):
            #     print('com_act_seq')
            #     print(self.sm.userdata.com_act_seq)
            #     return 'succeeded'
            # StateMachine.add('TEST', CBState(even_cb), 
            #                 {'succeeded':'COM_IT'})

            ############## compound action iteration container ################
            compound_action_it = Iterator(
                outcomes = ['COM_ITER_SUCCESS', 'COM_ITER_FAIL', 'COM_ITER_RETRY'],
                it = lambda: range(0, len(self.sm.userdata.com_act_seq.action_sequence)),
                it_label = 'com_index',
                input_keys = ['com_act_seq', 'prim_act_seq'],
                output_keys = ['com_act_seq', 'prim_act_seq'],
                exhausted_outcome = 'COM_ITER_SUCCESS')

            with compound_action_it:
                com_action_sm = StateMachine(
                    outcomes = ['COM_ITER_SUCCESS', 'COM_ITER_FAIL','COM_ITER_CONTINUE', 'COM_ITER_RETRY'], input_keys = ['com_index', 'com_act_seq', 'prim_act_seq'], 
                    output_keys = ['com_act_seq', 'prim_act_seq'])
                
                with com_action_sm:
                    # Action decoding
                    def decode_cb(userdata, status, result):
                        self.sm.userdata.prim_act_seq = result

                    smach.StateMachine.add('DECODE_PLAN',
                            smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                                        goal = SystemGoal(command="DecodePlan"), 
                                        input_keys = ['com_act_seq', 'prim_act_seq'],
                                        output_keys = ['com_act_seq', 'prim_act_seq'],
                                        result_cb=decode_cb),
                                        {'succeeded':'PRIM_IT', 'preempted':'COM_ITER_RETRY',  'aborted':'COM_ITER_RETRY'})

                    # Update compound action state
                    smach.StateMachine.add('UPDATE_STATES',
                            smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                                        goal = SystemGoal(command="UpdateStates"),                  
                                        input_keys = ['com_act_seq', 'prim_act_seq'],
                                        output_keys = ['com_act_seq', 'prim_act_seq']),
                                        {'succeeded':'COM_ITER_CONTINUE', 'preempted':'COM_ITER_RETRY',  'aborted':'COM_ITER_RETRY'})  

                    ################### primitive action iteration container ##################
                    primitive_action_it = Iterator(
                        outcomes = ['PRIM_ITER_SUCCESS', 'PRIM_ITER_FAIL', 'PRIM_ITER_RETRY'],
                        it = lambda: range(0, len(self.sm.userdata.prim_act_seq.action_sequence)),
                        it_label = 'prim_index',
                        input_keys = ['com_act_seq', 'prim_act_seq'],
                        output_keys = ['com_act_seq', 'prim_act_seq'],
                        exhausted_outcome = 'PRIM_ITER_SUCCESS')

                    with primitive_action_it:
                        action_sm = StateMachine(
                            outcomes = ['PRIM_ITER_SUCCESS', 'PRIM_ITER_FAIL','PRIM_ITER_CONTINUE', 'PRIM_ITER_RETRY'], 
                            input_keys = ['com_act_seq', 'prim_act_seq'], 
                            output_keys = ['com_act_seq', 'prim_act_seq'])

                        with action_sm:
                    
                            #Check action condition
                            smach.StateMachine.add('CHECK_ACTION',
                            smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                                goal = SystemGoal(command="CheckAction")),
                            {'succeeded':'GET_SYMBOLIC_VALUES', 'preempted':'PRIM_ITER_RETRY',  'aborted':'PRIM_ITER_RETRY'})
                            #Assign symbolic values
                            smach.StateMachine.add('GET_SYMBOLIC_VALUES',
                            smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                                goal = SystemGoal(command="GetSymbolicValues")),
                            {'succeeded':'GET_METRIC_VALUES', 'preempted':'PRIM_ITER_RETRY',  'aborted':'PRIM_ITER_RETRY'})
                            #Assign metric values
                            smach.StateMachine.add('GET_METRIC_VALUES',
                            smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                                goal = SystemGoal(command="GetMetricValues")),
                            {'succeeded':'UPDATE_VALUES', 'preempted':'PRIM_ITER_RETRY',  'aborted':'PRIM_ITER_RETRY'})
                            #Update constraint values
                            smach.StateMachine.add('UPDATE_VALUES',
                            smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                                goal = SystemGoal(command="UpdateValues")),
                            {'succeeded':'CHECK_PRECONDITION', 'preempted':'PRIM_ITER_RETRY',  'aborted':'PRIM_ITER_RETRY'})
                            #
                            smach.StateMachine.add('CHECK_PRECONDITION',
                            smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                                goal = SystemGoal(command="CheckPrecond")),
                            {'succeeded':'GET_MOTION', 'preempted':'PRIM_ITER_RETRY',  'aborted':'PRIM_ITER_RETRY'})
                            #
                            smach.StateMachine.add('GET_MOTION',
                            smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                                goal = SystemGoal(command="GetMotion")),
                            {'succeeded':'EXECUTE_ACTION', 'preempted':'PRIM_ITER_RETRY',  'aborted':'PRIM_ITER_RETRY'})
                            #
                            smach.StateMachine.add('EXECUTE_ACTION',
                            smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                                goal = SystemGoal(command="ExecuteAction")),
                            {'succeeded':'CHECK_EFFECT', 'preempted':'PRIM_ITER_RETRY',  'aborted':'PRIM_ITER_RETRY'})
                            #
                            smach.StateMachine.add('CHECK_EFFECT',
                            smach_ros.SimpleActionState('/socialrobot/system', SystemAction,
                                goal = SystemGoal(command="CheckEffect")),
                            {'succeeded':'PRIM_ITER_CONTINUE', 'preempted':'PRIM_ITER_RETRY',  'aborted':'PRIM_ITER_RETRY'})            

                        #close action_sm
                        Iterator.set_contained_state('PRIMITIVE_ACTION_STATE', 
                                                    action_sm, 
                                                    loop_outcomes=['PRIM_ITER_CONTINUE'])

                    StateMachine.add('PRIM_IT', primitive_action_it,
                        {'PRIM_ITER_SUCCESS':'UPDATE_STATES',
                        'PRIM_ITER_FAIL':'COM_ITER_FAIL', 
                        'PRIM_ITER_RETRY':'COM_ITER_RETRY'})


                Iterator.set_contained_state('COMPOUND_ACTION_STATE', 
                                            com_action_sm, 
                                            loop_outcomes=['COM_ITER_CONTINUE'])
            #close the task_it
            StateMachine.add('COM_IT', compound_action_it,
                        {'COM_ITER_SUCCESS':'TASK_SUCCESS',
                        'COM_ITER_FAIL':'TASK_FAIL', 
                        'COM_ITER_RETRY':'TASK_RETRY'})        

        sis = smach_ros.IntrospectionServer('smach_server', self.sm, '/SM_ROOT')
        sis.start()
        self.sm.execute()
        rospy.spin()
        sis.stop()

if __name__=="__main__":
    state_machine = SocialrobotSM()
