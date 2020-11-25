#!/usr/bin/env python
import rospy
import rospkg
import rosparam
import tf

from socialrobot_behavior.srv import *
from socialrobot_behavior.msg import *
from socialrobot_hardware.msg import RobotState
import socialrobot_interface

from sensor_msgs.msg import JointState
from vision_msgs.msg import *
from std_msgs.msg import *
from interface import InterfaceBase

import moveit_commander

class BehaviorInterface(InterfaceBase):
    def __init__(self):   
        super(BehaviorInterface, self).__init__()
        '''
        Module for Behavior Manager
        '''
        rospy.loginfo("Initializing BehaviorInterface...")

        # get robot status
        self.left_group = moveit_commander.MoveGroupCommander("left_arm" )
        self.right_group = moveit_commander.MoveGroupCommander("right_arm")
        self.left_eef_group = moveit_commander.MoveGroupCommander("left_eef" )
        self.right_eef_group = moveit_commander.MoveGroupCommander("right_eef")
        self.base_group = []
        self.group = [self.right_group, self.left_group, self.base_group]

        # set the data label name for DB
        self.joint_state_name = "/behavior/jointState"

        # Subscribe data         
        topic_robot_state = "/sim_interface/vrep_state"
        topic_joint_state = "/sim_interface/joint_states"
        topic_object_list = "/sim_interface/object_list"
        if rospy.has_param('robot_hw') and rospy.get_param('robot_hw') != 'vrep':
            topic_robot_state = "/hw_interface/state"
            topic_joint_state = "/hw_interface/joint_states"
            topic_object_list = "/hw_interface/object_list"
        else:
            rospy.logwarn("Cannot find robot hardware param")

        rospy.Subscriber(topic_joint_state, JointState, self._callback_joint_state)
        rospy.Subscriber(topic_robot_state, Int32, self._callback_robot_state)
    
        # Publisher
        self.pub_joint_state = rospy.Publisher("/joint_state", JointState, queue_size=10)
        self.pub_robot_state = rospy.Publisher("~behavior/robot_state", RobotState, queue_size=10)
        self.pub_eef_tf = rospy.Publisher("/visual_robot_perception", Float32MultiArray, queue_size=10)

        # Subscriber
        self.listener = tf.TransformListener()  

    def execute_action(self, action, motion):
        rospy.loginfo("Executing %s..."%action.name)

        behavior_srv = rospy.ServiceProxy('/behavior/set_behavior', SetBehavior)
        behavior_req = socialrobot_behavior.srv.SetBehaviorRequest()
        behavior_req.header.frame_id = action.planner[0]
        behavior_req.trajectory = motion.jointTrajectory
        res = SetBehaviorResponse()
        behavior_srv(behavior_req)
        
        if res.result == SetBehaviorResponse.OK:
            return True
        else:
            return False
	
    def get_motion(self, req):
        '''
        Inpouts:    string planner_name
                    socialrobot_behavior/PlannerInputs inputs
        ---
        Outputs:    bool result
                    socialrobot_behavior/PlannerOutputs motion
        '''
        behavior_srv = rospy.ServiceProxy('/behavior/get_motion', GetMotion)
        return behavior_srv(req)

    def _callback_joint_state(self, joint_state):
        '''
        Transfer robot poses for knowledge 
        '''
        # publish enf-effector joints, position and for context_manager
        if joint_state.name:
            for group in self.group:
                if group:
                    joints = JointState()
                    perception = Float32MultiArray()

                    if group.get_name() == 'left_arm':
                        try:          	
                            joints.header = joint_state.header
                            joint_names = self.left_eef_group.get_active_joints()
                            for joint in joint_names:
                                if joint in joint_state.name:
                                    joints.name.append(joint)
                                    idx = joint_state.name.index(joint)
                                    joints.position.append(joint_state.position[idx])
                                    joints.velocity.append(joint_state.position[idx])
                                    joints.effort.append(joint_state.position[idx])   
                            (pos, ori) = self.listener.lookupTransform('/base_footprint', '/left_end_effect_point', rospy.Time(0))
                            perception.data = [pos[0], pos[1], pos[2], 0, 0, 0, 12.0]
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            rospy.sleep(1)
                    elif group.get_name() == 'right_arm':
                        try:          	
                            joints.header = joint_state.header
                            joint_names = self.right_eef_group.get_active_joints()
                            for joint in joint_names:
                                if joint in joint_state.name:
                                    joints.name.append(joint)
                                    idx = joint_state.name.index(joint)
                                    joints.position.append(joint_state.position[idx])
                                    joints.velocity.append(joint_state.position[idx])
                                    joints.effort.append(joint_state.position[idx])   
                            (pos, ori) = self.listener.lookupTransform('/base_footprint', '/right_end_effect_point', rospy.Time(0))
                            perception.data = [pos[0], pos[1], pos[2], 0, 0, 0, 11.0]
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            rospy.sleep(1)

                    self.pub_joint_state.publish(joints)
                    self.pub_eef_tf.publish(perception)
                else:
                    perception = Float32MultiArray()
                    perception.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0]
                    self.pub_eef_tf.publish(perception)
        return 

    def _callback_robot_state(self, state):
        '''
        Transfer Robot state
        '''
        robot_state = RobotState()
        robot_state.state = state.data
        
        if state.data == RobotState.READY_FOR_ACTION:
            rosparam.set_param("/robot_state", "READY")
        elif state.data == RobotState.ACTION_RUNNING:
            rosparam.set_param("/robot_state", "RUNNING")

        self.pub_robot_state.publish(robot_state)

        return
