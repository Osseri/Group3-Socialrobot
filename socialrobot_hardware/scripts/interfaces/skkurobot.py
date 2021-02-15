import abc
from abc import ABCMeta
from six import with_metaclass 

import math
import rospy
import rosservice
import rosparam
import actionlib

from socialrobot_hardware.srv import *
import socialrobot_interface.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from interface import InterfaceBase
ACTION_READY = 0
ACTION_BUSY = 1

RIGHT_ARM = 0
LEFT_ARM = 1
RIGHT_HAND = 2
LEFT_HAND = 3

class skkurobotInterface(InterfaceBase):    
    def __init__(self, robot_name, **params):
        super(skkurobotInterface, self).__init__(robot_name, **params)
        
        # subscriber for social_robot
        rospy.Subscriber("/skkurobot/joint_states", JointState, self.callback_joint_states, queue_size=10)
        rospy.Subscriber("/skkurobot/robot_state", Int32, self.callback_robot_state, queue_size=10)
        
        # publisher
        self.pub_joints = rospy.Publisher("/hw_interface/joint_states", JointState, queue_size=10)
       
        self.topic_name = ''
        self.srv_name = ''
        self.trajectory = None
        self.goal_recieved = False

    def callback_robot_state(self, data):
        return

    def callback_joint_states(self, data):
        self.pub_joints.publish(data)

    def get_state(self):
        # check hand moving state
        now = rospy.get_time()

        return 

    def set_motion(self, req):     
        res = SetJointTrajectoryResponse()
        robot_part = 0
        trajectory = req.trajectory
        self.joint_names = trajectory.joint_names
        
        # set robot part
        if self.joint_names ==  ['right_bh_j12_joint', 'right_bh_j22_joint', 'right_bh_j32_joint', 'right_bh_j11_joint']:
            robot_part = RIGHT_HAND
            self.planning_group = 'right_arm'

        elif self.joint_names == ['left_bh_j12_joint', 'left_bh_j22_joint', 'left_bh_j32_joint', 'left_bh_j11_joint']:
            robot_part = LEFT_HAND
            self.planning_group = 'left_arm'

        elif self.joint_names == ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']:
            robot_part = RIGHT_ARM
            self.planning_group = 'right_arm'

        elif self.joint_names == ['j1_joint', 'j2_joint', 'j3_joint', 'j4_joint', 'j5_joint', 'j6_joint', 'j7_joint']:
            robot_part = LEFT_ARM
            self.planning_group = 'left_arm'

        else:
            rospy.logerr('[hw_interface] undefined robot part')
            res.result = SetJointTrajectoryResponse.ERROR

        # set ROS topic
        if robot_part == LEFT_HAND:
            self.topic_name = '/bhand_node/hand_command'
        elif robot_part == RIGHT_HAND:
           self.topic_name = '/bhand_node/hand_command'

        pub_joint = rospy.Publisher(self.topic_name, JointState, queue_size=10)
        for pt in trajectory.points:
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time().now()
            joint_state.name = self.joint_names
            joint_state.position = pt.positions
            num_joints = len(self.joint_names)
            joint_state.velocity = [0.1] * num_joints
            joint_state.effort = [0] * num_joints
            pub_joint.publish(joint_state)

        res.result = SetJointTrajectoryResponse.OK
        return res

   
    def update(self):
        self.get_state()

        return 