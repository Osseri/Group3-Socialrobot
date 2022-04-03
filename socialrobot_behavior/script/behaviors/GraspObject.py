import abc
from abc import ABCMeta
from six import with_metaclass 
import math
import rospy
import rosservice
import rosparam

from socialrobot_motion.srv import *
from socialrobot_behavior.srv import *
from socialrobot_hardware.srv import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import Pose

import tf.transformations as tfm
import numpy as np 
import random
from behavior import BehaviorBase

class GraspObjectBehavior(BehaviorBase):
    ''' Gripper Open-Close  '''
    def __init__(self, name, **params):
        super(GraspObjectBehavior, self).__init__(name, **params)
        
        if self._hardware_if == 'vrep':
            self.service_name = '/sim_interface/set_motion'
            self.service_type = VrepSetJointTrajectory
        elif self._hardware_if == 'hw':
            self.service_name = '/hw_interface/set_motion'
            self.service_type = SetJointTrajectory

        armplan_srv = '/motion_plan/move_arm'
        self.srv_plan = rospy.ServiceProxy(armplan_srv, MotionPlan)

        self.skkurobot_gripper = {
            'left_joint': ['left_bh_j12_joint', 'left_bh_j22_joint', 'left_bh_j32_joint', 'left_bh_j11_joint'],
            'right_joint': ['right_bh_j12_joint', 'right_bh_j22_joint', 'right_bh_j32_joint', 'right_bh_j11_joint'],
            'left_close': [1.44, 1.44, 1.44, 0.0],
            'left_open': [0.0, 0.0, 0.0, 0.0],
            'right_close': [1.44, 1.44, 1.44, 0.0],
            'right_open': [0.0, 0.0, 0.0, 0.0]
        }

        self.social_robot_gripper = {
            'left_joint': ['LFinger_1', 'LFinger_2', 'LFinger_3'],
            'right_joint': ['RFinger_1', 'RFinger_2', 'RFinger_3'],
            'left_open': [-0.34, -0.34, -0.34],
            'left_close': [0.0872665, 0.0872665, 0.0872665],
            'right_open': [-0.34, -0.34, 0.34],
            'right_close': [0.0872665, 0.122173, -0.0872665]
        }

        robot_name = rosparam.get_param("/robot_name")
        self.openclose_joints = {}
        if robot_name == 'skkurobot':
            self.openclose_joints = self.skkurobot_gripper
        elif robot_name == 'social_robot':
            self.openclose_joints = self.social_robot_gripper

        self.input_args = ['robot_group']
        self.hardware_group = ['gripper']

    def prepare_behavior(self):
        rospy.loginfo('preparing...%s' % self._name)
        return True

    def run_behavior(self):
        rospy.loginfo('running...%s' % self._name)        
        return 1

    def get_motion(self, inputs):
        '''
        return the trajectory 
        '''
        rospy.loginfo("Calculating grasp motion..")

        res = self._call_ros_service(inputs)
        if res.planResult == MotionPlanResponse.SUCCESS:
            rospy.loginfo("Grasp planning is done.")
            self.motion_trajectory = res.jointTrajectory
        else:
            rospy.loginfo("Grasp planning is failed.")
        return res

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True

    def _call_ros_service(self, inputs):
        res = MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

        # target body
        robot_group = inputs.robot_group[0]
        if (robot_group != MotionPlanRequest.LEFT_GRIPPER) and (robot_group != MotionPlanRequest.RIGHT_GRIPPER) and (robot_group != MotionPlanRequest.BOTH_GRIPPER) and (robot_group != MotionPlanRequest.LEFT_ARM) and (robot_group != MotionPlanRequest.RIGHT_ARM) and (robot_group != MotionPlanRequest.BOTH_ARM):
            return res
 
        else:
            # create open-close motion trajectory manually
            res.jointTrajectory.header.stamp = rospy.Time().now()
            pos_start = pos_end = JointTrajectoryPoint()                
            pos_end.time_from_start = rospy.Duration(1.0)
            
            if robot_group == MotionPlanRequest.LEFT_GRIPPER or robot_group == MotionPlanRequest.LEFT_ARM:    
                res.jointTrajectory.joint_names = self.openclose_joints.get('left_joint') 
                pos_start.positions = self.openclose_joints.get('left_open')
                pos_end.positions = self.openclose_joints.get('left_close')

            elif robot_group == MotionPlanRequest.RIGHT_GRIPPER or robot_group == MotionPlanRequest.RIGHT_ARM:    
                res.jointTrajectory.joint_names = self.openclose_joints.get('right_joint')
                pos_start.positions = self.openclose_joints.get('right_open')
                pos_end.positions = self.openclose_joints.get('right_close')  

            elif robot_group == MotionPlanRequest.BOTH_GRIPPER or robot_group == MotionPlanRequest.BOTH_ARM:
                res.jointTrajectory.joint_names = self.openclose_joints.get('right_joint') + self.openclose_joints.get('left_joint') 
                pos_start.positions = self.openclose_joints.get('right_open') + self.openclose_joints.get('left_open')
                pos_end.positions = self.openclose_joints.get('right_close') + self.openclose_joints.get('left_close')

            res.jointTrajectory.points.append(pos_start)         
            res.jointTrajectory.points.append(pos_end)
            res.planResult = MotionPlanResponse.SUCCESS
            return res