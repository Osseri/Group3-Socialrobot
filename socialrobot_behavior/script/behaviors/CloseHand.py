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

class CloseHandBehavior(BehaviorBase):
    ''' Gripper Open-Close  '''
    def __init__(self, name, **params):
        super(CloseHandBehavior, self).__init__(name, **params)
        
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
            'left_close': [0.174533, 0.174533, 0.174533],
            'right_open': [-0.34, -0.34, 0.34],
            'right_close': [0.174533, 0.174533, -0.174533]
        }

        self.robot_name = rosparam.get_param("/robot_name")
        self.openclose_joints = {}
        if self.robot_name == 'skkurobot':
            self.openclose_joints = self.skkurobot_gripper
        elif self.robot_name == 'social_robot':
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

    def _call_ros_service(self, requirements):
        res = MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

        # check constraints CURRENT and POSITION mode 
        const = False
        if len(requirements.constraints)==1:
            const = requirements.constraints[0]

        # target body
        robot_group = requirements.robot_group[0]
        if (robot_group != MotionPlanRequest.LEFT_GRIPPER) and (robot_group != MotionPlanRequest.RIGHT_GRIPPER) and (robot_group != MotionPlanRequest.BOTH_GRIPPER) and (robot_group != MotionPlanRequest.LEFT_ARM) and (robot_group != MotionPlanRequest.RIGHT_ARM) and (robot_group != MotionPlanRequest.BOTH_ARM):
            return res
             
        res.jointTrajectory.header.stamp = rospy.Time().now()
        
        # create trajectories
        pos_start = JointTrajectoryPoint()
        pos_end = JointTrajectoryPoint()

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

        if len(requirements.goal_position)>0:  
            for i, goal_pos in enumerate(requirements.goal_position):                
                joint_state = self.get_group_pos(res.jointTrajectory.joint_names, goal_pos.joint_state)

                # joint state to joint trajectory
                res.jointTrajectory.joint_names = joint_state.name
                pos_end = JointTrajectoryPoint()
                pos_end.positions = joint_state.position
                pos_end.time_from_start = rospy.Duration(1.0 * i)
                res.jointTrajectory.points.append(pos_end)            

        if(const == 'POSITION'):
            self.add_trajectory_points(res.jointTrajectory, pos_start, pos_end)
        else:
            pos_end.time_from_start = rospy.Duration(0.0)
            res.jointTrajectory.points.append(pos_end)    

        res.planResult = MotionPlanResponse.SUCCESS
        return res

    def get_group_pos(self, gripper_joint_names, goal_pos):
        joint_state = JointState()

        for name in gripper_joint_names:
            if name in goal_pos.name:
                idx = goal_pos.name.index(name)
                joint_state.name.append(name)
                joint_state.position.append(goal_pos.position[idx])

        return joint_state

    def add_trajectory_points(self, trajectory, start_point, goal_point):
        '''
        create gripper trajectory for robocare robot
        '''
        duration = 1.0 #s
        trajectory.points = []
        if len(start_point.positions)>3: #dual hand
            # position mode joints first
            start_point.positions[0] = goal_point.positions[0]
            start_point.positions[1] = goal_point.positions[1]
            start_point.positions[3] = goal_point.positions[3]
            start_point.positions[4] = goal_point.positions[4]

            for i in range(5):
                start_point.time_from_start = rospy.Duration(0.0)
                if i==4:
                    start_point.time_from_start = rospy.Duration(0.1)
                trajectory.points.append(start_point)
            
            for i in range(5):
                goal_point.time_from_start = rospy.Duration(0.1)
                trajectory.points.append(goal_point)

        else: #single hand
            # position mode joints first
            start_point.positions[0] = goal_point.positions[0]
            start_point.positions[1] = goal_point.positions[1]

            for i in range(5):
                start_point.time_from_start = rospy.Duration(0.0)
                if i==4:
                    start_point.time_from_start = rospy.Duration(0.1)
                trajectory.points.append(start_point)
            
            for i in range(5):
                goal_point.time_from_start = rospy.Duration(0.1)
                trajectory.points.append(goal_point)

