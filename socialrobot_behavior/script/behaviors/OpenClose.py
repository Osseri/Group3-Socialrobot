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

class OpenCloseBehavior(BehaviorBase):
    ''' Gripper Open-Close  '''
    def __init__(self, name, **params):
        super(OpenCloseBehavior, self).__init__(name, **params)
        
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
            'left_open': [-0.41, -0.41, -0.38],
            'left_close': [0.0872665, 0.0872665, 0.0872665],
            'right_open': [-0.38, -0.38, 0.40],
            'right_close': [0.0872665, 0.122173, -0.0872665]
            #'right_close': [-0.0523599, -0.0523599, 0.0523599],
            #'left_close': [-0.0523599, -0.0523599, -0.0523599]
        }

        robot_name = rosparam.get_param("/robot_name")
        self.openclose_joints = {}
        if robot_name == 'skkurobot':
            self.openclose_joints = self.skkurobot_gripper
        elif robot_name == 'social_robot':
            self.openclose_joints = self.social_robot_gripper


    def check_requirement(self):
        rospy.loginfo('checking...%s' % self._name)
        self.service_list = rosservice.get_service_list()
        if self.service_name in self.service_list:
            return True
        rospy.llogerr('cannot find %s service in the list.' % self.service_name)
        return False

    def prepare_behavior(self):
        rospy.loginfo('preparing...%s' % self._name)
        return True

    def run_behavior(self):
        rospy.loginfo('running...%s' % self._name)
        if self._hardware_if == 'vrep':
            move_srv = rospy.ServiceProxy(self.service_name, self.service_type)
            move_req = VrepSetJointTrajectoryRequest()
            move_req.trajectory = self.behavior_data.get('trajectory')
            move_req.duration = self.behavior_data.get('duration')

            res = move_srv(move_req)

            if res.result == VrepSetJointTrajectoryResponse.OK:
                return 1

        elif self._hardware_if == 'hw':
            move_srv = rospy.ServiceProxy(self.service_name, self.service_type)
            move_req = SetJointTrajectoryRequest()
            move_req.trajectory = self.behavior_data.get('trajectory')
            move_req.duration = self.behavior_data.get('duration')

            res = move_srv(move_req)
            
            if res.result == SetJointTrajectoryResponse.OK:
                return 1    
        
        return -1

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

        req = MotionPlanRequest()
        res = MotionPlanResponse()

        # target body
        body_type = inputs.targetBody
        if (body_type != MotionPlanRequest.LEFT_GRIPPER) and (body_type != MotionPlanRequest.RIGHT_GRIPPER):
            return res

        req.targetBody = body_type
        self.srv_plan(req)

        res.planResult = MotionPlanResponse.ERROR_FAIL
        grasp_pose = inputs.graspPose
        if grasp_pose:            
            try:
                # create open-close motion trajectory manually
                res.jointTrajectory.header.stamp = rospy.Time().now()
                pos_start = pos_end = JointTrajectoryPoint()                
                pos_end.time_from_start = rospy.Duration(1.0)
                
                if body_type == MotionPlanRequest.LEFT_GRIPPER:    
                    res.jointTrajectory.joint_names = self.openclose_joints.get('left_joint')
                    if grasp_pose[0] > 0:   
                        pos_start.positions = self.openclose_joints.get('left_open')
                        pos_end.positions = self.openclose_joints.get('left_close')
                    else:                                     
                        pos_start.positions = self.openclose_joints.get('left_close')
                        pos_end.positions = self.openclose_joints.get('left_open')

                elif body_type == MotionPlanRequest.RIGHT_GRIPPER:    
                    res.jointTrajectory.joint_names = self.openclose_joints.get('right_joint')
                    if grasp_pose[0] > 0:  
                        pos_start.positions = self.openclose_joints.get('right_open')
                        pos_end.positions = self.openclose_joints.get('right_close')
                    else:                     
                        pos_start.positions = self.openclose_joints.get('right_close')  
                        pos_end.positions = self.openclose_joints.get('right_open')    

                res.jointTrajectory.points.append(pos_start)         
                res.jointTrajectory.points.append(pos_end)
                res.planResult = MotionPlanResponse.SUCCESS
                return res

            except rospy.ServiceException as e:
                return res   
        else:
            return res
