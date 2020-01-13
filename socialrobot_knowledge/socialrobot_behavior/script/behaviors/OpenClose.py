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
            self.service_type = None

        # barrett has 4 dof joints
        # PI = math.pi
        # self.barrett_joints = {'names': ('_bhand_finger_1_prox_joint','_bhand_finger_2_prox_joint','_bhand_finger_3_med_joint','_bhand_finger_3_dist_joint'),
        #                     'init_value': (0.0, 0.0, 0.0, 0.0),
        #                     'range': (-PI, PI, -2, -1),
        #                     'direction': (1, 1, 1, 1) }
        # self.barrett_eigen = {'open_close': (0.0, 0.0, 1.0, 1.0),
        #                     'spread': (1.0, 1.0, 0.0, 0.0) }
        self.barrett_openclose_joints = {
            'left_joint': ('Barrett_openCloseJoint0_left', 'Barrett_openCloseJoint_left'),
            'right_joint': ('Barrett_openCloseJoint0_right', 'Barrett_openCloseJoint_right'),
            'init_value': (0.0, 0.0),
            'min_value': (-0.2, -0.2),
            'max_value': (0.0, 0.0),
            'direction': (1, 1)
        }

        self.robocare_openclose_joints = {
            'left_joint': ('LFinger_1', 'LFinger_2', 'LFinger_3'),
            'right_joint': ('RFinger_1', 'RFinger_2', 'RFinger_3'),
            'init_value': (0.0, 0.0, 0.0),
            'min_value': (-0.4, -0.4, -0.4),
            'max_value': (0.4, 0.4, 0.4),
            'direction': (1, 1, -1)
        }

        robot_name = rosparam.get_param("/robot_name")
        self.openclose_joints = {}
        if robot_name == 'skkurobot':
            self.openclose_joints = self.barrett_openclose_joints
        elif robot_name == 'social_robot':
            self.openclose_joints = self.robocare_openclose_joints


    def check_requirement(self):
        rospy.loginfo('checking...%s' % self._name)
        self.service_list = rosservice.get_service_list()

        if self.service_name in self.service_list:
            return True

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
            
        return -1

    def get_motion(self, inputs):
        '''
        return the trajectory 
        '''
        rospy.loginfo("Calculating grasp motion..")

        ret , srv_response = self._call_ros_service(inputs)
        if ret == MotionPlanResponse.SUCCESS:
            rospy.loginfo("Grasp planning is done.")
            self.motion_trajectory = srv_response.jointTrajectory
        else:
            rospy.loginfo("Grasp planning if failed.")
        return ret

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True

    def _call_ros_service(self, inputs):

        # service_name: at this time, it is hard coded.
        service_name = '/motion_plan/gripper_open_close'
        #rospy.wait_for_service(service_name)
        grasp_pose = inputs.graspPose
        if grasp_pose:
            try:
                # call the ros service
                plan_gripper = rospy.ServiceProxy(service_name, MotionPlan)

                plan_req = MotionPlanRequest()
                    
                # target body
                body_type = inputs.targetBody
                if (body_type != MotionPlanRequest.LEFT_GRIPPER) and (body_type != MotionPlanRequest.RIGHT_GRIPPER):
                    return (MotionPlanResponse.ERROR_INPUT, None)
                plan_req.targetBody = body_type

                # target pose
                target_joint_state = JointState()
                
                if body_type == MotionPlanRequest.LEFT_GRIPPER:
                    target_joint_state.name = list(self.openclose_joints.get('left_joint'))
                elif body_type == MotionPlanRequest.RIGHT_GRIPPER:
                    target_joint_state.name = list(self.openclose_joints.get('right_joint'))
                
                # positions = self.barrett_joints.get('init_value')
                # delta = self.barrett_joints.get('range')
                # openclose_idx = self.barrett_eigen.get('open_close')
                # spread_idx = self.barrett_eigen.get('spread')

                # positions = map(lambda p,dp,o_idx,s_idx: o_idx*grasp_pose[0]*dp + s_idx*grasp_pose[1]*dp + p, positions, delta, openclose_idx, spread_idx)
                # target_joint_state.position = list(positions)
                
                if grasp_pose[0] > 0:   # close gripper
                    positions = map(lambda (x,y): x * y, zip(self.openclose_joints.get('min_value'), self.openclose_joints.get('direction')))
                else:                   # open gripper                    
                    positions = map(lambda (x,y): x * y, zip(self.openclose_joints.get('max_value'), self.openclose_joints.get('direction')))
                target_joint_state.position = list(positions)

                plan_req.targetJointState = target_joint_state
                plan_req.goalType = MotionPlanRequest.JOINT_SPACE_GOAL
                
                
                result = plan_gripper(plan_req)
                return (MotionPlanResponse.SUCCESS, result)

            except rospy.ServiceException as e:
                rospy.logerr('Service call failed: %s' % e)
                return (MotionPlanResponse.ERROR_FAIL, None)    
        else:
            return (MotionPlanResponse.ERROR_FAIL, None)  