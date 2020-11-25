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

class MoveArmBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(MoveArmBehavior, self).__init__(name, **params)
        
        if self._hardware_if == 'vrep':
            self.service_name = '/sim_interface/set_motion'
            self.service_type = VrepSetJointTrajectory
        elif self._hardware_if == 'hw':
            self.service_name = '/hw_interface/set_motion'
            self.service_type = SetJointTrajectory

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
        rospy.loginfo("Calculating manipulator motion..")

        res = self._call_ros_service(inputs)
        if res.planResult == MotionPlanResponse.SUCCESS:
            rospy.loginfo("Motion planning is done.")
            self.motion_trajectory = res.jointTrajectory
        else:
            rospy.loginfo("Motion planning is failed.")
        return res        

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True

    def _call_ros_service(self, inputs):
        service_name = '/motion_plan/move_arm'   
        plan_res = MotionPlanResponse()
        try:
            # call the ros service
            plan_arm = rospy.ServiceProxy(service_name, MotionPlan)
            plan_req = MotionPlanRequest()
            
            # target body
            body_type = inputs.targetBody
            if body_type != MotionPlanRequest.LEFT_ARM and body_type != MotionPlanRequest.RIGHT_ARM and body_type != MotionPlanRequest.BOTH_ARM:
                rospy.logerr('[MoveArm] Target robot part is not Manipulator!')
                return (MotionPlanResponse.ERROR_INPUT, None)
            plan_req.targetBody = body_type

            # target pose
            target_pose = inputs.poseGoal
            target_joint_state = inputs.jointGoal
            plan_req.targetPose = target_pose
            if target_joint_state.position:
                plan_req.targetJointState = target_joint_state
                plan_req.goalType = MotionPlanRequest.JOINT_SPACE_GOAL
            else:
                plan_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL

            # target object (move object within hand)
            target_object = inputs.targetObject
            plan_req.targetObject = target_object

            # obstacles
            plan_req.obstacle_ids = inputs.obstacle_ids
            plan_req.obstacles = inputs.obstacles
            
            plan_res = plan_arm(plan_req)

            if plan_res.planResult == MotionPlanResponse.SUCCESS:
                return plan_res

            plan_res.planResult = MotionPlanResponse.ERROR_NO_SOLUTION
            return plan_res

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            plan_res.planResult = MotionPlanResponse.ERROR_FAIL
            return plan_res


            