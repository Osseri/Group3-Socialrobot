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
        armplan_srv = '/motion_plan/move_arm'

        self.srv_plan = rospy.ServiceProxy(armplan_srv, MotionPlan)

        self.input_args = ['robot_group',
                            'static_object',
                            'dynamic_object']
        self.hardware_group = ['arm']

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
        rospy.loginfo("Calculating move arm motion..")
        res = self._call_ros_service(inputs)
        return res        

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True

    def _call_ros_service(self, requirements):
        try:
            plan_req = MotionPlanRequest()
            plan_res = MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

           # target body
            robot_group = requirements.robot_group[0]    
            if robot_group == MotionPlanRequest.RIGHT_ARM or robot_group == MotionPlanRequest.RIGHT_GRIPPER:
                robot_group = MotionPlanRequest.RIGHT_ARM
            elif robot_group == MotionPlanRequest.LEFT_ARM or robot_group == MotionPlanRequest.LEFT_GRIPPER:
                robot_group = MotionPlanRequest.LEFT_ARM
            elif robot_group == MotionPlanRequest.BOTH_ARM:
                robot_group = MotionPlanRequest.BOTH_ARM
            plan_req.targetBody = robot_group

            # get bounding box of target object
            if len(requirements.target_object) > 0:
                for obj in requirements.target_object:
                    plan_req.targetObject = [obj.id]

            # set obstacles
            obstacles = requirements.static_object + requirements.dynamic_object
            for obs in obstacles:
                plan_req.obstacle_ids.append(obs.id)
                plan_req.obstacles.append(obs.bb3d)

            # target pose
            goal_pose = requirements.goal_position
            if goal_pose == []:
                rospy.logerr("[MoveArm] goal position is not decided.")
                return plan_res
            
            target_pose = goal_pose[0].pose
            target_joint_state = goal_pose[0].joint_state
            if target_joint_state.position:
                plan_req.goalType = MotionPlanRequest.JOINT_SPACE_GOAL
                plan_req.targetJointState = [target_joint_state]
            else:
                plan_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL
                plan_req.targetPose = [target_pose]

            # request to motion planner
            plan_res = self.srv_plan(plan_req)
            return plan_res

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            plan_res.planResult = MotionPlanResponse.ERROR_FAIL
            return plan_res


            