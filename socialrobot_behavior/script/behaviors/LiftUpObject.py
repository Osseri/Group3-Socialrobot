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
import tf
import tf.transformations as tfm
import numpy as np 
import random
from behavior import BehaviorBase

class LiftUpObjectBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(LiftUpObjectBehavior, self).__init__(name, **params)
        
        if self._hardware_if == 'vrep':
            self.service_name = '/sim_interface/set_motion'
            self.service_type = VrepSetJointTrajectory
        elif self._hardware_if == 'hw':
            self.service_name = '/hw_interface/set_motion'
            self.service_type = SetJointTrajectory
        armplan_srv = '/motion_plan/move_arm'

        self.srv_plan = rospy.ServiceProxy(armplan_srv, MotionPlan)
        self.listener = tf.TransformListener()
        self.talker = tf.TransformBroadcaster(queue_size=10)
        robot_name = rosparam.get_param("/robot_name")
        if robot_name == 'skkurobot':
            self.gripper_info = {
                'left': '7dof_RISE_wrist_link',
                'right': '6dof_connection_link'
            }
        elif robot_name == 'social_robot':
            self.gripper_info = {
                'left': 'LHand_base',
                'right': 'RHand_base'
            }

        self.input_args = ['robot_group',
                            'target_object'
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
            plan_req.targetBody = robot_group

            # grasped object
            if len(requirements.target_object) == 1:
                plan_req.targetObject = [requirements.target_object[0].id]

            # set obstacles
            obstacles = requirements.static_object + requirements.dynamic_object
            for obs in obstacles:
                plan_req.obstacle_ids.append(obs.id)
                plan_req.obstacles.append(obs.bb3d)

            # target pose
            waypoints = self.get_goal_pose(robot_group)            
            
            plan_req.targetPose = waypoints

            # request to motion planner
            plan_res = self.srv_plan(plan_req)
            return plan_res

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            plan_res.planResult = MotionPlanResponse.ERROR_FAIL
            return plan_res


    def get_goal_pose(self, robot_group):
        '''
        get goal pose array to lift up(down)
        '''
        waypoints = []
        # get transforms between basefootprint and wrist
        is_trans = False
        while is_trans == False:
            try:
                if robot_group is MotionPlanRequest.LEFT_ARM or robot_group is MotionPlanRequest.LEFT_ARM_WITHOUT_WAIST:
                    base_to_wrist_trans, base_to_wrist_rot = self.listener.lookupTransform(
                        '/base_footprint', self.gripper_info['left'], rospy.Time(0))
                elif robot_group is MotionPlanRequest.RIGHT_ARM or robot_group is MotionPlanRequest.RIGHT_ARM_WITHOUT_WAIST:
                    base_to_wrist_trans, base_to_wrist_rot = self.listener.lookupTransform(
                        '/base_footprint', self.gripper_info['right'], rospy.Time(0))
                is_trans = True
            except:
                pass
        is_trans = False

        # create goal pose 
        goal_pose = Pose()
        goal_pose.position.x = base_to_wrist_trans[0]
        goal_pose.position.y = base_to_wrist_trans[1]
        goal_pose.position.z = base_to_wrist_trans[2] + 0.05

        goal_pose.orientation.x = base_to_wrist_rot[0]
        goal_pose.orientation.y = base_to_wrist_rot[1]
        goal_pose.orientation.z = base_to_wrist_rot[2]
        goal_pose.orientation.w = base_to_wrist_rot[3]
        waypoints.append(goal_pose)

        goal_pose.position.z += 0.05
        waypoints.append(goal_pose)

        return waypoints
            