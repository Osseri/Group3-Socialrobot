import abc
from abc import ABCMeta
from six import with_metaclass 
import rospy
import rosparam
import actionlib
import math

from socialrobot_motion.srv import *
from socialrobot_behavior.srv import *
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from behavior import BehaviorBase

class ApproachBaseBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(ApproachBaseBehavior, self).__init__(name, **params)
        
        self.goal_pose = []

    def check_requirement(self):
        rospy.loginfo('checking...%s' % self._name)
        return True

    def prepare_behavior(self):
        rospy.loginfo('preparing...%s' % self._name)
        return True

    def run_behavior(self):           
        return True

    def get_motion(self, inputs):
        '''
        return the trajectory 
        '''
        rospy.loginfo("Calculating mobile motion..")

        res = MotionPlanResponse()
        if self._call_ros_service(inputs):
            res.planResult == MotionPlanResponse.SUCCESS
            rospy.loginfo("Motion planning is done.")
        else:
            res.planResult = MotionPlanResponse.ERROR_NO_SOLUTION
            rospy.loginfo("Motion planning is failed.")
        return res        

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True

    def _call_ros_service(self, inputs):

        return self._get_goal_position(inputs)

    def _get_goal_position(self, inputs):
        #inputs.obstacle_ids
        #inputs.obstacles
        body = inputs.targetBody
        target = inputs.targetObject[0]
        offset = 0.0

        if body == inputs.LEFT_ARM or body == inputs.LEFT_GRIPPER:
            offset = -0.1
        elif body == inputs.RIGHT_ARM or body == inputs.RIGHT_GRIPPER:
            offset = 0.1

        # calculate goal position for approaching to object
        #TODO:
        waypoints = []  
        goal_pose = Pose()
        
        if target == 'obj_juice':
            goal_pose.position.x = -1.3000e-01
            goal_pose.position.y = 4.2500e-01 + offset
            goal_pose.orientation.w = 1.0

        elif target == 'obj_milk':
            goal_pose.position.x = -1.3000e-01
            goal_pose.position.y = 1.0000e-01 + offset
            goal_pose.orientation.w = 1.0

        elif target == 'obj_box':
            goal_pose.position.x = -1.3000e-01
            goal_pose.position.y = -3.0000e-01
            goal_pose.orientation.w = 1.0

        elif target == 'obj_fridge':
            goal_pose.position.x = -4.5000e-01
            goal_pose.position.y = -1.2500e+00
            goal_pose.orientation.x = 0.0
            goal_pose.orientation.y = 0.0
            goal_pose.orientation.z = -0.707
            goal_pose.orientation.w = 0.707
           
        waypoints.append(goal_pose)
        self.goal_pose = waypoints        

        return True

            