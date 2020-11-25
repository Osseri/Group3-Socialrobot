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

class MoveBaseBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(MoveBaseBehavior, self).__init__(name, **params)
        
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
        goal_pos = inputs.targetObject[0]

        # calculate goal position for approaching to object
        #TODO:
        waypoints = []     
        if goal_pos == 'pos_human':
            goal_pose = Pose()   
            goal_pose.position.x = -1.3000e+00
            goal_pose.position.y = +1.1000e+00
            goal_pose.orientation.x = 0
            goal_pose.orientation.y = 0
            goal_pose.orientation.z = 0.940
            goal_pose.orientation.w = 0.342      
            waypoints.append(goal_pose)
        self.goal_pose = waypoints

        return True

            

            