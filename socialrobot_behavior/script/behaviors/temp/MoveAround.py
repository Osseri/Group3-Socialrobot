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

class MoveAroundBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(MoveAroundBehavior, self).__init__(name, **params)
        
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

        # fridge
        pos_fridge = Pose()   
        pos_fridge.position.x = -0.4
        pos_fridge.position.y = -1.1
        pos_fridge.orientation.x = 0.0
        pos_fridge.orientation.y = 0.0
        pos_fridge.orientation.z = -0.707
        pos_fridge.orientation.w = 0.707

        # sink
        pos_sink = Pose()   
        pos_sink.position.x = -1.16
        pos_sink.position.y = -0.3577
        pos_sink.orientation.x = 0.0
        pos_sink.orientation.y = 0.0
        pos_sink.orientation.z = 1
        pos_sink.orientation.w = 0

        # table
        pos_table = Pose()   
        pos_table.position.x = -0.4
        pos_table.position.y = 0.0
        pos_table.orientation.x = 0.0
        pos_table.orientation.y = 0.0
        pos_table.orientation.z = 0.0
        pos_table.orientation.w = 1.0

        #temp
        pos_temp = Pose()  
        pos_temp.position.x = -1.3
        pos_temp.position.y = 1.1
        pos_temp.orientation.x = 0.0
        pos_temp.orientation.y = 0.0
        pos_temp.orientation.z = 0.9
        pos_temp.orientation.w = -0.4


        waypoints.append(pos_temp)
        waypoints.append(pos_sink)
        waypoints.append(pos_fridge)
        waypoints.append(pos_table)
        self.goal_pose = waypoints    
        return True

            

            