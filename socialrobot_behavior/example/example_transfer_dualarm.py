#!/usr/bin/env python
import roslib
roslib.load_manifest('socialrobot_behavior')

import os
import os.path
import sys
import signal
import rospy
import rosparam

from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_motion.srv import *
from socialrobot_hardware.srv import *
from socialrobot_behavior.srv import *
from socialrobot_msgs import msg as social_robot_msg
import moveit_commander

class AppraochArmTest():

    def __init__(self):
        obj_topic = "/socialrobot/perception/objects"
        joint_topic = "/joint_states"
        self.detected_objects = None
        self.joint_states = None
        self.scene = moveit_commander.PlanningSceneInterface()     
        rospy.sleep(1.0)  

        # 
        rospy.Subscriber(joint_topic, JointState, self._callback_joint_states)
        rospy.sleep(1.0)

    def _callback_joint_states(self, data):
        self.joint_states = data
        return

    def get_motion(self, request):
        # 
        if self.joint_states == None:
            while(self.joint_states == None):
                rospy.sleep(rospy.Duration(0.2))  
        
        request.requirements.current_position.joint_state = self.joint_states
        # get motion
        motion_srv = rospy.ServiceProxy('/behavior/get_motion', GetMotion)
        #print(request)
        res = motion_srv(request)
        return res

    def set_motion(self, plan):
        # set behavior
        if plan.result:
            behavior_srv = rospy.ServiceProxy('/behavior/set_behavior', SetBehavior)
            behavior_req = SetBehaviorRequest()
            behavior_req.behavior_name = plan_req.requirements.name
            behavior_req.trajectory = plan.motion.jointTrajectory
            behavior_res = behavior_srv(behavior_req)
            return behavior_res


##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('behavior_example')
    robot_name = rosparam.get_param("/robot_name")
    example = AppraochArmTest()
    rospy.sleep(1.0)

    plan_req = GetMotionRequest()
    plan_req.requirements.name = "transferobject"

    # arm type
    plan_req.requirements.robot_group = [plan_req.requirements.BOTH_ARM]

    # add obstacles from topic
    motion_plan = example.get_motion(plan_req)
    print(motion_plan)
    example.set_motion(motion_plan)
