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

class MoveArmTest():

    def __init__(self):
        obj_topic = "/socialrobot/perception/objects"
        self.detected_objects = None

        # 
        rospy.Subscriber(obj_topic, social_robot_msg.Objects, self._callback_objects)
        rospy.sleep(1.0)

    def _callback_objects(self, data):        
        if data.detected_objects != []:
            self.detected_objects = data.detected_objects
        return

    def get_motion(self, target_object, request):
        # if detected object replace
        if self.detected_objects != None:
            for obj in self.detected_objects:
                request.requirements.dynamic_object.append(obj)
                if obj.id == target_object:
                    request.requirements.target_object.append(obj)
        
        # get motion
        motion_srv = rospy.ServiceProxy('/behavior/get_motion', GetMotion)
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
    example = MoveArmTest()

    plan_req = GetMotionRequest()
    plan_req.requirements.name = "liftupobject" # liftupobject , liftdownobject

    # arm type
    plan_req.requirements.robot_group = [plan_req.requirements.LEFT_ARM]

    grasped_object = 'obj_red_gotica'
    # add obstacles from topic
    motion_plan = example.get_motion(grasped_object, plan_req)
    example.set_motion(motion_plan)

    print(motion_plan.motion.jointTrajectory.joint_names)
    for pos in motion_plan.motion.jointTrajectory.points:
        print(pos.positions)
