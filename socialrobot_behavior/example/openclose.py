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

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('behavior_example')

    plan_req = GetMotionRequest()
    
    # set gripper and action
    #action_name = "open_hand"
    action_name = "close_hand"
    plan_req.inputs.targetBody = plan_req.inputs.LEFT_GRIPPER
    
    # pose template
    gripper_behavior =''
    gripper_open = [0.0, 0.0]
    gripper_close = [1.0, 0.0]

    plan_req.planner_name = "openclose"
    if action_name == "open_hand":
        gripper_behavior = gripper_open
    elif action_name == "close_hand":
        gripper_behavior = gripper_close     
    gripper_box = [0.0, 0.0]
    gripper_circle = [0.0, 0.3]
    gripper_hook = [0.0, 1.0]

    # set grasp pose
    plan_req.inputs.graspPose = map(lambda x,y: x+y, gripper_behavior, gripper_box) 

    # get motion
    motion_srv = rospy.ServiceProxy('/behavior/get_motion',GetMotion)
    res = motion_srv(plan_req)

    # set behavior
    if res.result:
        behavior_srv = rospy.ServiceProxy('/behavior/set_behavior', SetBehavior)
        behavior_req = SetBehaviorRequest()
        behavior_req.header.frame_id = plan_req.planner_name
        behavior_req.trajectory = res.motion.jointTrajectory
        behavior_res = behavior_srv(behavior_req)
