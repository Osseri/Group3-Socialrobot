#!/usr/bin/env python
import roslib
roslib.load_manifest('socialrobot_behavior')

import os
import os.path
import sys
import signal
import rospy
import rosparam
import math

from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_msgs.msg import Position
from socialrobot_motion.srv import *
from socialrobot_hardware.srv import *
from socialrobot_behavior.srv import *

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('behavior_example')

    plan_req = GetMotionRequest()
    
    # fill behavior requirements
    plan_req.requirements.name = "openhand" # closehand, openhand
    plan_req.requirements.robot_group = [plan_req.inputs.BOTH_GRIPPER] # RIGHT_GRIPPER, LEFT_GRIPPER, BOTH_GRIPPER

    # OPTIONAL: if you want to set gripper joints manually
    # plan_req.requirements.constraints.append("POSITION")
    # goal_pos = Position()
    # goal_pos.joint_state.name = ['LFinger_1', 'LFinger_2', 'LFinger_3']
    # goal_deg = [-20, -20, -20]  
    # # goal_deg = [-10, -10, 20]
    # # goal_deg = [-10, -10, 20]
    # goal_pos.joint_state.position = [i*math.pi/180 for i in goal_deg]
    # plan_req.requirements.goal_position.append(goal_pos)
    
    # get motion
    motion_srv = rospy.ServiceProxy('/behavior/get_motion',GetMotion)
    res = motion_srv(plan_req)

    # set behavior
    if res.result:
        behavior_srv = rospy.ServiceProxy('/behavior/set_behavior', SetBehavior)
        behavior_req = SetBehaviorRequest()
        behavior_req.behavior_name = plan_req.requirements.name
        behavior_req.trajectory = res.motion.jointTrajectory
        behavior_res = behavior_srv(behavior_req)
    
    print(res.motion.jointTrajectory.joint_names)
    for pos in res.motion.jointTrajectory.points:
        print(pos.positions)