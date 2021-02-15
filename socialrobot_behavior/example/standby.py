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
    robot_name = rosparam.get_param("/robot_name")

    plan_req = GetMotionRequest()
        
    # goal type
    goalType = "JOINT_SPACE_GOAL"

    if robot_name == 'social_robot':
        # red_gotica
        obs1 = BoundingBox3D()
        c1 = Pose()
        c1.position.x = 0.375
        c1.position.y = 0.214997
        c1.position.z = 0.828779
        c1.orientation.x = 1.31936e-05
        c1.orientation.y = 2.20794e-10
        c1.orientation.z = 6.07222e-07
        c1.orientation.w = 1
        obs1.center = c1
        v1 = Vector3()
        v1.x = 0.0618015
        v1.y = 0.059508
        v1.z = 0.23814
        obs1.size = v1

        # gotica
        obs2 = BoundingBox3D()
        c2 = Pose()
        c2.position.x = 0.385
        c2.position.y = 0
        c2.position.z = 0.827404
        c2.orientation.x = 1.31627e-05
        c2.orientation.y = 2.26816e-10
        c2.orientation.z = -1.15535e-18
        c2.orientation.w = 1.0
        obs2.center = c2
        v2 = Vector3()
        v2.x = 0.065
        v2.y = 0.065
        v2.z = 0.23544
        obs2.size = v2
        
        # table
        obs3 = BoundingBox3D()
        c = Pose()
        c.position.x = 0.550006
        c.position.y = 8.80659e-06
        c.position.z = 0.365011
        c.orientation.x = 0
        c.orientation.y = 0
        c.orientation.z = 0.707
        c.orientation.w = 0.707
        obs3.center = c
        v = Vector3()
        v.x = 1.1342161893844604
        v.y = 0.7088739275932312
        v.z = 0.6899999976158142
        obs3.size = v
        plan_req.inputs.obstacles.append(obs1)
        plan_req.inputs.obstacles.append(obs2)
        plan_req.inputs.obstacles.append(obs3)

        # target pose
        target_joint_state = JointState()
        target_joint_state.name = [
            'Waist_Roll', 'Waist_Pitch', 'Shoulder_Pitch', 'Shoulder_Roll', 'Elbow_Pitch', 'Elbow_Yaw',
            'Wrist_Pitch', 'Wrist_Roll'
        ]

        target_joint_state.position = [0, 0, 0, -1.23, 0, 0, 0, 0]
        plan_req.inputs.jointGoal = target_joint_state

    # arm type
    plan_req.inputs.targetBody = plan_req.inputs.LEFT_ARM
    plan_req.planner_name = "standby"


    # add obstacles 
    plan_req.inputs.obstacle_ids = ['obj_red_gotica','obj_gotica', 'obj_table']

    # get motion
    motion_srv = rospy.ServiceProxy('/behavior/get_motion', GetMotion)
    res = motion_srv(plan_req)

    # set behavior
    if res.result:
        behavior_srv = rospy.ServiceProxy('/behavior/set_behavior', SetBehavior)
        behavior_req = SetBehaviorRequest()
        behavior_req.header.frame_id = plan_req.planner_name
        behavior_req.trajectory = res.motion.jointTrajectory
        behavior_res = behavior_srv(behavior_req)
