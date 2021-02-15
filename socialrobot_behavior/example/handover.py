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

    # arm type
    plan_req.inputs.targetBody = plan_req.inputs.LEFT_ARM
    plan_req.planner_name = "handover"

    # set approach direction
    plan_req.inputs.approachDirection = plan_req.inputs.APPROACH_SIDE

    # add obstacles
    #SKKU robot
    if robot_name == 'skkurobot':
        # juice
        obs1 = BoundingBox3D()
        c1 = Pose()
        c1.position.x = +5.1475e-01
        c1.position.y = -5.7710e-02
        c1.position.z = +8.2867e-01
        c1.orientation.x = 0
        c1.orientation.y = 0
        c1.orientation.z = 0
        c1.orientation.w = 1.0
        obs1.center = c1
        v1 = Vector3()
        v1.x = 0.082402
        v1.y = 0.079344
        v1.z = 0.23814
        obs1.size = v1

        # milk
        obs2 = BoundingBox3D()
        c2 = Pose()
        c2.position.x = +7.5000e-01
        c2.position.y = -8.2703e-02
        c2.position.z = +8.2735e-01
        c2.orientation.x = 0
        c2.orientation.y = 0
        c2.orientation.z = 0
        c2.orientation.w = 1.0
        obs2.center = c2
        v2 = Vector3()
        v2.x = 0.082402
        v2.y = 0.079344
        v2.z = 0.23814
        obs2.size = v2

        # table
        obs3 = BoundingBox3D()
        c = Pose()
        c.position.x = 0.650006
        c.position.y = 8.80659e-06
        c.position.z = 0.365011
        c.orientation.x = 0
        c.orientation.y = 0
        c.orientation.z = 0.707
        c.orientation.w = 0.707
        obs3.center = c
        v = Vector3()
        v.x = 1.13422
        v.y = 0.708874
        v.z = 0.69
        obs3.size = v

    #Social_robot
    elif robot_name == 'social_robot':
        # red_gotica
        obs1 = BoundingBox3D()
        c1 = Pose()
        c1.position.x = +3.0000e-01
        c1.position.y = +2.0000e-01
        c1.position.z = +8.2750e-01
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
        c2.position.x = +4.0000e-01
        c2.position.y = -1.5003e-02
        c2.position.z = +8.2886e-01
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

        # obj_bakey
        obs3 = BoundingBox3D()
        c3 = Pose()
        c3.position.x = +3.0000e-01
        c3.position.y = -9.9997e-02
        c3.position.z = +8.2886e-01
        c3.orientation.x = 1.31936e-05
        c3.orientation.y = 2.20794e-10
        c3.orientation.z = 6.07222e-07
        c3.orientation.w = 1
        obs3.center = c3
        v3 = Vector3()
        v3.x = 0.0618015
        v3.y = 0.059508
        v3.z = 0.23814
        obs3.size = v3

        # table
        obs4 = BoundingBox3D()
        c4 = Pose()
        c4.position.x = 0.550006
        c4.position.y = 8.80659e-06
        c4.position.z = 0.365011
        c4.orientation.x = 0
        c4.orientation.y = 0
        c4.orientation.z = 0.707
        c4.orientation.w = 0.707
        obs4.center = c4
        v4 = Vector3()
        v4.x = 1.1342161893844604
        v4.y = 0.7088739275932312
        v4.z = 0.6899999976158142
        obs4.size = v4

    # add obstacles
    plan_req.inputs.obstacle_ids = ['obj_red_gotica', 'obj_table']
    plan_req.inputs.targetObject = ['obj_red_gotica']
    plan_req.inputs.obstacles.append(obs1)
    plan_req.inputs.obstacles.append(obs4)

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
