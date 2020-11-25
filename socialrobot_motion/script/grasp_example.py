#!/usr/bin/env python
import roslib
roslib.load_manifest('socialrobot_motion')

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

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('example')

    plan_req = MotionPlanRequest()

    # arm type
    plan_req.targetBody = MotionPlanRequest.LEFT_GRIPPER

    # obstacles 
    plan_req.obstacle_ids = ['obj_table','obj_juice']

    # table
    obs = BoundingBox3D()
    c = Pose()
    c.position.x = 1.025
    c.position.y = -0.17499
    c.position.z = 0.365
    c.orientation.x = 0
    c.orientation.y = -0.707115888596
    c.orientation.z = 0
    c.orientation.w = 0.707097649574
    obs.center = c
    v = Vector3()
    v.x = 0.730000257492
    v.y = 0.750000059605
    v.z = 1.20000016689
    obs.size = v
    plan_req.obstacles.append(obs)

    # juice
    obs1 = BoundingBox3D()
    c1 = Pose()
    c1.position.x = 0.514697432518
    c1.position.y = -0.0829000025988
    c1.position.z = 0.829887330532
    c1.orientation.x = -6.52484368402e-06
    c1.orientation.y = -1.13013602459e-05
    c1.orientation.z = 0.5
    c1.orientation.w = 0.866025388241
    obs1.center = c1
    v1 = Vector3()
    v1.x = 0.0500000007451
    v1.y = 0.0500000007451
    v1.z = 0.20000000298
    obs1.size = v1
    plan_req.obstacles.append(obs1)



    # target object
    plan_req.targetObject = 'obj_milk'
    # milk
    c2 = Pose()
    c2.position.x = 0.714797377586
    c2.position.y = -0.0826999992132
    c2.position.z = 0.829892516136
    c2.orientation.x = -7.56194354664e-17
    c2.orientation.y = -1.30496864585e-05
    c2.orientation.z = 1.84444939782e-18            
    c2.orientation.w = 1.0              
    
    plan_req.targetObjectPose = c2
   

    plan_srv = rospy.ServiceProxy('/motion_plan/grasp_plan', MotionPlan)

    plan_res = plan_srv(plan_req)
    print(plan_res)

    #move_srv = rospy.ServiceProxy('/sim_interface/set_motion', VrepSetJointTrajectory)
    #move_req = VrepSetJointTrajectoryRequest()
    #move_req.trajectory = plan_res.jointTrajectory
    #move_req.duration = 2.0

    #move_srv(move_req)
