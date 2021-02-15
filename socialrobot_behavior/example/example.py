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
    rospy.init_node('example')

    plan_req = MotionPlanRequest()

    # arm type
    plan_req.targetBody = MotionPlanRequest.LEFT_ARM

    # goal type
    #plan_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL
    plan_req.goalType = MotionPlanRequest.JOINT_SPACE_GOAL

    # current joint state (start state)
    current_joint_state = JointState()
    current_joint_state.name = ['right_bhand_finger_1_prox_joint', 'left_bhand_finger_2_prox_joint', '7dof_RISE_joint_7',
        '7dof_RISE_joint_6', '7dof_RISE_joint_5', '7dof_RISE_joint_4', 'left_bhand_finger_1_prox_joint',
        'right_bhand_finger_2_prox_joint', '7dof_RISE_joint_1', '7dof_RISE_joint_2', '6dof_RISE_joint_2',
        '6dof_RISE_joint_3', '6dof_RISE_joint_1', '6dof_RISE_joint_6', '7dof_RISE_joint_3', '6dof_RISE_joint_4',
        '6dof_RISE_joint_5']
    current_joint_state.position = [-1.5707963705062866, -1.5707963705062866, 0.0, 0.0, 0.0, 0.0, -1.5707963705062866, -1.5707963705062866, 0.0,0.0, 0.0, -1.5707961320877075, 1.5707963705062866, 0.0, 0.0, 1.2217304706573486, 2.384185791015625e-07]
    plan_req.currentJointState = current_joint_state

    # obstacles 
    plan_req.obstacle_ids = ['obj_table','obj_juice', 'obj_milk']

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

    # milk
    obs2 = BoundingBox3D()
    c2 = Pose()
    c2.position.x = 0.714797377586
    c2.position.y = -0.0826999992132
    c2.position.z = 0.829892516136
    c2.orientation.x = -7.56194354664e-17
    c2.orientation.y = -1.30496864585e-05
    c2.orientation.z = 1.84444939782e-18            
    c2.orientation.w = 1.0              
    obs2.center = c2
    v2 = Vector3()
    v2.x = 0.0500000007451
    v2.y = 0.0500000007451
    v2.z = 0.20000000298
    obs2.size = v2
    plan_req.obstacles.append(obs2)
   
    # target pose
    if plan_req.goalType == MotionPlanRequest.CARTESIAN_SPACE_GOAL:
        # cartesian space goal
        #x:0.374 y:-0.58 z:0.95 w:0.369 q(x):0.628 q(y):0.527 q(z):-0.439
        target_pose = Pose()
        target_pose.position.x = 0.374
        target_pose.position.y = -0.58
        target_pose.position.z = 0.88
        target_pose.orientation.w = 0.369
        target_pose.orientation.x = 0.628
        target_pose.orientation.y = 0.527
        target_pose.orientation.z = -0.439
        plan_req.targetPose = target_pose

    else:
        target_joint_state = JointState()
        target_joint_state.name = ['right_bhand_finger_1_prox_joint', 'left_bhand_finger_2_prox_joint', '7dof_RISE_joint_7',
        '7dof_RISE_joint_6', '7dof_RISE_joint_5', '7dof_RISE_joint_4', 'left_bhand_finger_1_prox_joint',
        'right_bhand_finger_2_prox_joint', '7dof_RISE_joint_1', '7dof_RISE_joint_2', '6dof_RISE_joint_2',
        '6dof_RISE_joint_3', '6dof_RISE_joint_1', '6dof_RISE_joint_6', '7dof_RISE_joint_3', '6dof_RISE_joint_4',
        '6dof_RISE_joint_5']
        target_joint_state.position = [-1.5707963705062866, -1.5707963705062866, 0.30, 0.30, 0.30, 0.0, -1.5707963705062866, -1.5707963705062866, 0.0,0.0, 0.0, -1.5707961320877075, 1.5707963705062866, 0.0, 0.0, 1.2217304706573486, 2.384185791015625e-07]
        plan_req.targetJointState = target_joint_state

    plan_srv = rospy.ServiceProxy('/motion_plan/move_arm', MotionPlan)

    plan_res = plan_srv(plan_req)

    behavior_srv = rospy.ServiceProxy('/behavior/set_behavior', SetBehavior)
    behavior_req = SetBehaviorRequest()
    behavior_req.header.frame_id = 'move_arm'
    behavior_req.trajectory = plan_res.jointTrajectory
    behavior_res = behavior_srv(behavior_req)
    print(behavior_res)
    #move_srv = rospy.ServiceProxy('set_motion', VrepSetJointTrajectory)
    #move_req = VrepSetJointTrajectoryRequest()
    #move_req.trajectory = plan_res.jointTrajectory
    #move_req.duration = 2.0

    #move_srv(move_req)
