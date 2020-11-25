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
    plan_req.targetBody = MotionPlanRequest.RIGHT_ARM

    # goal type
    #plan_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL
    plan_req.goalType = MotionPlanRequest.JOINT_SPACE_GOAL

    # current joint state (start state)
    current_joint_state = JointState()
    current_joint_state.name = ['Waist_Roll', 'Waist_Pitch', 'Head_Yaw', 'Head_Pitch', 'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch',
  'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll', 'LFinger_1', 'LFinger_2', 'LFinger_3', 'RShoulder_Pitch',
  'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll', 'RFinger_1',
  'RFinger_2', 'RFinger_3', 'active_joint_1', 'active_joint_2', 'active_joint_3', 'active_joint_4']

    current_joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3490793704986572, 0.34906792640686035, -0.34905529022216797, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.4149893701187466e-08, -1.611215338925831e-07, -6.372786742758763e-08, -1.725284448639286e-07]

    plan_req.currentJointState = current_joint_state

    # obstacles 
    plan_req.obstacle_ids = ['obj_table', 'obj_juice','obj_milk']
    #plan_req.targetObject = 'obj_juice'
    # table
    obs = BoundingBox3D()
    c = Pose()
    c.position.x = 0.57500565052
    c.position.y = 0.0250088647008
    c.position.z = 0.36501121521
    c.orientation.x = -0.499993562698
    c.orientation.y = 0.500006496906
    c.orientation.z = -0.500006496906
    c.orientation.w = -0.499993562698
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
    c1.position.x = 0.390748232603
    c1.position.y = 0.217293173075
    c1.position.z = 0.801787078381
    c1.orientation.x = -3.72598707443e-09
    c1.orientation.y = -1.3020775441e-05
    c1.orientation.z = -2.45272501579e-17
    c1.orientation.w = 1.0
    obs1.center = c1
    v1 = Vector3()
    v1.x = 0.0450000055134
    v1.y = 0.0450000055134
    v1.z = 0.143729999661
    obs1.size = v1
    plan_req.obstacles.append(obs1)

    # milk
    obs2 = BoundingBox3D()
    c2 = Pose()
    c2.position.x = 0.438998311758
    c2.position.y = 0.29629996419
    c2.position.z = 0.808184683323  
    c2.orientation.x = -3.72905573087e-09
    c2.orientation.y = -1.30315002025e-05
    c2.orientation.z = 1.12574913855e-16         
    c2.orientation.w = 1.0              
    obs2.center = c2
    v2 = Vector3()
    v2.x = 0.0450000055134
    v2.y = 0.0450000055134
    v2.z = 0.156530082226
    obs2.size = v2
    plan_req.obstacles.append(obs2)
   
    # target pose
    if plan_req.goalType == MotionPlanRequest.CARTESIAN_SPACE_GOAL:
        # cartesian space goal                
        # -6.71148e-08 0.170225, 0.9205
        # 0.14194, 0.833169, -0.472182, 0.2250454
        target_pose = Pose()
        target_pose.position.x = 0.28838533245
        target_pose.position.y = 0.299717217684
        target_pose.position.z = 0.808184742928
        target_pose.orientation.x = 0.5
        target_pose.orientation.y = 0.5
        target_pose.orientation.z = -0.5
        target_pose.orientation.w = 0.5
        plan_req.targetPose = target_pose
        # juice
        #   x: 0.243248229846
        #   y: 0.217293173075
        #   z: 0.801787078381
        # milk
        #x: 0.28838533245
        #y: 0.299717217684
        #z: 0.808184742928


    else:
        target_joint_state = JointState()
        target_joint_state.name = ['7dof_RISE_joint_1', '7dof_RISE_joint_2', '7dof_RISE_joint_3', '7dof_RISE_joint_4', '7dof_RISE_joint_5',
  '7dof_RISE_joint_6', '7dof_RISE_joint_7']

        target_joint_state.position = [1.611972300168166, -1.4578431113963095, -2.1906990503479227, 1.1665289138644317, 0.18415297989159973, -0.546418528531665, 0.7706464255599107]

        plan_req.targetJointState = target_joint_state

    plan_srv = rospy.ServiceProxy('/motion_plan/move_arm', MotionPlan)

    plan_res = plan_srv(plan_req)
    
    if plan_res.planResult == MotionPlanResponse.SUCCESS:
        print 'Motion is calculated'
        move_srv = rospy.ServiceProxy('/sim_interface/set_motion', VrepSetJointTrajectory)
        move_req = VrepSetJointTrajectoryRequest()
        move_req.trajectory = plan_res.jointTrajectory
        move_req.duration = 2.0

        #move_srv(move_req)
