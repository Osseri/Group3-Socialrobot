#!/usr/bin/env python
import arm_planner
import grasp_planner
from socialrobot_behavior.srv import *
from socialrobot_hardware.srv import *
from socialrobot_motion.srv import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from vision_msgs.msg import *
from sensor_msgs.msg import *
from trajectory_msgs.msg import *
from actionlib_msgs.msg import GoalStatusArray
from math import pi
import moveit_commander
import rosparam
import rospy
import signal
import sys
import os.path
import os
import roslib
roslib.load_manifest('socialrobot_motion')

moveit_start = False
def callback_moveit_status(moveit_status):
    global moveit_start
    if not moveit_start:
        rosparam.set_param("/moveit_status", "True")
        moveit_start = True
            
##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('moveit_planner')    
    rospy.loginfo("Waiting moveit initialization...")
    sub_moveit = rospy.Subscriber("/move_group/status", GoalStatusArray, callback_moveit_status)

    # wait for moveit planner
    rosparam.set_param("/moveit_status", "False")
    moveit_start = rosparam.get_param("/moveit_status")
    while not moveit_start:
        moveit_start = rosparam.get_param("/moveit_status")
        rospy.sleep(2.0)	
    
    # get rosparam /robot_name
    robot_name = 'skkurobot'
    if rospy.has_param('robot_name'):
        robot_name = rospy.get_param('robot_name')


    # get joint information
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Arm Planner
    ap = arm_planner.ArmPlanner(robot=robot,
                                scene=scene)
    rospy.Service('/motion_plan/move_arm',
                  MotionPlan,
                  ap.callback_arm_plan)

    # Grasp Planner
    gp = grasp_planner.GraspPlanner(robot_name=robot_name,
                                    robot=robot)
    rospy.Service('/motion_plan/grasp_plan',
                  MotionPlan,
                  gp.callback_plan_grasp)

    # rospy.Service('/motion_plan/gripper_open_close',
    #             MotionPlan,
    #             gp.callback_open_close)

    # Start
    rospy.loginfo('[Socialrobot_motion] Service Started!')
    rospy.spin()
