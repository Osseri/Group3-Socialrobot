#!/usr/bin/env python
import arm_planner
import grasp_planner
#import single_arm_planner
from socialrobot_behavior.srv import *
from socialrobot_hardware.srv import *
from socialrobot_motion.srv import *
from socialrobot_msgs.srv import *
from std_msgs.msg import *
import std_srvs.srv as std_srv
from geometry_msgs.msg import *
from vision_msgs.msg import *
from sensor_msgs.msg import *
from trajectory_msgs.msg import *
from actionlib_msgs.msg import GoalStatusArray
from math import pi
import moveit_commander
import rosparam
import rospy
import roslib
roslib.load_manifest('socialrobot_motion')

def callback_moveit_status(moveit_status):
    rosparam.set_param("/moveit_status", "True")
            
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
    moveit_start = False
    while not moveit_start:
        if rospy.has_param('/moveit_status'):
            moveit_start = rosparam.get_param("/moveit_status")
        rospy.sleep(0.5)	
    
    # get rosparam /robot_name
    robot_name = 'social_robot'
    if rospy.has_param('robot_name'):
        robot_name = rospy.get_param('robot_name')

    # get joint information
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Arm Planner
    ap = arm_planner.ArmPlanner(commander=moveit_commander,
                                robot=robot,
                                scene=scene)
    rospy.Service('/motion_plan/move_arm',
                  MotionPlan,
                  ap.callback_arm_plan)
    rospy.Service('/motion_plan/reset',
                  std_srv.Empty,
                  ap.callback_reset)    
    rospy.Service('/motion_plan/get_scene_objects',
                  socialrobot_msgs.srv.GetObjects,
                  ap.callback_get_scene_objects)  
    rospy.Service('/motion_plan/update_scene_objects',
                  socialrobot_msgs.srv.UpdateObjects,
                  ap.callback_update_scene_objects)  
    rospy.Service('/motion_plan/get_group_info',
                  socialrobot_msgs.srv.GetGroups,
                  ap.callback_get_group_info)  
                  
    # Grasp Planner
    gp = grasp_planner.GraspPlanner(robot_name=robot_name,
                                    robot=robot)
    rospy.Service('/motion_plan/grasp_plan',
                  MotionPlan,
                  gp.callback_plan_grasp)

    #sap = single_arm_planner.SingleArmPlanner()
    #rospy.Service('/motion_plan/single_arm',
    #            MotionPlan,
    #            sap.callback_arm_plan)

    # Start
    rospy.loginfo('[Socialrobot_motion] Service Started!')
    rospy.spin()
