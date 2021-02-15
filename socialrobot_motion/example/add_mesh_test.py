#!/usr/bin/env python

import moveit_commander
import geometry_msgs
import rospy
import rospkg
import sys



if __name__ == '__main__':
    rospy.init_node('attach_object_async', anonymous=True)
    
    scene = moveit_commander.PlanningSceneInterface()

    # get path
    rospack = rospkg.RosPack()
    rospack.list() 
    dir_path = rospack.get_path('socialrobot_motion') + "/mesh/moveit/"
    obstacle_name = 'obj_fridge'
    obstacle_dir = dir_path + obstacle_name + ".stl"

    # set objects params
    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.header.frame_id = '/map'
    object_pose.pose.position.x = +8.6500e-01
    object_pose.pose.position.y = -2.6400e-01
    object_pose.pose.position.z = +6.7543e-02

    object_pose.pose.orientation.x = 0.0
    object_pose.pose.orientation.y = 0.0
    object_pose.pose.orientation.z = 1.0
    object_pose.pose.orientation.w = 0.0
    scale = [1,1,1]
    
    scene.add_mesh(obstacle_name, object_pose, obstacle_dir, size=scale)

    #
    obstacle_name = 'obj_juice'
    obstacle_dir = dir_path + obstacle_name + ".stl"

    # set objects params
    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.header.frame_id = '/map'
    object_pose.pose.position.x = +7.7500e-01
    object_pose.pose.position.y = -3.9000e-01
    object_pose.pose.position.z = +7.7801e-01

    object_pose.pose.orientation.x = 0.0
    object_pose.pose.orientation.y = 0.0
    object_pose.pose.orientation.z = 1.0
    object_pose.pose.orientation.w = 0.0
    scale = [1,1,1]
    
    scene.add_mesh(obstacle_name, object_pose, obstacle_dir, size=scale)