#!/usr/bin/env python

import moveit_commander
import geometry_msgs
import rospy
import sys
import rospkg


if __name__ == '__main__':
    rospy.init_node('attach_object_async', anonymous=True)
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)

    # get path
    rospack = rospkg.RosPack()
    rospack.list() 
    dir_path = rospack.get_path('socialrobot_motion') + "/mesh/moveit/"
    obstacle_name = 'obj_fridge'
    obstacle_dir = dir_path + obstacle_name + ".stl"

    # set objects params
    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.header.frame_id = '/base_footprint'
    object_pose.pose.position.x = 1.0
    object_pose.pose.position.y = 0.0
    object_pose.pose.position.z = 0.0

    object_pose.pose.orientation.x = 0.0
    object_pose.pose.orientation.y = 0.0
    object_pose.pose.orientation.z = 0.0
    object_pose.pose.orientation.w = 1.0
    scale = [1,1,1]
    
    scene.add_mesh(obstacle_name, object_pose, obstacle_dir, size=scale)
    #scene.add_box(obstacle_name, object_pose, size=scale)
    #scene.remove_world_object(obstacle_name)