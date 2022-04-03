#!/usr/bin/env python
import rospy
import rosparam

from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_hardware.srv import *
from tf.transformations import quaternion_from_euler


##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('path_following_test')

    # build path
    path = Path()
    path.header.frame_id = 'base_footprint'

    path_length = 100
    # goal distance
    x = -0.4
    y = 0.4
    theta = 0.0

    for i in range(path_length):
        pose = PoseStamped()
        pose.header.frame_id='base_footprint'
        pose.pose.position.x = x/path_length*i
        pose.pose.position.y = y/path_length*i
        pose.pose.position.z = 0

        # 0.9 deg = 0.015708 rad
        deg = theta / path_length
        rad = deg /180 * 3.14159 * i
        q = quaternion_from_euler(0, 0, rad)

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        path.poses.append(pose)
        
    # request
    behavior_srv = rospy.ServiceProxy('/set_path', SetPathTrajectory)
    behavior_req = SetPathTrajectoryRequest()
    behavior_req.trajectory = path    
    behavior_res = behavior_srv(behavior_req)