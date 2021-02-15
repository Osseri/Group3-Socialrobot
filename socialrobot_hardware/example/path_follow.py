#!/usr/bin/env python
import rospy
import rosparam

from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_hardware.srv import *

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('path_following_test')

    # build path
    path = Path()
    path.header.frame_id = 'base_footprint'

    for i in range(100):
        pose = PoseStamped()
        pose.header.frame_id='base_footprint'
        pose.pose.position.x = i*-0.02
        pose.pose.position.y = i*0.02
        pose.pose.position.z = 0

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1

        path.poses.append(pose)
    
    # request
    behavior_srv = rospy.ServiceProxy('/set_path', SetPathTrajectory)
    behavior_req = SetPathTrajectoryRequest()
    behavior_req.trajectory = path
    
    behavior_res = behavior_srv(behavior_req)