#!/usr/bin/env python
import rospy
import rosparam

from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
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

    path_pub = rospy.Publisher('/mobile_path', Path, queue_size=10)
    pose_pub = rospy.Publisher('/mobile_pose', PoseArray, queue_size=10)

    plan_req = GetMotionRequest()

    plan_req.planner_name = "opendoor"

    # robot type
    plan_req.inputs.targetBody = plan_req.inputs.MOBILE_BASE

    # options
    plan_req.inputs.door_info = [0.529964, -0.06500, 0.982926, 0.5, -0.5, -0.5, 0.5]
    plan_req.inputs.desired_door_angle = 120

    # get path
    motion_srv = rospy.ServiceProxy('/behavior/get_motion', GetMotion)
    res = motion_srv(plan_req)
     

    # set behavior
    if res.result:
        behavior_srv = rospy.ServiceProxy('/behavior/set_behavior', SetBehavior)
        behavior_req = SetBehaviorRequest()
        behavior_req.header.frame_id = plan_req.planner_name
        behavior_req.path = res.motion.pathTrajectory
        behavior_res = behavior_srv(behavior_req)