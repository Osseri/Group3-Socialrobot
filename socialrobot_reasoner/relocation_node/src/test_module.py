#!/usr/bin/env python

import rospy
from relocation_node.srv._relocate_env_srv import *

def feasible_check_client():
    rospy.wait_for_service('relocation_srv')
    try:
        f_check_srv = rospy.ServiceProxy('relocation_srv', relocate_env_srv)
        pub_msg = relocate_env_srvRequest()

        pub_msg.robot_height = 0.075
        pub_msg.robot_pose = [0.32299999999999995, -0.06575]
        pub_msg.target_id = 8

        pub_msg.N = 10
        pub_msg.R = [0.025, 0.026, 0.03, 0.028, 0.026, 0.025, 0.03, 0.03, 0.029, 0.025, 0.03]
        print pub_msg.R, type(pub_msg.R)
        pub_msg.H = [0.073, 0.071, 0.068, 0.07, 0.071, 0.069, 0.07, 0.073, 0.066, 0.066, 0.075]
        pub_msg.X = [0.475, 0.306, 0.475, 0.36405000000000004, 0.306, 0.17099999999999999, 0.43005000000000004, 0.20405, 0.20900000000000002, 0.17099999999999999]
        pub_msg.Y = [0.19475, 0.14075000000000001, 0.36975, 0.26625, 0.40975000000000006, 0.16075, 0.27425, 0.28725, 0.04225, 0.38375000000000004]

        pub_msg.x_min = 0.06299999999999999
        pub_msg.x_max = 0.583
        pub_msg.y_min = -0.06575
        pub_msg.y_max = 0.51775

        resp1 = f_check_srv(pub_msg)
        # f_ori[0]
        print resp1

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    print "Requesting for feasibility check in moveIT"
    feasible_check_client()
