#!/usr/bin/env python

import rospy
from rearrange_node.srv._rearrange_env_srv import *
from rearrange_node.msg._env_object_info_msg import *

def example():
    rospy.wait_for_service('rearrange_srv')
    try:
        f_check_srv = rospy.ServiceProxy('rearrange_srv', rearrange_env_srv)
        pub_msg = rearrange_env_srvRequest()
        pub_msg.robot_pose = [0.0, 0.6245] # left eef x-y position

        pub_msg.target.object_name = ['milk']
        pub_msg.target.object_position = (+3.0000e-01, +9.9997e-02, +8.2750e-01)
        pub_msg.target.object_orientation = (0.0, 0.0, 0.0, 1.0)
        pub_msg.target.object_scale = (6.5000e-02, 6.5000e-02, 2.3544e-01)

        obs_tmp=env_object_info_msg()
        pub_msg.objects.append(obs_tmp)

        pub_msg.objects[0].object_name = ['juice']
        pub_msg.objects[0].object_position.x, pub_msg.objects[0].object_position.y, pub_msg.objects[0].object_position.z = 3.0000e-01, +1.6500e-01, +8.2750e-01
        pub_msg.objects[0].object_orientation.x, pub_msg.objects[0].object_orientation.y, pub_msg.objects[0].object_orientation.z, pub_msg.objects[0].object_orientation.w = 0.0, 0.0, 0.0, 1.0
        pub_msg.objects[0].object_scale.x, pub_msg.objects[0].object_scale.y, pub_msg.objects[0].object_scale.z = 6.5000e-02, 6.5000e-02, 2.3544e-01

        pub_msg.workspace.object_name = ['table']
        pub_msg.workspace.object_position.x, pub_msg.workspace.object_position.y, pub_msg.workspace.object_position.z = 5.5001e-01, +8.8066e-06, +3.6501e-01
        pub_msg.workspace.object_orientation.x, pub_msg.workspace.object_orientation.y, pub_msg.workspace.object_orientation.z, pub_msg.workspace.object_orientation.w = 0.0, 0.0, 0.0, 1.0
        pub_msg.workspace.object_scale.x, pub_msg.workspace.object_scale.y, pub_msg.workspace.object_scale.z = 7.0887e-01, 1.1342e+00, 6.9000e-01

        resp1 = f_check_srv(pub_msg)
        # f_ori[0]
        print resp1

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    print "Example"
    example()
