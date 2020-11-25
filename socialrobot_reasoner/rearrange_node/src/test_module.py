#!/usr/bin/env python

import rospy
from rearrange_node.srv._rearrange_env_srv import *
from rearrange_node.msg._env_object_info_msg import *

def example():
    rospy.wait_for_service('rearrange_srv')
    try:
        f_check_srv = rospy.ServiceProxy('rearrange_srv', rearrange_env_srv)
        pub_msg = rearrange_env_srvRequest()

        pub_msg.target.object_name = ['target']
        pub_msg.target.object_position.x, pub_msg.target.object_position.y, pub_msg.target.object_position.z = 0.9, 0.05, 1.0 + 0.1
        pub_msg.target.object_orientation.x, pub_msg.target.object_orientation.y, pub_msg.target.object_orientation.z, pub_msg.target.object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.target.object_scale.x, pub_msg.target.object_scale.y, pub_msg.target.object_scale.z = 0.06, 0.06, 0.2

        for i in range(3):
            obs_tmp=env_object_info_msg()
            pub_msg.objects.append(obs_tmp)

        pub_msg.objects[0].object_name = ['obj1']
        pub_msg.objects[0].object_position.x, pub_msg.objects[0].object_position.y, pub_msg.objects[0].object_position.z = 0.75, 0.1, 1.0 + 0.1
        pub_msg.objects[0].object_orientation.x, pub_msg.objects[0].object_orientation.y, pub_msg.objects[0].object_orientation.z, pub_msg.objects[0].object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.objects[0].object_scale.x, pub_msg.objects[0].object_scale.y, pub_msg.objects[0].object_scale.z = 0.06, 0.06, 0.2

        pub_msg.objects[1].object_name = ['obj2']
        pub_msg.objects[1].object_position.x, pub_msg.objects[1].object_position.y, pub_msg.objects[1].object_position.z = 0.75, 0.0, 1.0 + 0.1
        pub_msg.objects[1].object_orientation.x, pub_msg.objects[1].object_orientation.y, pub_msg.objects[1].object_orientation.z, pub_msg.objects[1].object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.objects[1].object_scale.x, pub_msg.objects[1].object_scale.y, pub_msg.objects[1].object_scale.z = 0.06, 0.06, 0.2

        pub_msg.objects[2].object_name = ['obj3']
        pub_msg.objects[2].object_position.x, pub_msg.objects[2].object_position.y, pub_msg.objects[2].object_position.z = 0.75, -0.1, 1.0 + 0.1
        pub_msg.objects[2].object_orientation.x, pub_msg.objects[2].object_orientation.y, pub_msg.objects[2].object_orientation.z, pub_msg.objects[2].object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.objects[2].object_scale.x, pub_msg.objects[2].object_scale.y, pub_msg.objects[2].object_scale.z = 0.06, 0.06, 0.2

        pub_msg.workspace.object_name = ['table']
        pub_msg.workspace.object_position.x, pub_msg.workspace.object_position.y, pub_msg.workspace.object_position.z = 0.7, 0.0, 0.5
        pub_msg.workspace.object_orientation.x, pub_msg.workspace.object_orientation.y, pub_msg.workspace.object_orientation.z, pub_msg.workspace.object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.workspace.object_scale.x, pub_msg.workspace.object_scale.y, pub_msg.workspace.object_scale.z = 0.5, 0.8, 1.0

        resp1 = f_check_srv(pub_msg)
        # f_ori[0]
        print resp1

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    print "Example"
    example()
