#!/usr/bin/env python
import geometry_msgs
import rospy
import sys
from socialrobot_perception_msgs.srv import *

class GetObjectsTest:

    def __init__(self):

        rospy.loginfo("wait for server.")
        self.srv = rospy.ServiceProxy("/motion_plan/get_scene_objects", GetObjects)
    
    def request(self):

        req = GetObjectsRequest()
        res = self.srv(req)
        print(res)


if __name__ == '__main__':
    rospy.init_node('example', anonymous=True)
    client = GetObjectsTest()

    client.request()

