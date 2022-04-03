#!/usr/bin/env python
import geometry_msgs
import rospy
import sys
from socialrobot_msgs.srv import *
from socialrobot_msgs.msg import Object

class UpdateObjectsTest:

    def __init__(self):

        rospy.loginfo("wait for server.")
        self.srv = rospy.ServiceProxy("/motion_plan/update_scene_objects", UpdateObjects)
    
    def request(self):

        req = UpdateObjectsRequest()
        req.command = UpdateObjectsRequest.REMOVE
        req.object_ids = ['a','b','c']
        
        res = self.srv(req)
        print(res)

    def add_object(self):
        req = UpdateObjectsRequest()
        req.command = UpdateObjectsRequest.ADD
        obj = Object()
        obj.id = 'test'
        obj.bb3d.center.position.x = 1.0
        obj.bb3d.center.position.y = 0.0
        obj.bb3d.center.position.z = 0.0
        obj.bb3d.center.orientation.x = 0.0
        obj.bb3d.center.orientation.y = 0.0
        obj.bb3d.center.orientation.z = 0.0
        obj.bb3d.center.orientation.w = 1.0
        obj.bb3d.size.x = 1.0
        obj.bb3d.size.y = 1.0
        obj.bb3d.size.z = 1.0

        req.objects.append(obj)
        
        res = self.srv(req)

if __name__ == '__main__':
    rospy.init_node('example', anonymous=True)
    client = UpdateObjectsTest()

    client.add_object()

