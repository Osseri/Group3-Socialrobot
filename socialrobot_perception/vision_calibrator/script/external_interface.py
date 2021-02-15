#!/usr/bin/env python
import os.path
import copy
import yaml
import rospy

from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_perception_msgs.srv import *
from socialrobot_perception_msgs.msg import *
from obj_msg.msg import *
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection3DArray

import tf
import tf.transformations as tfm
import tf2_ros
import numpy as np 

class PerceptionManager():
   
    def __init__(self):
        self.detected_object = []

        # Subscriber
        rospy.Subscriber('/tracking_result', Detection3DArray, self._callback_bbox_3d)

        # Publisher
        self.pub_objects = rospy.Publisher("/external/objects", Objects, queue_size=10)
        self.pub_marker = rospy.Publisher('/external/objects_marker', Marker, queue_size=10)


    def publish_marker(self, objects):
        for i, obj in enumerate(objects.detected_objects):
            # publish cube marker
            robotMarker = Marker()
            robotMarker.header.frame_id = obj.header.frame_id
            robotMarker.header.stamp    = rospy.get_rostime()
            robotMarker.ns = "object"
            robotMarker.id = i
            robotMarker.type = Marker().CUBE
            robotMarker.action = Marker().ADD
            robotMarker.pose.position = obj.bb3d.center.position
            robotMarker.pose.orientation = obj.bb3d.center.orientation
            robotMarker.scale.x = obj.bb3d.size.x
            robotMarker.scale.y = obj.bb3d.size.y
            robotMarker.scale.z = obj.bb3d.size.z
            robotMarker.color.r = 0.0
            robotMarker.color.g = 1.0
            robotMarker.color.b = 0.0
            robotMarker.color.a = 1.0

            robotMarker.lifetime = rospy.Duration(1)
            self.pub_marker.publish(robotMarker)    	

            # publish text marker
            textMarker = Marker()
            textMarker.header.frame_id = obj.header.frame_id
            textMarker.ns = ""
            textMarker.id = i
            textMarker.text = obj.name.data
            textMarker.type = Marker().TEXT_VIEW_FACING
            textMarker.action = Marker().ADD
            textMarker.pose.position = obj.bb3d.center.position
            textMarker.pose.orientation = obj.bb3d.center.orientation
            textMarker.scale.z = 0.1
            textMarker.color.r = 1.0
            textMarker.color.g = 1.0
            textMarker.color.b = 1.0
            textMarker.color.a = 1.0
            textMarker.lifetime = rospy.Duration(10)
            self.pub_marker.publish(textMarker)   


    def _callback_bbox_3d(self, data):
        self.detected_object = []
        for d3d in data.detections:
            #d3d.bbox.size.z += 0.04
            #d3d.bbox.center.position.z += 0.01 + 0.02
            self.detected_object.append(d3d)   

        # add static object(table)
        # d3d = Detection3D()   
        # d3d.tracking_id = int(0)
        # d3d.bbox.center.position.x = 0.550006
        # d3d.bbox.center.position.y = 0
        # d3d.bbox.center.position.z = 0.4 + 0.01
        # d3d.bbox.center.orientation.x = 0
        # d3d.bbox.center.orientation.y = 0
        # d3d.bbox.center.orientation.z = 0
        # d3d.bbox.center.orientation.w = 1    
        # d3d.bbox.size.x = 0.7088739275932312
        # d3d.bbox.size.y = 1.2642161893844604
        # d3d.bbox.size.z = 0.80   
        # self.detected_object.append(d3d)

    def convert_msg(self, d3d): 
        obj = Object()
        obj.header.stamp = rospy.Time.now()
        obj.header.frame_id = 'base_footprint'
        obj.name.data = d3d.tracking_id
        obj.bb3d.center.position = d3d.bbox.center.position
        obj.bb3d.center.orientation = d3d.bbox.center.orientation
        obj.bb3d.size = d3d.bbox.size
        return obj

    def update(self):  
        # convert message format
        objs = Objects()
        
        for d3d in self.detected_object:
            objs.detected_objects.append(self.convert_msg(d3d))
        self.publish_data(objs) 


    def publish_data(self, objects):
        self.publish_marker(objects)
        self.pub_objects.publish(objects)
        
##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('external_perception')        

    # perception manager
    pm = PerceptionManager()

    # Start
    rospy.loginfo('[PerceptionInterface] Started!')


    loop_freq = 10 # 10hz
    r = rospy.Rate(loop_freq)
    while not rospy.is_shutdown():
        pm.update()
        r.sleep
