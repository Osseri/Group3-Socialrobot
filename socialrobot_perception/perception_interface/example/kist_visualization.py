#!/usr/bin/env python
import os.path
import copy
import yaml
import rospy
import rospkg

from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_msgs.srv import *
from socialrobot_msgs.msg import *
from socialrobot_hardware.msg import *
from obj_msg.msg import *
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray

import tf
import tf.transformations as tfm
import tf2_ros
import numpy as np 

class PerceptionManager():
    CONFIG_PATH = rospkg.RosPack().get_path('perception_interface') + '/config/'
    def __init__(self):
        self.dataset = []
        self.detected_object = []

        # Subscriber
        rospy.Subscriber('/bbox_3d', Detection3DArray, self._callback_bbox_3d)

        # Publisher
        self.pub_objects = rospy.Publisher("/socialrobot/perception/objects", Objects, queue_size=10)
        self.pub_marker = rospy.Publisher('/socialrobot/visualization/objects', MarkerArray, queue_size=10)

         # read configs
        self.read_dataset()

    def read_dataset(self):
        fname = self.CONFIG_PATH + 'dataset.txt'
        f = open(fname, "r")
        data = f.readlines()
        
        for x in data:
            self.dataset.append(x[:-1])     

    def publish_marker(self, objects):
        '''
        socialrobot_msgs/Objects to visialization_msgs/MarkerArray
        '''
        marker_array = MarkerArray()
        for i, obj in enumerate(objects):
            # publish cube marker
            marker = Marker()
            marker.header = obj.header
            marker.ns = "object"
            marker.id = i
            marker.type = Marker().CUBE
            marker.action = Marker().MODIFY
            marker.pose.position = obj.bb3d.center.position
            marker.pose.orientation = obj.bb3d.center.orientation
            marker.scale.x = obj.bb3d.size.x
            marker.scale.y = obj.bb3d.size.y
            marker.scale.z = obj.bb3d.size.z
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.lifetime = rospy.Duration(1)	
            marker_array.markers.append(marker)

            # publish text marker
            textMarker = Marker()
            textMarker.header = obj.header
            textMarker.ns = "text"
            textMarker.id = i
            textMarker.text = obj.id
            textMarker.type = Marker().TEXT_VIEW_FACING
            textMarker.action = Marker().MODIFY
            #textMarker.pose = obj.bb3d.center
            textMarker.pose.position.x = obj.bb3d.center.position.x
            textMarker.pose.position.y = obj.bb3d.center.position.y
            textMarker.pose.position.z = obj.bb3d.center.position.z + obj.bb3d.size.z/2
            textMarker.scale.z = 0.05
            textMarker.color.r = 1.0
            textMarker.color.g = 1.0
            textMarker.color.b = 1.0
            textMarker.color.a = 1.0
            textMarker.lifetime = rospy.Duration(1)
            marker_array.markers.append(textMarker)
        self.pub_marker.publish(marker_array)   

    def _callback_bbox_3d(self, data):
        self.detected_object = []
        for d3d in data.detections:            
            self.detected_object.append(self.convert_msg(d3d))   
        #publish obj msg
        pub_msg = Objects()
        pub_msg.detected_objects = self.detected_object

        #hard-coding ADD: table
        object4 = Object() 
        object4.header.stamp = rospy.Time.now()
        object4.header.frame_id = 'base_footprint'
        object4.id = "obj_table"
        obs4 = BoundingBox3D()
        c4 = Pose()
        c4.position.x = 0.550006
        c4.position.y = 0.0
        c4.position.z = 0.4
        c4.orientation.x = 0
        c4.orientation.y = 0
        c4.orientation.z = 0.707
        c4.orientation.w = 0.707
        obs4.center = c4
        v4 = Vector3()
        v4.x = 1.1342161893844604
        v4.y = 0.7088739275932312
        v4.z = 0.8
        obs4.size = v4
        object4.bb3d = obs4

        pub_msg.detected_objects.append(object4)

        self.pub_objects.publish(pub_msg)
                    
    def convert_msg(self, d3d): 
        obj = Object()
        obj.header.stamp = rospy.Time.now()
        obj.header.frame_id = 'base_footprint'
        obj.id = self.dataset[int(d3d.tracking_id)]
        obj.bb3d = d3d.bbox
        return obj

    def update(self):          
        self.publish_marker(self.detected_object)
        
##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('kist_test')        

    # perception manager
    pm = PerceptionManager()

    loop_freq = 10 # 10hz
    r = rospy.Rate(loop_freq)
    while not rospy.is_shutdown():
        pm.update()
        r.sleep
