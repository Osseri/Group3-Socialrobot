#!/usr/bin/env python
import os
import os.path
import sys
import signal
import rospy
import rosparam

from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_perception_msgs.srv import *
from socialrobot_perception_msgs.msg import *
from socialrobot_hardware.msg import *

class PerceptionManager():
    def __init__(self):

        self.current_obstacles = {}
        self.current_obstacles_base = {}
        rospy.Subscriber('/sim_interface/objects', ObjectInfo, self._callback_objects)
        rospy.Subscriber('/sim_interface/objects_base', ObjectInfo, self._callback_objects_base)
        self.pub_objects = rospy.Publisher("/perception/objects", Objects, queue_size=10)
        #self.pub_objects_base = rospy.Publisher("/perception/objects_base", Objects, queue_size=10)

    def publish(self):
        objs = Objects()
        for name, bb3d in self.current_obstacles.items():
            obj = Object()
            obj.header.stamp = rospy.Time.now()
            obj.header.frame_id = 'map'
            obj.name.data = name
            bb3d.size.x *= 1.0
            bb3d.size.y *= 1.0
            bb3d.size.z *= 1.0
            obj.bb3d = bb3d     
            objs.detected_objects.append(obj)   
        self.pub_objects.publish(objs)

    def publish_base(self):
        objs = Objects()
        for name, bb3d in self.current_obstacles_base.items():
            obj = Object()
            obj.header.stamp = rospy.Time.now()
            obj.header.frame_id = 'base_footprint'
            obj.name.data = name
            bb3d.size.x *= 1.0
            bb3d.size.y *= 1.0
            bb3d.size.z *= 1.0
            obj.bb3d = bb3d     
            objs.detected_objects.append(obj)   
        self.pub_objects.publish(objs)

    def update(self): 
        self.publish_base()   

    def _callback_objects(self, data):
        self.current_obstacles = {}
        for idx, name in enumerate(data.names):
            self.current_obstacles[name] = data.obstacles[idx]

    def _callback_objects_base(self, data):
        self.current_obstacles_base = {}
        for idx, name in enumerate(data.names):
            self.current_obstacles_base[name] = data.obstacles[idx]

##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('perception')        

    # perception manager
    pm = PerceptionManager()

    # Start
    rospy.loginfo('[PerceptionManager] Service Started!')


    loop_freq = 10 # 10hz
    r = rospy.Rate(loop_freq)
    while not rospy.is_shutdown():
        pm.update()
        r.sleep()
