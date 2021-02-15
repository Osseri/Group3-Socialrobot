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

        self.pub_objects = rospy.Publisher("/perception/objects", Objects, queue_size=10)

    def publish(self):
        # red_gotica
        obs1 = BoundingBox3D()
        c1 = Pose()
        c1.position.x = +3.0000e-01
        c1.position.y = +9.9997e-02
        c1.position.z = +8.2886e-01
        c1.orientation.x = 1.31936e-05
        c1.orientation.y = 2.20794e-10
        c1.orientation.z = 6.07222e-07
        c1.orientation.w = 1
        obs1.center = c1
        v1 = Vector3()
        v1.x = 0.0618015 
        v1.y = 0.059508 
        v1.z = 0.23814
        obs1.size = v1

        # gotica
        obs2 = BoundingBox3D()
        c2 = Pose()
        c2.position.x = +4.0000e-01
        c2.position.y = -1.5003e-02
        c2.position.z = +8.2886e-01
        c2.orientation.x = 1.31627e-05 
        c2.orientation.y = 2.26816e-10
        c2.orientation.z = -1.15535e-18   
        c2.orientation.w = 1.0              
        obs2.center = c2
        v2 = Vector3()
        v2.x = 0.065 
        v2.y = 0.065 
        v2.z = 0.23544
        obs2.size = v2

        # bakey
        obs3 = BoundingBox3D()
        c3 = Pose()
        c3.position.x = +3.0000e-01
        c3.position.y = -9.9997e-02
        c3.position.z = +8.2886e-01
        c3.orientation.x = 1.31936e-05
        c3.orientation.y = 2.20794e-10
        c3.orientation.z = 6.07222e-07
        c3.orientation.w = 1
        obs3.center = c3
        v3 = Vector3()
        v3.x = 0.0618015 
        v3.y = 0.059508 
        v3.z = 0.23814
        obs3.size = v3
        
        # table
        obs4 = BoundingBox3D()
        c4 = Pose()
        c4.position.x = 0.550006
        c4.position.y = 8.80659e-06
        c4.position.z = 0.365011
        c4.orientation.x = 0
        c4.orientation.y = 0
        c4.orientation.z = 0.707
        c4.orientation.w = 0.707
        obs4.center = c4
        v4 = Vector3()
        v4.x = 1.1342161893844604
        v4.y = 0.7088739275932312
        v4.z = 0.6899999976158142
        obs4.size = v4

        detected_objects=[obs1,obs2,obs3,obs4]
        object_ids = ['obj_red_gotica','obj_gotica','obj_bakey','obj_table']

        objs = Objects()
        for i,bb3d in enumerate(detected_objects):
            obj = Object()
            obj.header.stamp = rospy.Time.now()
            obj.header.frame_id = '/base_footprint'
            obj.name.data = object_ids[i]
            obj.bb3d = bb3d     
            objs.detected_objects.append(obj)   
        self.pub_objects.publish(objs)

    def update(self): 
        self.publish()   


##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('fake_perception')        

    # perception manager
    pm = PerceptionManager()

    # Start
    rospy.loginfo('[PerceptionManager] Fake publisher Started!')


    loop_freq = 10 # 10hz
    r = rospy.Rate(loop_freq)
    while not rospy.is_shutdown():
        pm.update()
        r.sleep()
