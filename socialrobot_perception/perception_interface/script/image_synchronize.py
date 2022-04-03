#!/usr/bin/env python
import os.path
import copy
import yaml
import rospy
import rospkg

from sensor_msgs.msg import *

class Synchronizer():

    def __init__(self):
       
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self._callback_depth)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self._callback_depth_info)
        rospy.Subscriber('/camera/color/image_rect_color/republish', Image, self._callback_rgb)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self._callback_rgb_info)

        # Publisher
        self.pub_depth = rospy.Publisher("/camera/test/depth", Image, queue_size=10)
        self.pub_depth_info = rospy.Publisher('/camera/test/depth_info', CameraInfo, queue_size=10)
        self.pub_rgb = rospy.Publisher("/camera/test/rgb", Image, queue_size=10)
        self.pub_rgb_info = rospy.Publisher('/camera/test/rgb_info', CameraInfo, queue_size=10)

        self.depth = None
        self.rgb = None
        self.depth_info = None
        self.rgb_info = None

    def _callback_depth(self, data):
        self.depth = data

    def _callback_depth_info(self, data):
        self.depth_info = data

    def _callback_rgb(self, data):
        self.rgb = data

    def _callback_rgb_info(self, data):
        self.rgb_info = data

    def update(self):
        
        if self.depth != None and self.rgb != None and self.depth_info != None and self.rgb_info != None:
            self.depth.header.stamp = rospy.Time().now()
            self.depth_info.header.stamp = self.depth.header.stamp
            self.rgb.header.stamp = self.depth.header.stamp
            self.rgb_info.header.stamp = self.depth.header.stamp

            self.pub_depth.publish(self.depth)
            self.pub_depth_info.publish(self.depth_info)
            self.pub_rgb.publish(self.rgb)
            self.pub_rgb_info.publish(self.rgb_info)

##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('image_synchronizer')        

    # perception manager
    sync = Synchronizer()

    # Start
    rospy.loginfo('[image_synchronizer] Started!')
    
    loop_freq = 20 # 20hz
    r = rospy.Rate(loop_freq)
    while not rospy.is_shutdown():
        sync.update()
        r.sleep
