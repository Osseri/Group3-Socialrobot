#!/usr/bin/env python
import os.path
import copy
import yaml
import rospy
import rospkg

from sensor_msgs.msg import *

class Synchronizer():

    def __init__(self):
       
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self._callback_image)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self._callback_info)

        # Publisher
        self.pub_image = rospy.Publisher("/camera/test/depth", Image, queue_size=10)
        self.pub_info = rospy.Publisher('/camera/test/camera_info', CameraInfo, queue_size=10)

        self.image = None
        self.info = None

    def _callback_image(self, data):
        self.image = data

        if self.info != None:
            self.image.header.stamp = rospy.Time().now()
            self.info.header.stamp = self.image.header.stamp

            self.pub_image.publish(self.image)
            self.pub_info.publish(self.info)

    def _callback_info(self, data):
        self.info = data

    def update(self):
        pass

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
    rospy.spin()

    # loop_freq = 1 # 10hz
    # r = rospy.Rate(loop_freq)
    # while not rospy.is_shutdown():
    #     sync.update()
    #     r.sleep
