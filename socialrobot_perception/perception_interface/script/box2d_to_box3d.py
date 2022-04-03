#!/usr/bin/env python
import os.path
import copy
import yaml
import rospy
import rospkg

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from PIL import Image as img
from StringIO import StringIO
from sensor_msgs.msg import *
from vision_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray

class Transformer():

    def __init__(self):
       
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self._callback_depth)
        rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self._callback_depth_info)
        rospy.Subscriber('/hand_detection/detection2D', Detection2DArray, self._callback_detection2d)

        # Publisher
        self.pub_marker = rospy.Publisher('/hand_detection/marker', Marker, queue_size=10)

        self.camera_frame = '/camera_color_optical_frame'
        self.model = PinholeCameraModel()
        self.bridge = CvBridge()
        self.depth = None
        self.depth_info = None
        self.detection2d = None
        self.need_depth_info = True
        self.need_depth = True

        self.update()

    def _callback_depth(self, depth_image):
        """ Get depth at chosen pixel using depth image """
        depth = self.bridge.imgmsg_to_cv2(depth_image , "32FC1")
        self.depth = np.asarray(depth)
        self.need_depth = False

    def _callback_depth_info(self, info):
        """ Define Pinhole Camera Model parameters using camera info msg """
        if self.need_depth_info:
            rospy.loginfo('Got depth info!')
            self.model.fromCameraInfo(info)  # define model params
            self.frame = info.header.frame_id
            self.need_depth_info = False

    def _callback_detection2d(self, data):
        self.detection2d = data

    def creat_point_marker(self, vec, frame_id='/map'):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = vec[0]
        marker.pose.position.y = vec[1]
        marker.pose.position.z = vec[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        return marker

    def update(self):
        loop_freq = 10 # 10hz
        r = rospy.Rate(loop_freq)                  
    
        while not rospy.is_shutdown():
            if self.detection2d != None and not self.need_depth_info and not self.need_depth:
                for detection in self.detection2d.detections:
                    bb2d = detection.bbox
                    bb2d_x = int(bb2d.center.x)
                    bb2d_y = int(bb2d.center.y)
                    
                    c=[bb2d_x, bb2d_y]

                    pt = list(self.model.projectPixelTo3dRay((c[0], c[1])))
                    pt[:] = [x/pt[2] for x in pt]

                    # depth image is noisy - let's make mean of few pixels
                    da = []
                    for x in range(int(c[0]) - 2, int(c[0]) + 3):
                        for y in range(int(c[1]) - 2, int(c[1]) + 3):
                            da.append(self.depth[y, x]/1000.0)

                    d = np.mean(da)
                    pt[:] = [x*d for x in pt]


                    self.pub_marker.publish(self.creat_point_marker(pt, frame_id=self.camera_frame))

            self.detection2d = None
            r.sleep


##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('bbox2d_to_bbox3d')        

    # perception manager
    transformer = Transformer()

    # Start
    rospy.loginfo('[bbox2d_to_bbox3d] Started!')
    rospy.spin()


