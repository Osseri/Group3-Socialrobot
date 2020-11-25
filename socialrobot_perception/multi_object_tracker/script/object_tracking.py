#!/usr/bin/env python
import os
import sys


from utils.timer import Timer

import numpy as np
import os
import cv2


### ROS STUFF
import rospy 
import math
import copy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import *
from vision_msgs.msg import *
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
from affnet.msg import *

from tracker import Tracker

class Tracking_box:
    def __init__(self):
                        
        #subscriber
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/camera/aligned_depth/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/obj_aff", aff_msgs, self.detection_result_callback)

        self.detected_objects = []
        self.tracked_objects = []

        self.bridge = CvBridge()
        self.isImage = False
        self.isDetection = False
        self.min_prop = 0.96
        self.depth_image = None
        self.rgb_image = None
        self.viz_image = None
        self.tracker = Tracker()

    def start(self):
        return

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")  
        except CvBridgeError as e:
            print(e)
        self.isImage = True

    def rgb_callback(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")            
        except CvBridgeError as e:
            print(e)
        self.isImage = True

    def update(self):
        if self.isImage:
            centers = []
            bboxes = []
            if self.detected_objects:
                for obj in self.detected_objects.detections:
                    corner = self.getBoundingBoxCorner(obj.bbox)
                    bbox = (corner[0], corner[1], corner[2]-corner[0], corner[3]-corner[1])
                    bboxes.append(bbox)   

                    center = (obj.bbox.center.x, obj.bbox.center.y)
                    centers.append(center) 

            self.tracked_objects = self.tracker.update(centers, bboxes,self.rgb_image)

        return

    def getBoundingBoxCorner(self, boundingBox2D):
        bb = boundingBox2D
        center_x = bb.center.x
        center_y = bb.center.y
        size_x = bb.size_x
        size_y = bb.size_y

        lt_x = int((center_x - size_x/2))
        lt_y = int((center_y - size_y/2))
        rb_x = int(round(center_x + size_x/2))
        rb_y = int(round(center_y + size_y/2))
        corner = (lt_x, lt_y, rb_x, rb_y)
        return corner

    def detection_result_callback(self, data): 
        objects = Detection2DArray()

        # if detected object exit
        if data.bb_elements:
            for i, obj in enumerate(data.bb_elements):
                if obj.prop > self.min_prop:  # if reliable
                    detected_object = Detection2D()
                    bb = BoundingBox2D()
                    cf = ObjectHypothesisWithPose()

                    bb.center.x = int(round((obj.bb_lt_p + obj.bb_rb_p)/2))
                    bb.center.y = int(round((obj.bb_lt_q + obj.bb_rb_q)/2))
                    bb.size_x = obj.bb_rb_p - obj.bb_lt_p 
                    bb.size_y = obj.bb_rb_q - obj.bb_lt_q 

                    cf.id = obj.id
                    cf.score = obj.prop

                    detected_object.bbox = bb
                    detected_object.results.append(cf)
                    objects.detections.append(detected_object)
            self.detected_objects = objects

        # no detected objects
        else:            
            self.detected_objects = []
        self.isDetection = True
     
    def show_results(self, timer):
        if self.isImage:
            # copy image
            vis_img = self.rgb_image.copy()

            # Calculate Frames per second (FPS)
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

            # show detected object   
            if self.detected_objects:
                for i, obj in enumerate(self.detected_objects.detections):
                    bbox = self.getBoundingBoxCorner(obj.bbox)
                    cv2.rectangle(vis_img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0,0,0), 2)
                    cv2.circle(vis_img, (obj.bbox.center.x, obj.bbox.center.y), 3, (0,0,0), -1)
            #cv2.putText(vis_img, "FPS : " + str(fps), (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

            # show tracked object
            track_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
                    (0, 255, 255), (255, 0, 255), (255, 127, 255),
                    (127, 0, 255), (127, 0, 127)]
            if self.tracked_objects:
                for i, track in enumerate(self.tracked_objects):
                    bbox = [int(j) for j in track.bbox]
                    clr = track.track_id % len(track_colors)
                    cv2.rectangle(vis_img, (bbox[0], bbox[1]), (bbox[0]+bbox[2], bbox[1]+bbox[3]), track_colors[clr], 2)
                    cv2.circle(vis_img, (int(track.center[0]), int(track.center[1])), 1, track_colors[clr], -1)

                    if track.status == "DETECTING":
                        cv2.putText(vis_img, "ID"+str(track.track_id), (bbox[0], bbox[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
                    elif track.status == "TRACKING":
                        cv2.putText(vis_img, "ID"+str(track.track_id), (bbox[0], bbox[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                    if (len(track.trace) > 1):
                        for j in range(len(track.trace)-1):
                            # Draw trace line
                            x1 = track.trace[j][0]
                            y1 = track.trace[j][1]
                            x2 = track.trace[j+1][0]
                            y2 = track.trace[j+1][1]
                            cv2.line(vis_img, (int(x1), int(y1)), (int(x2), int(y2)),track_colors[clr], 2)

            cv2.imshow("original", vis_img)
            cv2.waitKey(1)


def main(args):
    rospy.init_node('Track_Node')    
    
    tb=Tracking_box()    

    while not rospy.is_shutdown():
        # Start timer
        timer = cv2.getTickCount()

        # Update tracker
        tb.update()

        # Show results
        tb.show_results(timer)
                 
if __name__ == '__main__':
    main(sys.argv)
