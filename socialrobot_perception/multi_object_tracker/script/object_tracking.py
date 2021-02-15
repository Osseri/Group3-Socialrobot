#!/usr/bin/env python
import os
import sys


# from utils.timer import Timer

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
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import *

from tracker import Tracker
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_from_matrix

###
from std_msgs.msg import *


class Tracking_box:
    def __init__(self):

        self.detected_objects = []
        self.tracked_objects = []

        self.bridge = CvBridge()
        self.isImage = False
        self.isDetection = False
        self.min_prop = 0.7
        self.depth_image = None
        self.rgb_image = None
        self.viz_image = None
        self.tracker = Tracker()
        self.dcnt=0
        self.depth_mv=[]
        self.K_camera = np.zeros((3, 3))
        self.distortion_camera = np.zeros((8)) 
        self.bboxes=[]
        self.pointclouds=[]
        self.marker_location=[]
        self.ptc=[]
        self.grid_length=0.0375
        row = 3
        col = 4
        
        for i in range(row):
            for j in range(col):
                loc = np.array([j*self.grid_length, i*self.grid_length, 0, 1])
                if i == 0 and j == 0:
                    self.marker_location = loc#[0:3]
                else:
                    self.marker_location = np.vstack([self.marker_location, loc])
        self.height = 480
        self.width = 640
        self.T_base_to_head = np.array([[ 3.93604419e-02, -3.09072655e-01,  9.27840227e-01, -5.37369173e-02],
                                        [-9.97538081e-01, -3.58104399e-02,  4.34783083e-02,  3.22804709e-02],
                                        [ 2.63874262e-02, -9.51119974e-01, -3.12488204e-01,  1.10943806e+00],
                                        [ 0.        ,  0.        ,  0.        ,  1.        ]])

        

        self.T_chessboard_to_external_cam = []        
        self.T_chessboard_to_robocare_cam = []
        self.T_robocare_to_external = []
                        
        
        self.box_pub = rospy.Publisher("/tracking_result", Detection3DArray, queue_size=10)
        rospy.Subscriber("/cam_e/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/cam_e/color/camera_info", CameraInfo, self.info_callback)
        rospy.Subscriber("/cam_e/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/bboxes_info", BoundingBoxes, self.detection_result_callback)
        rospy.Subscriber("/cam_e/calibration/result", Float32MultiArray, self.cam_e_callback)
        rospy.Subscriber("/camera/calibration/result", Float32MultiArray, self.cam_r_callback)


    def start(self):
        return

    def cam_e_callback(self, data):
        self.T_chessboard_to_external_cam = np.array(data.data).reshape(4,4)

    def cam_r_callback(self, data):
        self.T_chessboard_to_robocare_cam = np.array(data.data).reshape(4,4)



    def depth_callback(self, data):
        try:
            depth_input = self.bridge.imgmsg_to_cv2(data, "32FC1")  
            self.depth_image = cv2.medianBlur(depth_input,5)
            n=5
            if self.dcnt>0:
                self.depth_mv = (self.depth_mv * (n-1) + self.depth_image) / n
            self.depth_mv = self.depth_image
            self.dcnt = self.dcnt + 1 

        except CvBridgeError as e:
            print(e)
        self.isImage = True


    def rgb_callback(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")            
        except CvBridgeError as e:
            print(e)
        self.isImage = True
    
    def info_callback(self, data):
        for i in range(0, 3):
            for j in range(0, 3):
                self.K_camera[i][j] = data.K[i*3 + j]
        for i in range(5):
            self.distortion_camera[i] = data.D[i]

    def update(self):
        if self.isImage:
            centers = []
            bboxes = []
            ids = []
            if self.detected_objects:
                for obj in self.detected_objects.detections:
                    id = obj.results[0].id
                    corner = self.getBoundingBoxCorner(obj.bbox)
                    bbox = (corner[0], corner[1], corner[2]-corner[0], corner[3]-corner[1])
                    bboxes.append(bbox)   

                    center = (obj.bbox.center.x, obj.bbox.center.y)
                    centers.append(center) 
                    ids.append(id)
                    
            self.tracked_objects = self.tracker.update(ids, centers, bboxes,self.rgb_image)
            
            if self.T_chessboard_to_external_cam !=[] and self.T_chessboard_to_robocare_cam !=[]:
                self.T_robocare_to_external = np.linalg.inv(self.T_chessboard_to_robocare_cam).dot(self.T_chessboard_to_external_cam)
                self.T_robocare_base_to_external = self.T_base_to_head.dot(self.T_robocare_to_external)
                if self.depth_mv != []:
                    self.bboxes=[]
                    self.depths=[]
                    self.pointclouds=[]
                    tracking_result = Detection3DArray()
                    for i in range(len(self.tracked_objects)):
                        coord=[]
                        pts=[]
                        xmin_i=int(round(self.tracked_objects[i].bbox[0]))
                        ymin_i=int(round(self.tracked_objects[i].bbox[1]))
                        xmax_i=int(round(self.tracked_objects[i].bbox[0]+self.tracked_objects[i].bbox[2]))
                        ymax_i=int(round(self.tracked_objects[i].bbox[1]+self.tracked_objects[i].bbox[3]))
                        cnt = 0
                        for x in range(xmin_i, xmax_i):
                            for y in range(ymin_i, ymax_i):
                                depth = self.depth_image[y][x]

                                if depth !=0.0:
                                    pts.append(np.array([x,y]))

                                    temp = np.linalg.inv(self.K_camera).dot(np.array([x,y,1]))*depth/1000.0
                                    
                                    P = np.dot(self.T_robocare_base_to_external, np.array([temp[0],temp[1],temp[2],1.0]))
                                    if cnt==0:
                                        coord = P
                                    else:
                                        coord = np.vstack([coord, P])
                                    cnt=cnt+1
                        self.depths.append(pts)
                        self.pointclouds.append(coord)
                        
                        if coord !=[]:
                            detection = Detection3D()
                            
                            self.xmax_p=[]
                            self.ymax_p=[]
                            self.zmax_p=[]
                            self.xmin_p=[]
                            self.ymin_p=[]
                            self.zmin_p=[]

                            x = coord.T[0]
                            y = coord.T[1]
                            z = coord.T[2]
                            xlowerbound = x.mean() - 1.5*np.std(x)
                            xupperbound = x.mean() + 1.5*np.std(x)
                            ylowerbound = y.mean() - 1.5*np.std(y)
                            yupperbound = y.mean() + 1.5*np.std(y)
                            zlowerbound = z.mean() - 1.5*np.std(z)
                            zupperbound = z.mean() + 1.5*np.std(z)

                            newcoord = []

                            cnt=0

                            for cd in coord:
                                if cd[0]>xlowerbound and cd[0]<xupperbound and cd[1]>ylowerbound and cd[1]<yupperbound and cd[2]>zlowerbound and cd[2]<zupperbound:
                                    newcoord.append(cd)
                                cnt=cnt+1
                                
                            xmin = min(np.array(newcoord).T[0])
                            xmax = max(np.array(newcoord).T[0])
                            ymin = min(np.array(newcoord).T[1])
                            ymax = max(np.array(newcoord).T[1])
                            zmin = min(np.array(newcoord).T[2])
                            zmax = max(np.array(newcoord).T[2])

                            
                            detection.bbox.center.position.x = (xmin + xmax)/2
                            detection.bbox.center.position.y = (ymin + ymax)/2
                            detection.bbox.center.position.z = (zmin + zmax)/2
                            rot = np.identity(4)                            
                            quat=quaternion_from_matrix(rot)
                            detection.bbox.center.orientation.x = quat[0]
                            detection.bbox.center.orientation.y = quat[1]
                            detection.bbox.center.orientation.z = quat[2]
                            detection.bbox.center.orientation.w = quat[3]
                            detection.bbox.size.x = (xmax-xmin)
                            detection.bbox.size.y = (ymax-ymin)
                            detection.bbox.size.z = (zmax-zmin)
                            
                            detection.is_tracking = True

                            if self.tracked_objects[i].track_id == 0:
                                detection.tracking_id = "Background"
                            elif self.tracked_objects[i].track_id == 1:
                                detection.tracking_id = "1_red_gotica"
                            elif self.tracked_objects[i].track_id == 2:
                                detection.tracking_id = "2_bakey"
                            elif self.tracked_objects[i].track_id == 3:
                                detection.tracking_id = "3_pringles_onion"
                            elif self.tracked_objects[i].track_id == 4:
                                detection.tracking_id = "4_diget_box"
                            elif self.tracked_objects[i].track_id == 5:
                                detection.tracking_id = "5_diget"
                            elif self.tracked_objects[i].track_id == 6:
                                detection.tracking_id = "6_gotica"
                            elif self.tracked_objects[i].track_id == 7:
                                detection.tracking_id = "7_vitamin_water"
                            elif self.tracked_objects[i].track_id == 8:
                                detection.tracking_id = "8_diget_small_box"
                            
                            self.bboxes.append(detection.bbox)
                        tracking_result.detections.append(detection)
                    
                    self.box_pub.publish(tracking_result)
            else:
                rospy.loginfo("Waiting for the calibration info")
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
        if data.bounding_boxes:
            for i, obj in enumerate(data.bounding_boxes):
                if obj.probability > self.min_prop:  # if reliable
                    detected_object = Detection2D()
                    bb = BoundingBox2D()
                    cf = ObjectHypothesisWithPose()

                    bb.center.x = int(round((obj.xmin + obj.xmax)/2))
                    bb.center.y = int(round((obj.ymin + obj.ymax)/2))
                    bb.size_x = obj.xmax - obj.xmin 
                    bb.size_y = obj.ymax - obj.ymin 

                    cf.id = obj.Class
                    cf.score = obj.probability

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
            # if self.depths:
            #     for i in range(len(self.depths)):
            #         for pt in self.depths[i]:
            #             clr = self.tracked_objects[i].track_id % len(track_colors)
            #             cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 1, track_colors[clr], -1)

            
            
            K_camera = np.zeros((3,4))
            for r in range(3):
                for c in range(3):
                    K_camera[r][c]=self.K_camera[r][c] 

            if self.T_chessboard_to_external_cam !=[]: 
                for mk in self.marker_location:                    
                    mkp = np.dot(K_camera,np.dot(np.linalg.inv(self.T_chessboard_to_external_cam),mk))
                    
                    cv2.circle(vis_img, (int(mkp[0]/mkp[2]), int(mkp[1]/mkp[2])), 3, (0, 0, 255), -1)

            # if self.pointclouds:
            #     for pointcloud in self.pointclouds:
            #         for pt in pointcloud:
            #             mkp = np.dot(K_camera,np.dot(np.linalg.inv(self.T_chessboard_to_external_cam),pt))
            #             cv2.circle(vis_img, (int(mkp[0]/mkp[2]), int(mkp[1]/mkp[2])), 1, (127, 0, 127), -1)



            if self.bboxes and self.pointclouds:
                for i in range(len(self.bboxes)):
                    ptsw=np.zeros((8,4))
                    ptsc=np.zeros((8,3))
                    cx = self.bboxes[i].center.position.x
                    cy = self.bboxes[i].center.position.y
                    cz = self.bboxes[i].center.position.z
                    xlen = self.bboxes[i].size.x
                    ylen = self.bboxes[i].size.y
                    zlen = self.bboxes[i].size.z
                    ptsw[0] = np.array([cx - xlen/2, cy - ylen/2, cz - zlen/2, 1])
                    ptsw[1] = np.array([cx + xlen/2, cy - ylen/2, cz - zlen/2, 1])
                    ptsw[2] = np.array([cx + xlen/2, cy + ylen/2, cz - zlen/2, 1])
                    ptsw[3] = np.array([cx - xlen/2, cy + ylen/2, cz - zlen/2, 1])
                    ptsw[4] = np.array([cx - xlen/2, cy - ylen/2, cz + zlen/2, 1])
                    ptsw[5] = np.array([cx + xlen/2, cy - ylen/2, cz + zlen/2, 1])
                    ptsw[6] = np.array([cx + xlen/2, cy + ylen/2, cz + zlen/2, 1])
                    ptsw[7] = np.array([cx - xlen/2, cy + ylen/2, cz + zlen/2, 1])

                    
                    for j in range(8):
                        tmp=np.dot(K_camera,np.dot(np.linalg.inv(self.T_robocare_base_to_external),ptsw[j]))
                        ptsc[j][0]=tmp[0]/tmp[2]    
                        ptsc[j][1]=tmp[1]/tmp[2]
                        ptsc[j][2]=tmp[2]/tmp[2]
                    
                    p1 = (int(round(ptsc[0][0])), int(round(ptsc[0][1])))
                    p2 = (int(round(ptsc[1][0])), int(round(ptsc[1][1])))
                    p3 = (int(round(ptsc[2][0])), int(round(ptsc[2][1])))
                    p4 = (int(round(ptsc[3][0])), int(round(ptsc[3][1])))
                    p5 = (int(round(ptsc[4][0])), int(round(ptsc[4][1])))
                    p6 = (int(round(ptsc[5][0])), int(round(ptsc[5][1])))
                    p7 = (int(round(ptsc[6][0])), int(round(ptsc[6][1])))
                    p8 = (int(round(ptsc[7][0])), int(round(ptsc[7][1])))

                    
                    clr = self.tracked_objects[i].track_id % len(track_colors)
                    cv2.line(vis_img, p1, p2,track_colors[clr], 2)
                    cv2.line(vis_img, p1, p4,track_colors[clr], 2)
                    cv2.line(vis_img, p2, p3,track_colors[clr], 2)
                    cv2.line(vis_img, p3, p4,track_colors[clr], 2)
                    cv2.line(vis_img, p5, p6,track_colors[clr], 2)
                    cv2.line(vis_img, p5, p8,track_colors[clr], 2)
                    cv2.line(vis_img, p6, p7,track_colors[clr], 2)
                    cv2.line(vis_img, p7, p8,track_colors[clr], 2)
                    cv2.line(vis_img, p1, p5,track_colors[clr], 2)
                    cv2.line(vis_img, p2, p6,track_colors[clr], 2)
                    cv2.line(vis_img, p3, p7,track_colors[clr], 2)
                    cv2.line(vis_img, p4, p8,track_colors[clr], 2)


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
