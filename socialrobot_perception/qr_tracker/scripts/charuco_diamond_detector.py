#!/usr/bin/env python
import os
import sys

import numpy as np
import cv2

### ROS STUFF
import rospy
import rosparam
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from std_msgs.msg import Int32, Bool, Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped

# ChAruco board variables
CHARUCOBOARD_ROWCOUNT = 7
CHARUCOBOARD_COLCOUNT = 5 
SQUARE_LENGTH = 0.054
MARKER_LENGTH = 0.032
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_1000)

# Create constants to be passed into OpenCV and Aruco methods
CHARUCO_BOARD = cv2.aruco.CharucoBoard_create(
        squaresX=CHARUCOBOARD_COLCOUNT,
        squaresY=CHARUCOBOARD_ROWCOUNT,
        squareLength=SQUARE_LENGTH,
        markerLength=MARKER_LENGTH,
        dictionary=ARUCO_DICT)

class MarkerDetector:
    def __init__(self):
        rospy.init_node('diamond_detector')
        self.bridge = CvBridge()

        self.camera_points = []
        self.robot_points = []
        self.marker_location = []
        self.marker_location_list = []
        self.imgpoints_list = []

        self.frame_goal = 1
        self.frame_cnt = 0

        self.K_camera = np.zeros((3, 3))
        self.distortion_camera = np.zeros((8))
        self.P_camera = np.zeros((3, 4))

        self.grid_length = 0.055
        self.calib_iter = 1
        self.chessboard_size = np.array([6,4])

        self.motion_num = Int32()
        
        # subscriber
        self.image_sub = rospy.Subscriber("/cam_e/color/image_raw", Image, self.img_callback)
        self.intrinsic_sub = rospy.Subscriber("/cam_e/color/camera_info", CameraInfo, self.camera_callback)

        # publisher
        self.result_pub = rospy.Publisher("/cam_e/calibration/result", Float32MultiArray, queue_size=10)
    
        row = self.chessboard_size[0]
        col = self.chessboard_size[1]

        for i in range(row):
            for j in range(col):
                loc = np.array([j*self.grid_length, i*self.grid_length, 0, 1])
                if i == 0 and j == 0:
                    self.marker_location = loc[0:3]
                else:
                    self.marker_location = np.vstack([self.marker_location, loc[0:3]])

        rospy.loginfo("Init " + rospy.get_name() + " node")

    def camera_callback(self, data):
        for i in range(0, 3):
            for j in range(0, 3):
                self.K_camera[i][j] = data.K[i*3 + j]
        for i in range(len(data.D)):
            self.distortion_camera[i] = data.D[i]     

    def img_callback(self, data):
    
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        img = cv2.undistort(img, self.K_camera, self.distortion_camera, None, self.K_camera)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners_all = [] # Corners discovered in all images processed
        ids_all = [] # Aruco ids corresponding to corners discovered

        # Find aruco markers in the query image
        corners, ids, _ = cv2.aruco.detectMarkers(image=gray, dictionary=ARUCO_DICT)

        # Outline the aruco markers found in our query image
        if( ids is not None ):
            img = cv2.aruco.drawDetectedMarkers(image=img, corners=corners)
            diamond_corners, diamond_ids = cv2.aruco.detectCharucoDiamond(img, corners, ids,
                                               SQUARE_LENGTH/MARKER_LENGTH, cameraMatrix=self.K_camera,
                                               distCoeffs=self.distortion_camera)
            cv2.aruco.drawDetectedDiamonds(img, diamond_corners, diamond_ids)

            if diamond_corners is not None and diamond_ids is not None and len(diamond_corners) == len(diamond_ids):
                ret = cv2.aruco.estimatePoseSingleMarkers(diamond_corners, SQUARE_LENGTH, self.K_camera, self.distortion_camera)       
                (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
                cv2.aruco.drawAxis(img, self.K_camera, self.distortion_camera, rvec, tvec, 10)  
        else:
            pass

        cv2.imshow('Charuco board', img)
        cv2.waitKey(1)

def main(args):
    MD = MarkerDetector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
