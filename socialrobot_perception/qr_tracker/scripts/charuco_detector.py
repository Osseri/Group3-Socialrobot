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
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_1000)

# Create constants to be passed into OpenCV and Aruco methods
CHARUCO_BOARD = cv2.aruco.CharucoBoard_create(
        squaresX=CHARUCOBOARD_COLCOUNT,
        squaresY=CHARUCOBOARD_ROWCOUNT,
        squareLength=0.1,
        markerLength=0.08,
        dictionary=ARUCO_DICT) 
# save board
imboard = CHARUCO_BOARD.draw((3508, 4961))
cv2.imwrite( "chessboard.tiff", imboard)

class MarkerDetector:
    def __init__(self):
        rospy.init_node('charuco_detector')
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

            # Get charuco corners and ids from detected aruco markers
            response, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                                                            markerCorners=corners,
                                                            markerIds=ids,
                                                            image=gray,
                                                            board=CHARUCO_BOARD)

            # If a Charuco board was found, let's collect image/corner points
            # Requiring at least 20 squares
            if response > 5:
                # Add these corners and ids to our calibration arrays
                corners_all.append(charuco_corners)
                ids_all.append(charuco_ids)
                
                # Draw the Charuco board we've detected to show our calibrator the board was properly detected
                img = cv2.aruco.drawDetectedCornersCharuco(
                        image=img,
                        charucoCorners=charuco_corners,
                        charucoIds=charuco_ids)

                # posture estimation from a charuco board
                retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(charuco_corners, charuco_ids, CHARUCO_BOARD, self.K_camera, self.distortion_camera)  

                if retval:
                    # axis length 100 can be changed according to your requirement
                    cv2.aruco.drawAxis(img, self.K_camera, self.distortion_camera, rvec, tvec, 100)  
        
            else:
                print('cannot detect aruco corners')

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
