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

class image_converter:
  def __init__(self):
    rospy.init_node('extrinsic_calibrator')
    self.bridge = CvBridge()

    self.marker_location = []

    self.K_camera = np.zeros((3, 3))
    self.distortion_camera = np.zeros((8))
    self.P_camera = np.zeros((3, 4))

    self.grid_length = 0.055
    self.chessboard_size = np.array([6,4])

    self.cam_info=rosparam.get_param(rospy.get_name()+"/cam")

    if self.cam_info==0:
      # subscriber
      self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.img_callback)
      self.intrinsic_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_callback)
      # publisher
      self.image_pub = rospy.Publisher("/camera/calibration/image", Image, queue_size=10)
      self.result_pub = rospy.Publisher("/camera/calibration/result", Float32MultiArray, queue_size=10)
      # service
      #rospy.ServiceProxy("/camera/get_transform",)  
      self.cam = 'robocare cam'

    elif self.cam_info==1:
      self.image_sub = rospy.Subscriber("/cam_e/color/image_raw", Image, self.img_callback)
      self.intrinsic_sub = rospy.Subscriber("/cam_e/color/camera_info", CameraInfo, self.camera_callback)
      # publisher
      self.image_pub = rospy.Publisher("/cam_e/calibration/image", Image, queue_size=10)
      self.result_pub = rospy.Publisher("/cam_e/calibration/result", Float32MultiArray, queue_size=10)
      # service
      #rospy.ServiceProxy("/cam_e/get_transform",)  
      self.cam = 'external cam'


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
    
      if len(self.marker_location)>0:
        row = self.chessboard_size[0]
        col = self.chessboard_size[1]        

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        imgpoints = np.zeros((row*col,2))
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        img = cv2.undistort(img, self.K_camera, self.distortion_camera, None, self.K_camera)
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (col, row), None)

        if ret == True:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            for i in range(row * col):
              imgpoints[i][0] = (corners2[i][0][0])
              imgpoints[i][1] = (corners2[i][0][1])
            img = cv2.drawChessboardCorners(img, (col, row), corners2, ret)
            for i in range(row * col):
              location=(int(imgpoints[i][0]), int(imgpoints[i][1]))
              font = cv2.FONT_HERSHEY_SIMPLEX
              fontScale = 1.0
              #cv2.putText(img, str(i+1), location, font, fontScale, (0, 0, 255), 2)
           
            retval, rvec, tvec = cv2.solvePnP(self.marker_location, imgpoints, self.K_camera, None)             
            rotation_matrix = np.zeros((3, 3))
            cv2.Rodrigues(rvec, rotation_matrix)
            HT = np.zeros((4, 4))
            for i in range(3):
              for j in range(3):
                HT[i][j] = rotation_matrix[i][j]
              HT[i][3] = tvec[i]
              HT[3][i] = 0
            HT[3][3] = 1
            HT = np.linalg.inv(HT)

            result = Float32MultiArray()

            for H in HT:
              for i in range(4):
                result.data.append(H[i])

            # draw axis
            cv2.aruco.drawAxis(img, self.K_camera, self.distortion_camera, rvec, tvec, 100)  

            self.result_pub.publish(result)    
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img))  
            #cv2.imshow(self.cam, img)
            #cv2.waitKey(1)
        else:  
            rospy.logwarn("cannot detect chessboard with " + self.cam)        
        

def main(args):
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
