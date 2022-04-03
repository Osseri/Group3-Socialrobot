#!/usr/bin/env python
import rospy 
import rosparam
import numpy as np
from std_msgs.msg import Int32MultiArray, Int32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import tf

class ArucoDetector():
    '''
    aruco detector
    '''

    def __init__(self):
        rospy.loginfo('[ArucoDetector] Registration Started!')
        self.bridge = CvBridge()
        self.dist=[]
        self.mtx=[]
        self.frame_id=''

        self.tf_pub = tf.TransformBroadcaster()

        self.markerLength  = rospy.get_param("~marker_size", default=0.05)
        self.camera_frame = rospy.get_param("~camera_frame", default='camera_color_optical_frame')
        self.image_topic = rospy.get_param("~color_topic", default='/camera/color/image_raw')
        self.camera_info = rospy.get_param("~color_info", default='/camera/color/camera_info')

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info, CameraInfo, self.camera_info_callback)
        self.image_pub = rospy.Publisher("/aruco_detector/result", Image, queue_size=10)
        self.qrlist_pub = rospy.Publisher("/aruco_detector/detected_markers", Int32MultiArray, queue_size=10)

    def __del__(self):
        pass

    def camera_info_callback(self, data):
        #convert ros msg to cv2 format
        self.mtx = np.reshape(np.array(data.K), (-1,3))
        self.dist = np.array(data.D)   
        self.frame_id = data.header.frame_id

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
            # operations on the frame come here
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters = aruco.DetectorParameters_create()

            #lists of ids and the corners beloning to each id
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
            
            marker_arr = Int32MultiArray()
            if np.all(ids != None):

                #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
                rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners, self.markerLength, self.mtx, self.dist) 
                #(rvec-tvec).any() # get rid of that nasty numpy value array error

                # Draw Axis
                for i in range(0, ids.size):                
                    aruco.drawAxis(cv_image, self.mtx, self.dist, rvec[i], tvec[i], 0.05) 
                    strg = "QR:" + str(ids[i][0])
                    corner = (int(corners[i][0][0][0]),int(corners[i][0][0][1]))
                    cv2.putText(cv_image, strg, (corner), font, 1, (0,255,0), 2, cv2.LINE_AA)
                
                    # publish tf
                    self.aruco2tf(strg, rvec[i][0], tvec[i][0])

                    marker = Int32()
                    marker_arr.data.append(ids[i][0])
                # publish marker list
                self.qrlist_pub.publish(marker_arr)
                
                # Draw A square around the markers
                aruco.drawDetectedMarkers(cv_image, corners)   
            (rows,cols,channels) = cv_image.shape
            #cv2.imshow("Image window", cv_image)
            #cv2.waitKey(1)

        except CvBridgeError as e:
            pass

        # try:
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #     pass

    def aruco2tf(self, id, rvec, tvec):
        rotation_matrix = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 1]],
                            dtype=float)
        rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
        quat = tf.transformations.quaternion_from_matrix(rotation_matrix)
        
        self.tf_pub.sendTransform((tvec[0], tvec[1], tvec[2]), 
                (quat[0], quat[1], quat[2], quat[3]),
                rospy.Time.now(),
                id, self.camera_frame)

        return 

if __name__ =='__main__':
    # Initialize ROS node
    rospy.init_node('aruco_detector', anonymous=True)
    ad = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
