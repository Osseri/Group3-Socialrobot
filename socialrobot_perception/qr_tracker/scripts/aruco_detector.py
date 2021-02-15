import rospy 
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco

class ArucoDetector():
    '''
    aruco detector
    '''

    def __init__(self):
        rospy.loginfo('[ArucoDetector] Registration Started!')
        self.camera_info='/camera/color/camera_info'
        self.image_topic='/camera/color/image_raw'
        self.bridge = CvBridge()
        self.dist=[]
        self.mtx=[]
        self.frame_id=''

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info, CameraInfo, self.camera_info_callback)
        self.image_pub = rospy.Publisher("aruco_detector/result", Image, queue_size=10)


    def __del__(self):
        # Exit
        rospy.loginfo('[ArucoDetector] terminated!')

    def camera_info_callback(self, data):
        #convert ros msg to cv2 format
        self.mtx = np.reshape(np.array(data.K), (-1,3))
        self.dist = np.array(data.D)   
        self.frame_id = data.header.frame_id

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # operations on the frame come here
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        parameters = aruco.DetectorParameters_create()
        markerLength = 0.02675

        #lists of ids and the corners beloning to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
        
        if np.all(ids != None):
            #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
            rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners, markerLength, self.mtx, self.dist) 
            #(rvec-tvec).any() # get rid of that nasty numpy value array error
            
            # Draw Axis
            for i in range(0, ids.size):                
                aruco.drawAxis(cv_image, self.mtx, self.dist, rvec[i], tvec[i], 0.05) 
                strg = str(ids[i][0])+', '
                #cv2.putText(cv_image, "Id: " + strg, (0,64), font, 1, (0,255,0), 2, cv2.LINE_AA)
            # Draw A square around the markers
            #aruco.drawDetectedMarkers(cv_image, corners) 
                   
            


        (rows,cols,channels) = cv_image.shape
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

if __name__ =='__main__':
    # Initialize ROS node
    rospy.init_node('aruco_detector', anonymous=True)
    ad = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")