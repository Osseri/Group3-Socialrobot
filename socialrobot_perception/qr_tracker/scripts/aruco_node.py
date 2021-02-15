#!/usr/bin/env python
import rospy
import rospkg
import rosparam

from aruco_detector import ArucoDetector


if __name__ =='__main__':
    # Initialize ROS node
    rospy.init_node('aruco_detector', anonymous=True)
    ad = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")