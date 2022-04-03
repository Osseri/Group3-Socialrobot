#!/usr/bin/env python
import yaml
import rospy
import rospkg

from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_perception_msgs.srv import *
from socialrobot_perception_msgs.msg import *
from obj_msg.msg import *
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection3DArray

import tf
import tf.transformations as tfm
import tf2_ros
import tf2_msgs.msg
import numpy as np 
import math

class TFPublisher():

    def __init__(self):   
        self.T_odom_to_cam_e = None        
        self.listener = tf.TransformListener()     
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        self.robot_base_tf = 'hand_plate'
        self.camera_base_tf = 'QR:49'
        self.robot_base = 'base_footprint'
        self.camera_base = 'camera_link'
        self.robot_head = 'Head_Pitch'

        self.update()


    def get_tf(self):
        self.listener.waitForTransform(self.robot_base, self.robot_base_tf, rospy.Time(), rospy.Duration(4.0))
        self.listener.waitForTransform(self.robot_head, self.robot_base_tf, rospy.Time(), rospy.Duration(4.0))
        self.listener.waitForTransform(self.camera_base, self.camera_base_tf, rospy.Time(), rospy.Duration(4.0))

        robot_to_marker_trans, robot_to_marker_rot= None, None     # robot to marker
        head_to_marker_trans, head_to_marker_rot= None, None     # robot to marker
        marker_to_cam_trans, marker_to_cam_rot = None, None    # camera to marker

        # get trasforms    
        robot_to_marker_trans, robot_to_marker_rot = self.listener.lookupTransform(self.robot_base_tf, self.robot_base, rospy.Time(0))        
        head_to_marker_trans, head_to_marker_rot = self.listener.lookupTransform(self.robot_base_tf, self.robot_head, rospy.Time(0))
        marker_to_cam_trans, marker_to_cam_rot = self.listener.lookupTransform(self.camera_base_tf, self.camera_base, rospy.Time(0))
                 
                
        # base -> marker
        trans_mat = tf.transformations.translation_matrix(robot_to_marker_trans)
        rot_mat = tf.transformations.quaternion_matrix(robot_to_marker_rot)
        base_to_marker = np.linalg.inv(np.dot(trans_mat, rot_mat))

        # head -> marker
        trans_mat = tf.transformations.translation_matrix(head_to_marker_trans)
        rot_mat = tf.transformations.quaternion_matrix(head_to_marker_rot)
        head_to_marker = np.linalg.inv(np.dot(trans_mat, rot_mat))

        # marker -> camera
        trans_mat = tf.transformations.translation_matrix(marker_to_cam_trans)
        rot_mat = tf.transformations.quaternion_matrix(marker_to_cam_rot)
        marker_to_cam = np.dot(trans_mat, rot_mat) 


        # head -> camera
        head_to_cam = np.dot(head_to_marker, marker_to_cam)
        trans = tf.transformations.translation_from_matrix(head_to_cam) 
        rot = tf.transformations.quaternion_from_matrix(head_to_cam) 
        self.T_head_to_cam = [trans, rot]

        print('---------- head to camera ----------')
        print('Matrix:', head_to_cam)
        print('Translation:', trans)
        print('Rotation:', rot)

        # base -> camera
        base_to_cam = np.dot(base_to_marker, marker_to_cam)
        trans = tf.transformations.translation_from_matrix(base_to_cam) 
        rot = tf.transformations.quaternion_from_matrix(base_to_cam) 
        self.T_base_to_cam = [trans, rot]

        print('---------- base to camera ----------')
        print('Matrix:', base_to_cam)
        print('Translation:', trans)
        print('Rotation:', rot)

    def publish_camera_tf(self, from_tf, to_tf, trans, rot, broadcaster=None):
        quat_norm = rot / np.linalg.norm(rot)

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = from_tf
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = to_tf
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]

        t.transform.rotation.x = quat_norm[0]
        t.transform.rotation.y = quat_norm[1]
        t.transform.rotation.z = quat_norm[2]
        t.transform.rotation.w = quat_norm[3]

        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)

    def update(self):  
        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

            # get static tf 
            self.get_tf()
            
            self.publish_camera_tf(self.robot_head, 'camera_link', self.T_head_to_cam[0], self.T_head_to_cam[1])

##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('handeye_calibrator')        

    # perception manager
    tf_publisher = TFPublisher()
    rospy.spin()
