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

        self.external_chess = '/cam_e_chessboard'
        self.external_camera = rospy.get_param("~external_camera_link", "/cam_e_link")
        self.robot_chess = '/chessboard'
        self.fixed_map_tf = '/odom'

        # get map tf
        pos = ori = None
        try:
            if rospy.has_param("~base_frame"):
                self.fixed_map_tf = rospy.get_param("~base_frame")
            self.listener.waitForTransform(self.fixed_map_tf, self.robot_chess, rospy.Time(), rospy.Duration(1.0))
            pos,ori = self.listener.lookupTransform(self.fixed_map_tf, self.robot_chess, rospy.Time(0))

            # if map found
            rospy.loginfo("found transform between %s and %s." %(self.fixed_map_tf, self.robot_chess))
            self.found_map = True  
        except:
            return
            
        # get static tf 
        if not self.get_tf():
            return

        self.update()

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

    def get_tf(self):
        chess_to_cam_e_trans, chess_to_cam_e_rot= None, None     # chessboard to external camera
        chess_to_odom_trans, chess_to_odom_rot = None, None    # chessboard to odom

        # get trasform between depth to camera     
        while(chess_to_cam_e_trans == None or chess_to_cam_e_rot == None or chess_to_odom_trans == None or chess_to_odom_rot == None):  
            try:     
                self.listener.waitForTransform(self.external_chess, self.external_camera, rospy.Time(), rospy.Duration(1.0))
                chess_to_cam_e_trans, chess_to_cam_e_rot = self.listener.lookupTransform(self.external_chess, self.external_camera, rospy.Time(0))
            except:           
                rospy.logwarn("[chess_calibrator] cannot find transform between %s and %s." %(self.external_chess, self.external_camera))   
                #return False

            try:     
                self.listener.waitForTransform(self.robot_chess, self.fixed_map_tf, rospy.Time(), rospy.Duration(1.0))
                chess_to_odom_trans, chess_to_odom_rot = self.listener.lookupTransform(self.robot_chess, self.fixed_map_tf, rospy.Time(0))
            except:            
                rospy.logwarn("[chess_calibrator] cannot find transform between %s and %s." %(self.robot_chess, self.fixed_map_tf))
                #return False                
                
        # odom -> chessboard
        trans_mat = tf.transformations.translation_matrix(chess_to_odom_trans)
        rot_mat = tf.transformations.quaternion_matrix(chess_to_odom_rot)
        odom_to_chess_mat = np.linalg.inv(np.dot(trans_mat, rot_mat))

        # chessboard -> cam_e_link
        trans_mat = tf.transformations.translation_matrix(chess_to_cam_e_trans)
        rot_mat = tf.transformations.quaternion_matrix(chess_to_cam_e_rot)
        chess_to_cam_e_mat = np.dot(trans_mat, rot_mat) 

        odom_to_cam_e_mat = np.dot(odom_to_chess_mat, chess_to_cam_e_mat)

        trans = tf.transformations.translation_from_matrix(odom_to_cam_e_mat) 
        rot = tf.transformations.quaternion_from_matrix(odom_to_cam_e_mat) 
        self.T_odom_to_cam_e = [trans, rot]

        return True

    def update(self):  
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_camera_tf(self.fixed_map_tf, self.external_camera, 
                                    self.T_odom_to_cam_e[0], self.T_odom_to_cam_e[1])
            # Run this loop at about 10Hz
            rate.sleep()            

        
##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('chess_calibrator')        

    # perception manager
    tf_publisher = TFPublisher()
