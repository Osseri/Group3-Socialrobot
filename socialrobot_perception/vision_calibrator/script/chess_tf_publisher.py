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
        self.listener = tf.TransformListener()     
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub_initpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        self.robot_frame = '/base_footprint'
        self.robot_chess = '/cam_r_chessboard'
        self.external_chess = '/cam_e_chessboard'
        self.fixed_chess = '/chessboard'
        self.fixed_map_tf = '/map'
        self.found_map = False

        # static transform between map and chessboard
        map_to_robot = []
        map_to_chess = [[+1.6457e+00, +7.2900e-01, +1.0458e+00],
                        [-0.656756, 0.262052, 0.656756, 0.262052]]
        T_map_to_chess = np.dot(tf.transformations.translation_matrix(map_to_chess[0]), 
                                tf.transformations.quaternion_matrix(map_to_chess[1])) 

        # get transform between chess and robot
        pos = ori = None
        try:
            self.listener.waitForTransform(self.robot_chess, self.robot_frame, rospy.Time(), rospy.Duration(1.0))
            pos,ori = self.listener.lookupTransform(self.robot_chess, self.robot_frame, rospy.Time(0))
            T_chess_to_robot = np.dot(tf.transformations.translation_matrix(pos), 
                                        tf.transformations.quaternion_matrix(ori))
            T_map_to_robot = np.dot(T_map_to_chess, T_chess_to_robot) 
            map_to_robot = [tf.transformations.translation_from_matrix(T_map_to_robot),
                            tf.transformations.quaternion_from_matrix(T_map_to_robot)]              
        except:
            rospy.logerr("cannot find transform between %s and %s." %(self.robot_chess, self.robot_frame))
            return 

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
                    
            # init pose
            self.set_initial_pose(map_to_robot)

        except:
            rospy.logwarn("cannot find transform between %s and %s." %(self.fixed_map_tf, self.robot_chess))
            self.fixed_map_tf = "/odom"
            # if cannot find map, get odom tf 
            pos, ori = self.get_tf(self.fixed_map_tf, self.robot_chess)

        self.publish_chess_tf(pos, ori)

    def publish_chess_tf(self, trans, rot):
        
        while not rospy.is_shutdown():
            quat_norm = rot / np.linalg.norm(rot)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = self.fixed_map_tf
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = self.fixed_chess
            t.transform.translation.x = trans[0]
            t.transform.translation.y = trans[1]
            t.transform.translation.z = trans[2]

            t.transform.rotation.x = quat_norm[0]
            t.transform.rotation.y = quat_norm[1]
            t.transform.rotation.z = quat_norm[2]
            t.transform.rotation.w = quat_norm[3]

            tfm = tf2_msgs.msg.TFMessage([t])

            # Run this loop at about 10Hz
            rospy.sleep(0.1)
            self.pub_tf.publish(tfm)

    def get_tf(self, from_tf, to_tf):
        pos = ori = None        
        try:
            self.listener.waitForTransform(from_tf, to_tf, rospy.Time(), rospy.Duration(1.0))
            pos,ori = self.listener.lookupTransform(from_tf, to_tf, rospy.Time(0))
            rospy.loginfo("found transform between %s and %s." %(from_tf, to_tf))
        except:
            rospy.logerr("cannot find transform between %s and %s." %(from_tf, to_tf))

        return pos, ori

    def set_initial_pose(self, robot_pose):        
        req = PoseWithCovarianceStamped()
        req.header.stamp = rospy.Time.now()
        req.header.frame_id = self.fixed_map_tf
        req.pose.pose.position.x = robot_pose[0][0]
        req.pose.pose.position.y = robot_pose[0][1]
        req.pose.pose.orientation.x = robot_pose[1][0]
        req.pose.pose.orientation.y = robot_pose[1][1]
        req.pose.pose.orientation.z = robot_pose[1][2]
        req.pose.pose.orientation.w = robot_pose[1][3]
        req.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        self.pub_initpose.publish(req)

       
##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('chess_tf_publisher')        

    # perception manager
    tf_publisher = TFPublisher()
