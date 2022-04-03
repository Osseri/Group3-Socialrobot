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
    CONFIG_PATH = rospkg.RosPack().get_path('perception_interface') + '/config/'

    def __init__(self):
        self.listener = tf.TransformListener()
        self.dataset = []
        self.publishedTF = False
        self.base_to_camera_mat = {'mat':None, 'trans':None, 'rot':None}
        self.head_to_camera_mat = {'mat':None, 'trans':None, 'rot':None}
        self.T_chessboard_to_external_cam = []   
        self.T_chessboard_to_robocare_cam = []
        self.T_robocare_to_external = []          # robocare head camera to external camera
        self.T_robocare_base_to_external = []     # robocare base_footprint to external camera
        self.T_base_to_head = np.array([[ 3.93604419e-02, -3.09072655e-01,  9.27840227e-01, -5.37369173e-02],
                                        [-9.97538081e-01, -3.58104399e-02,  4.34783083e-02,  3.22804709e-02],
                                        [ 2.63874262e-02, -9.51119974e-01, -3.12488204e-01,  1.10943806e+00],
                                        [ 0.        ,  0.        ,  0.        ,  1.        ]])
        # sub & pub
        rospy.Subscriber("/cam_e/calibration/result", Float32MultiArray, self.cam_e_callback)
        rospy.Subscriber("/camera/calibration/result", Float32MultiArray, self.cam_r_callback)
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        # read configs
        self.read_camera_matrix()
        self.read_dataset()

        # get static tf from head to camera link
        self.update()


    def cam_e_callback(self, data):
        self.T_chessboard_to_external_cam = np.array(data.data).reshape(4,4)

    def cam_r_callback(self, data):
        self.T_chessboard_to_robocare_cam = np.array(data.data).reshape(4,4)

    def read_dataset(self):
        fname = self.CONFIG_PATH + 'dataset.txt'
        f = open(fname, "r")
        data = f.readlines()
        
        for x in data:
            self.dataset.append(x[:-1])        

    def read_camera_matrix(self):        
        fname = self.CONFIG_PATH + "camera_matrix.yaml"
        with open(fname) as f:
            data = yaml.safe_load(f)

        mat = data['camera_matrix']
        self.base_to_camera_mat['mat'] = mat
        self.base_to_camera_mat['trans'] = tf.transformations.translation_from_matrix(mat) 
        self.base_to_camera_mat['rot'] = tf.transformations.quaternion_from_matrix(mat)

    def calculate_head_to_camera_matrix(self):
        depth_to_camera_trans, depth_to_camera_rot, head_to_camera_trans, head_to_camera_rot = None

        # get trasform between depth to camera
        while(depth_to_camera_trans == None or depth_to_camera_rot == None 
                or head_to_camera_trans == None or head_to_camera_rot == None):
            try:      
                # self.publish_camera_tf('/base_footprint', '/perception_link', 
                #                     self.base_to_camera_mat['trans'], self.base_to_camera_mat['rot'])
                depth_to_camera_trans, depth_to_camera_rot = self.listener.lookupTransform('/camera_depth_optical_frame', '/camera_link', rospy.Time(0))
                head_to_camera_trans, head_to_camera_rot = self.listener.lookupTransform('/Head_Pitch', '/perception_link', rospy.Time(0))
            except:
                rospy.sleep(1)
                rospy.logwarn("wait for tf transform.")
                pass
        rospy.loginfo("camera transform is calculated.")
        
        trans_mat = tf.transformations.translation_matrix(head_to_camera_trans)
        rot_mat = tf.transformations.quaternion_matrix(head_to_camera_rot)
        head_to_depth_mat = np.dot(trans_mat, rot_mat) 

        trans_mat = tf.transformations.translation_matrix(depth_to_camera_trans)
        rot_mat = tf.transformations.quaternion_matrix(depth_to_camera_rot)
        depth_to_cam_mat = np.dot(trans_mat, rot_mat) 
        head_to_cam_mat = np.dot(head_to_depth_mat, depth_to_cam_mat) 

        trans = tf.transformations.translation_from_matrix(head_to_cam_mat) 
        rot = tf.transformations.quaternion_from_matrix(head_to_cam_mat)

        self.head_to_camera_mat['trans'] = trans
        self.head_to_camera_mat['rot'] = rot
        # self.publish_camera_tf('/Head_Pitch', '/camera_link', 
        #                         self.head_to_camera_mat['trans'], 
        #                         self.head_to_camera_mat['rot'])

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

    def publish_external_tf(self):
        if len(self.T_robocare_to_external)>0:
          base_to_camera_trans, base_to_color_rot= None, None
          camera_to_color_trans, camera_to_color_rot = None, None
          color_to_cam_trans, color_to_cam_rot = None, None

          # get trasform between depth to camera
          while(base_to_camera_trans == None or base_to_color_rot == None
                or camera_to_color_trans == None or camera_to_color_rot == None
                or color_to_cam_trans == None or color_to_cam_rot == None):
            try:     
                base_to_camera_trans, base_to_color_rot = self.listener.lookupTransform('/base_footprint', '/perception_link', rospy.Time(0))
            except:            
                rospy.sleep(1)
                rospy.logwarn("wait for tf transform between /base_footprint and /perception_link.")   
                pass
            try:                      
                camera_to_color_trans, camera_to_color_rot = self.listener.lookupTransform('/camera_depth_optical_frame', '/camera_color_optical_frame', rospy.Time(0))
            except:            
                rospy.sleep(1)
                rospy.logwarn("wait for tf transform between /camera_depth_optical_frame and /camera_color_optical_frame..")   
                pass
            try:     
                color_to_cam_trans, color_to_cam_rot = self.listener.lookupTransform('/cam_e_color_optical_frame', '/cam_e_link', rospy.Time(0))
            except:            
                rospy.sleep(1)
                rospy.logwarn("wait for tf transform between /cam_e_color_optical_frame and /cam_e_link..")   
                pass
            
                
          # base_footprint-> camera_color_optical_frame
          trans_mat = tf.transformations.translation_matrix(base_to_camera_trans)
          rot_mat = tf.transformations.quaternion_matrix(base_to_color_rot)
          base_to_camera_mat = np.dot(trans_mat, rot_mat) 

          trans_mat = tf.transformations.translation_matrix(camera_to_color_trans)
          rot_mat = tf.transformations.quaternion_matrix(camera_to_color_rot)
          base_to_color_mat = base_to_camera_mat.dot(np.dot(trans_mat, rot_mat))

          # base_footprint-> external_color_optical_frame
          camera_to_external_mat = self.T_robocare_to_external
          base_to_external_mat = np.dot(base_to_color_mat, camera_to_external_mat)

          # base_footprint-> external_camera_link
          trans_mat = tf.transformations.translation_matrix(color_to_cam_trans)
          rot_mat = tf.transformations.quaternion_matrix(color_to_cam_rot)
          color_to_external_mat = np.dot(trans_mat, rot_mat) 
          base_to_cam_mat = np.dot(base_to_external_mat, color_to_external_mat)

          trans = tf.transformations.translation_from_matrix(base_to_cam_mat) 
          rot = tf.transformations.quaternion_from_matrix(base_to_cam_mat) 
          self.publish_camera_tf('/base_footprint', '/cam_e_link', trans, rot)

    def update_camera_transform(self):
        if len(self.T_chessboard_to_external_cam)>0 and len(self.T_chessboard_to_robocare_cam)>0:
          # add offset
          robocare_to_chessboard = np.linalg.inv(self.T_chessboard_to_robocare_cam)

          trans = tf.transformations.translation_matrix([-0.01,0.05,-0.03])
          quaternion = tf.transformations.quaternion_from_euler(-3/180*math.pi, 1.5/180*math.pi, -0/180*math.pi)
          rot   = tf.transformations.quaternion_matrix(quaternion)
          offset = np.dot(rot, trans)
          fixed_robocare_to_chessboard = robocare_to_chessboard.dot(offset)

          # transform between robocare color_optical_frame to external color_optical_frame
          self.T_robocare_to_external = fixed_robocare_to_chessboard.dot(self.T_chessboard_to_external_cam)

          # publish external camera link tf
          trans = tf.transformations.translation_from_matrix(fixed_robocare_to_chessboard) 
          rot = tf.transformations.quaternion_from_matrix(fixed_robocare_to_chessboard) 
          self.publish_camera_tf('/camera_color_optical_frame', '/chess', trans, rot)


    def update(self):  
        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

            self.update_camera_transform()
            self.publish_external_tf()


        
##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('camera_tf_publisher')        

    # perception manager
    tf_publisher = TFPublisher()
    rospy.spin()
