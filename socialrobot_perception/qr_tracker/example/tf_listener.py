#!/usr/bin/env python
import rospy
import rospkg
import rosparam
import tf
import yaml
from visualization_msgs.msg import Marker
from vision_msgs.msg import BoundingBox3D
import socialrobot_interface.msg
import numpy

class TFListener():

    def __init__(self):

        self.listener = tf.TransformListener()
        self.br =  tf.TransformBroadcaster()

    def run(self):

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            frames = self.listener.allFramesAsString()
            
            base_frame = "Head_Pitch"
            qrcode_frame = "right_qrcode"          
            trans1 = rot1 = []     
            trans2 = rot2 = []
            if self.listener.frameExists(qrcode_frame) and self.listener.frameExists(base_frame):
                (trans1,rot1) = self.listener.lookupTransform(base_frame, qrcode_frame, rospy.Time(0))

                trans1_mat = tf.transformations.translation_matrix(trans1)
                rot1_mat   = tf.transformations.quaternion_matrix(rot1)
                mat1 = numpy.dot(trans1_mat, rot1_mat)
                
                camera_frame = "camera_rgb_frame"
                qrcode_frame = "QR:5"    
                if self.listener.frameExists(qrcode_frame) and self.listener.frameExists(camera_frame):
                    (trans2,rot2) = self.listener.lookupTransform(qrcode_frame, camera_frame, rospy.Time(0))

                    trans2_mat = tf.transformations.translation_matrix(trans2)
                    rot2_mat    = tf.transformations.quaternion_matrix(rot2)
                    mat2 = numpy.dot(trans2_mat, rot2_mat)
  

                    mat3 = numpy.dot(mat1, mat2)
                    trans3 = tf.transformations.translation_from_matrix(mat3) 
                    rot3 = tf.transformations.quaternion_from_matrix(mat3)

                    print trans3
                    print rot3

                    br = tf.TransformBroadcaster()
                    br.sendTransform(
                    trans3,
                    rot3,
                    rospy.Time.now(),
                    "camera_link",
                    base_frame);

        return


if __name__ =='__main__':
    # Initialize ROS node
    rospy.init_node('test', anonymous=True)   
    tl = TFListener()

    try:
        tl.run()
    except KeyboardInterrupt:
        print("Shutting down")