#!/usr/bin/env python
import rospy
import rospkg
import rosparam
import tf
import tf.transformations as tfm
import yaml
import sys
from math import pi
import numpy
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import BoundingBox3D
from geometry_msgs.msg import Pose, Point, Quaternion
import socialrobot_perception_msgs.msg
from socialrobot_msgs.msg import Object, Objects, Affordance
import numpy as np

def pose_2_mat(trans, rot):
    trans_vec = [trans['x'], trans['y'], trans['z']]
    rot_vec = [rot['x'], rot['y'], rot['z'], rot['w']]

    trans_mat = tf.transformations.translation_matrix(trans_vec)
    rot_mat = tf.transformations.quaternion_matrix(rot_vec)
    T = np.dot(trans_mat, rot_mat) 
    return T

def mat_2_pose(T):
    trans = tf.transformations.translation_from_matrix(T) 
    rot = tf.transformations.quaternion_from_matrix(T)
    
    pose_trans = {'x': trans[0], 'y': trans[1], 'z': trans[2]}
    pose_rot = {'x': rot[0], 'y': rot[1], 'z': rot[2], 'w': rot[3]}
    return pose_trans, pose_rot

class ObjectCreator():

    def __init__(self, config_path):

        self.listener = tf.TransformListener()
        self.config_path = config_path
        self.config_data = []
        self.base_frame = rospy.get_param("base_frame", default='base_footprint')
        self.objects = []
  
        self.pub_objects = rospy.Publisher("/perception/objects", socialrobot_perception_msgs.msg.Objects, queue_size=10)
        self.pub_objs = rospy.Publisher("/socialrobot/perception/objects", Objects, queue_size=10)
        self.pub_marker = rospy.Publisher("/socialrobot/perception/markers", MarkerArray, queue_size=10)   


    def run(self):
        # load yaml
        data = self.load(self.config_path)
        if not data:
            return

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            frames = self.listener.allFramesAsString()
            marker_array = MarkerArray()
            objects = []

            # detect objects
            for obj in self.config_data['objects']:

                # static objects
                if obj['name'] == 'obj_table':
                    name =  obj['name']
                    size = obj['size']
                    trans = obj['translation']
                    rot = obj['rotation']

                # dynamic objects 
                else:
                    marker_frame = 'QR:'+str(obj['id'])
                    trans = rot = None
                    
                    try:
                        (trans,rot) = self.listener.lookupTransform(marker_frame, 'base_footprint', rospy.Time(0))
                    except:
                        pass
                    if trans and rot:                     
                        # create object instance
                        obj = self.createObjectMsg(obj, parent_frame=self.base_frame, child_frame=marker_frame)
                        objects.append(obj) 

                        # create marker for base object
                        markers = self.createMarker(obj) 
                        marker_array.markers += markers 

            self.objectPublish(objects)
            self.markerPublish(marker_array)
            rate.sleep()
        return

    def load(self, yaml_path):
        '''
        load yaml file
        '''
        with open(yaml_path, 'r') as stream:
            try:
                self.config_data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return False
        return True

    def objectPublish(self, objects):
        self.pub_objs.publish(objects)        
        return True

    def markerPublish(self, markers):
        self.pub_marker.publish(markers)
        return

    def getBoundingBox(self, trans, rot, size, parent_frame='base_footprint', child_frame=None):
        '''
        get transform between robotbase and object
        '''
        trans_vec = [trans['x'], trans['y'], trans['z']]
        rot_vec = [rot['x'], rot['y'], rot['z'], rot['w']]
        
        if child_frame:
            # base_footprint to marker frame
            pos,ori = self.listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))

            # base_footprint to object center
            trans1_mat = tf.transformations.translation_matrix(pos)
            rot1_mat   = tf.transformations.quaternion_matrix(ori)
            mat1 = numpy.dot(trans1_mat, rot1_mat)

            trans2_mat = tf.transformations.translation_matrix([trans['x'], trans['y'], trans['z']])
            rot2_mat = tf.transformations.quaternion_matrix([rot['x'], rot['y'], rot['z'], rot['w']])
            mat2 = numpy.dot(trans2_mat, rot2_mat)

            mat3 = numpy.dot(mat1, mat2)
            trans_vec = tf.transformations.translation_from_matrix(mat3) 
            rot_vec = tf.transformations.quaternion_from_matrix(mat3)
        
        bb3d = BoundingBox3D()
        bb3d.center.position.x = trans_vec[0]
        bb3d.center.position.y = trans_vec[1] 
        bb3d.center.position.z = trans_vec[2] 
        bb3d.center.orientation.x = rot_vec[0]
        bb3d.center.orientation.y = rot_vec[1]
        bb3d.center.orientation.z = rot_vec[2]
        bb3d.center.orientation.w = rot_vec[3]
        bb3d.size.x = size['x']
        bb3d.size.y = size['y']
        bb3d.size.z = size['z']
        return bb3d

    def createObjectMsg(self, object_info, parent_frame='base_footprint', child_frame=None):
        '''
        create Object.msg
        '''
        name =  object_info['name']
        size = object_info['size']
        trans = object_info['translation']
        rot = object_info['rotation']
        aff = None

        obj = Object()
        obj.header.frame_id = parent_frame
        obj.id = name
        obj.type = 'dynamic'
        obj.bb3d = self.getBoundingBox(trans, rot, size, parent_frame, child_frame)
        
        if 'affordance' in object_info:
            for affordance in object_info['affordance']:
                aff_name =  affordance['name']
                aff_size = affordance['size']
                aff_trans = affordance['translation']
                aff_rot = affordance['rotation']

                aff = Affordance()
                aff.header.frame_id = parent_frame
                aff.id = aff_name

                # object center to affordance
                T_obj = pose_2_mat(trans, rot)
                T_aff = pose_2_mat(aff_trans, aff_rot)                
                pose_trans, pose_rot = mat_2_pose(np.dot(T_obj, T_aff))
                #
                aff.bb3d = self.getBoundingBox(pose_trans, pose_rot, aff_size, parent_frame, child_frame)
                obj.affordance.append(aff)

        return obj

    def createObject(self, trans, rot, size, name, object_frame):
        '''
        publish object information as socialrobot_perception_msgs/Object format
        '''
        pos,ori = self.listener.lookupTransform(self.base_frame, object_frame, rospy.Time(0))

        trans1_mat = tf.transformations.translation_matrix(pos)
        rot1_mat   = tf.transformations.quaternion_matrix(ori)
        mat1 = numpy.dot(trans1_mat, rot1_mat)

        trans2_mat = tf.transformations.translation_matrix([trans['x'], trans['y'], trans['z']])
        rot2_mat = tf.transformations.quaternion_matrix([rot['x'], rot['y'], rot['z'], rot['w']])
        mat2 = numpy.dot(trans2_mat, rot2_mat)

        mat3 = numpy.dot(mat1, mat2)
        trans3 = tf.transformations.translation_from_matrix(mat3) 
        rot3 = tf.transformations.quaternion_from_matrix(mat3)

        obj = socialrobot_perception_msgs.msg.Object()
        obj.name.data = name       
        bb3d = BoundingBox3D()
        bb3d.center.position.x = trans3[0]
        bb3d.center.position.y = trans3[1] 
        bb3d.center.position.z = trans3[2] 
        bb3d.center.orientation.x = rot3[0]
        bb3d.center.orientation.y = rot3[1]
        bb3d.center.orientation.z = rot3[2]
        bb3d.center.orientation.w = rot3[3]
        bb3d.size.x = size['x']
        bb3d.size.y = size['y']
        bb3d.size.z = size['z']
        obj.bb3d = bb3d  

        return obj

    def createMarker(self, objectMsg):
        marker_array = []
        # object base
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = objectMsg.header.frame_id
        marker.ns = objectMsg.id
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.MODIFY
        marker.pose.position = objectMsg.bb3d.center.position        
        marker.pose.orientation = objectMsg.bb3d.center.orientation
        marker.scale = objectMsg.bb3d.size
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.4
        marker.lifetime = rospy.Duration(0.7)
        marker_array.append(marker)

        # object affordance
        for i, aff in enumerate(objectMsg.affordance):
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = objectMsg.header.frame_id
            marker.ns = objectMsg.id
            marker.id = i+1
            marker.type = Marker.CUBE
            marker.action = Marker.MODIFY
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.6
            marker.scale = aff.bb3d.size
            marker.lifetime = rospy.Duration(0.7)
            marker.pose = aff.bb3d.center
            
            # # transform robot->object->affordance 
            # trans1_mat = tf.transformations.translation_matrix([objectMsg.bb3d.center.position.x, objectMsg.bb3d.center.position.y, objectMsg.bb3d.center.position.z])
            # rot1_mat   = tf.transformations.quaternion_matrix([objectMsg.bb3d.center.orientation.x, objectMsg.bb3d.center.orientation.y, objectMsg.bb3d.center.orientation.z, objectMsg.bb3d.center.orientation.w])
            # mat1 = numpy.dot(trans1_mat, rot1_mat)

            # trans2_mat = tf.transformations.translation_matrix([aff.bb3d.center.position.x, aff.bb3d.center.position.y, aff.bb3d.center.position.z])
            # rot2_mat = tf.transformations.quaternion_matrix([aff.bb3d.center.orientation.x, aff.bb3d.center.orientation.y, aff.bb3d.center.orientation.z, aff.bb3d.center.orientation.w])
            # mat2 = numpy.dot(trans2_mat, rot2_mat)

            # mat3 = numpy.dot(mat1, mat2)
            # trans_vec = tf.transformations.translation_from_matrix(mat3) 
            # rot_vec = tf.transformations.quaternion_from_matrix(mat3)

            # marker.pose.position.x = trans_vec[0]
            # marker.pose.position.y = trans_vec[1]
            # marker.pose.position.z = trans_vec[2]

            # marker.pose.orientation.x = rot_vec[0]
            # marker.pose.orientation.y = rot_vec[1]
            # marker.pose.orientation.z = rot_vec[2]
            # marker.pose.orientation.w = rot_vec[3]
            marker_array.append(marker)            
        return marker_array

if __name__ =='__main__':
    # Initialize ROS node
    rospy.init_node('qr_to_object', anonymous=True)   
    r = rospkg.RosPack()
    pkg_path = r.get_path('qr_tracker')
    yaml_path = pkg_path + '/cfg/objects.yaml'
    oc = ObjectCreator(yaml_path)

    try:
        oc.run()
    except KeyboardInterrupt:
        print("Shutting down")
