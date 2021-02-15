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
from visualization_msgs.msg import Marker
from vision_msgs.msg import BoundingBox3D
import geometry_msgs.msg
import socialrobot_perception_msgs.msg
import numpy as np

class ObjectCreator():

    def __init__(self, config_path):

        self.listener = tf.TransformListener()
        self.config_path = config_path
        self.config_data = []
        self.camera_frame = "/cam_e_color_optical_frame"
        self.base_frame = "/base_footprint"
        self.pub_marker = rospy.Publisher("/aruco_tracker/markers", Marker, queue_size=10)    
        self.pub_objects = rospy.Publisher("/perception/objects", socialrobot_perception_msgs.msg.Objects, queue_size=10)
        self.objects = []

    def run(self):
        # load yaml
        data = self.load(self.config_path)
        if not data:
            return

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            frames = self.listener.allFramesAsString()
            markers = []
            objects = []

            # detect objects
            for obj in self.config_data['objects']:
                try:
                    # add table object
                    if obj['name'] == 'obj_table':
                        name =  obj['name']
                        size = obj['size']
                        id = obj['id']
                        trans = obj['translation']
                        rot = obj['rotation']
                        marker = self.createMarker(trans, rot, size, id, 'base_footprint') 
                        markers.append(marker) 
                        obj = self.createObject(trans, rot, size, name, 'base_footprint')
                        objects.append(obj) 
                    else:
                        object_frame = 'QR:'+str(obj['id'])
                        (trans,rot) = self.listener.lookupTransform(object_frame, 'base_footprint', rospy.Time(0))
                        if trans:                    
                            name =  obj['name']
                            size = obj['size']
                            id = obj['id']
                            trans = obj['translation']
                            rot = obj['rotation']

                            # create marker for Rviz
                            marker = self.createMarker(trans, rot, size, id, object_frame) 
                            markers.append(marker)  

                            # create object as perception module
                            obj = self.createObject(trans, rot, size, name, object_frame)
                            objects.append(obj)

                except:
                    pass

            self.objectPublish(objects)
            self.markerPublish(markers)
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
        objs = socialrobot_perception_msgs.msg.Objects()

        #TODO: object position from base_footprint
        for obj in objects:             
            obj.bb3d.center.position.z += 0.05
            objs.detected_objects.append(obj)   

        if len(objs.detected_objects)>0:
            self.pub_objects.publish(objs)
        
        return True

    def createObject(self, trans, rot, size, name):

        obj = socialrobot_perception_msgs.msg.Object()

        return obj

    def createObject(self, trans, rot, size, name, object_frame):

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

    def createMarker(self, trans, rot, size, id, object_frame=None):
        marker = Marker()
        if object_frame:
            marker.header.frame_id = object_frame
        else:
            marker.header.frame_id = self.camera_frarme
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obj"
        marker.id = int(id)
        marker.type = Marker.CUBE
        marker.action = Marker.MODIFY
        marker.pose.position.x = trans['x']
        marker.pose.position.y = trans['y']
        marker.pose.position.z = trans['z']
        marker.pose.orientation.x = rot['x']
        marker.pose.orientation.y = rot['y']
        marker.pose.orientation.z = rot['z']
        marker.pose.orientation.w = rot['w']
        marker.scale.x = size['x']
        marker.scale.y = size['y']
        marker.scale.z = size['z']
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime = rospy.Duration(0.7)
        return marker

    def markerPublish(self, markers):
        for marker in markers:
            self.pub_marker.publish(marker)
        return


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