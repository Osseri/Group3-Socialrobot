#!/usr/bin/env python
import rospy
import rospkg
import rosparam
import tf
import yaml
import sys
from math import pi
import numpy
from visualization_msgs.msg import Marker
from vision_msgs.msg import BoundingBox3D
import geometry_msgs.msg
import socialrobot_interface.msg
import moveit_msgs.msg
import moveit_commander

class ObjectCreator():

    def __init__(self, config_path):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()    

        self.listener = tf.TransformListener()
        self.config_path = config_path
        self.config_data = []
        self.camera_frame = "/camera_rgb_frame"
        self.base_frame = "/base_footprint"
        self.pub_marker = rospy.Publisher("/aruco_tracker/markers", Marker, queue_size=10)    
        self.pub_objects = rospy.Publisher("/perception/objects", socialrobot_interface.msg.Objects, queue_size=10)

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
                    object_frame = 'QR:'+str(obj['id'])
                    (trans,rot) = self.listener.lookupTransform(object_frame, self.camera_frame, rospy.Time(0))
                    if trans:                        
                        name =  obj['name']
                        size = obj['size']
                        id = obj['id']
                        trans = obj['translation']
                        rot = obj['rotation']

                        # create marker for Rviz
                        marker = self.createMarker(trans, rot, size, id, object_frame) 
                        markers.append(marker)  

                        # create object for perception module (dummy)
                        obj = self.createObject(trans, rot, size, name, object_frame)
                        objects.append(obj)
                except:
                    pass

            # detect table
            try:
                left_trans, left_rot = self.listener.lookupTransform('QR:1', self.camera_frame, rospy.Time(0))
                right_trans, right_rot = self.listener.lookupTransform('QR:3', self.camera_frame, rospy.Time(0))
                if left_trans and right_trans:
                    name =  'obj_table'
                    size = {'x': 1.2, 'y':0.6, 'z':0.62}
                    id = 1
                    center_pt = [right_trans[0]-left_trans[0],
                                right_trans[1]-left_trans[1],
                                right_trans[2]-left_trans[2]]

                    # trans = {}
                    # rot = {}

                    # marker = self.createMarker(trans, rot, size, id) 
                    # markers.append(marker)  
            except:
                    pass

            self.objectPublish(objects)
            self.markerPublish(markers)
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
        objs = socialrobot_interface.msg.Objects()

        #TODO: object position from base_footprint
        for obj in objects:             
            objs.detected_objects.append(obj)   
        self.pub_objects.publish(objs)
        
        return True

    def createObject(self, trans, rot, size, name):

        obj = socialrobot_interface.msg.Object()

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

        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = self.base_frame
        object_pose.pose.orientation.x = rot3[0]
        object_pose.pose.orientation.y = rot3[1]
        object_pose.pose.orientation.z = rot3[2]
        object_pose.pose.orientation.w = rot3[3]
        object_pose.pose.position.x = trans3[0]
        object_pose.pose.position.y = trans3[1] 
        object_pose.pose.position.z = trans3[2] 
        size_tuple = (size['x'], size['y'], size['z'])
        #self.add_box(object_pose, name, size_tuple)

        obj = socialrobot_interface.msg.Object()
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

    def createMarker(self, trans, rot, size, id):
        marker = Marker()
        marker.header.frame_id = self.camera_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obj"
        marker.id = id
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
        marker.color.a = 0.3
        marker.lifetime.secs = 0.1

        return marker


    def createMarker(self, trans, rot, size, id, object_frame):
        marker = Marker()
        marker.header.frame_id = object_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obj"
        marker.id = id
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
        marker.color.a = 0.3
        marker.lifetime.secs = 0.1
        return marker

    def markerPublish(self, markers):
        for marker in markers:
            self.pub_marker.publish(marker)
        return


    def add_box(self, object_pose, object_name, size):
        scene = self.scene    
        size_tuple = size

        try:
            scene.add_box(object_name, object_pose, size = size_tuple)
        except:
            return -1

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