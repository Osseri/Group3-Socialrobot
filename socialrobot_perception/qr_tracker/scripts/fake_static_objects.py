#!/usr/bin/env python
import rospy
import rospkg
import rosparam
import tf
import tf.transformations as tfm
import yaml
import sys
import math
from math import pi
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import BoundingBox3D
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32MultiArray, Int32
from socialrobot_msgs.msg import Object, Objects, Affordance

rot_x = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (1, 0, 0))
rot_y = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 1, 0))
rot_z = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 0, 1))

def dot(a, b):
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w

def close_quaternion(anchor, query):
    if dot(anchor, query) < 0.0:  # not close
        query.x *= -1.0
        query.y *= -1.0
        query.z *= -1.0
        query.w *= -1.0
    # close
    return query

def normalize_quaternion(orientation):        
    length = np.sqrt(
        orientation.x * orientation.x +
        orientation.y * orientation.y +
        orientation.z * orientation.z +
        orientation.w * orientation.w
    )
    n = 1.0 / length
    orientation.x *= n
    orientation.y *= n
    orientation.z *= n
    orientation.w *= n
    return orientation


def weighted_average_quat(prev, current, ratio_of_current):
    """orientation type"""
    new_o = close_quaternion(prev, current)
    prev.x += ratio_of_current * new_o.x
    prev.y += ratio_of_current * new_o.y
    prev.z += ratio_of_current * new_o.z
    prev.w += ratio_of_current * new_o.w
    return normalize_quaternion(prev)


def weighted_average_quat_strict(prev, current, alpha, beta):
    """orientation type"""
    new_o = close_quaternion(prev, current)
    prev.x = alpha * prev.x + beta * new_o.x
    prev.y = alpha * prev.y + beta * new_o.y
    prev.z = alpha * prev.z + beta * new_o.z
    prev.w = alpha * prev.w + beta * new_o.w
    return normalize_quaternion(prev)

def pose_2_mat(trans, rot):
    if str(type(trans)) == "<type 'dict'>":
        trans_vec = [trans['x'], trans['y'], trans['z']]
        rot_vec = [rot['x'], rot['y'], rot['z'], rot['w']]
    else:
        trans_vec = trans
        rot_vec =rot

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

def pose_to_mat(pose):
    trans_vec = [pose.position.x, pose.position.y, pose.position.z]
    rot_vec = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    trans_mat = tf.transformations.translation_matrix(trans_vec)
    rot_mat = tf.transformations.quaternion_matrix(rot_vec)
    T = np.dot(trans_mat, rot_mat) 
    return T

def mat_to_pose(T):
    trans = tf.transformations.translation_from_matrix(T) 
    rot = tf.transformations.quaternion_from_matrix(T)

    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    return pose

def multiply_pose(pose1, pose2):
    mat1 = pose_to_mat(pose1)
    mat2 = pose_to_mat(pose2)
    mat3 = np.dot(mat1, mat2)
    return mat_to_pose(mat3)

def transform_pose(pose, parent_pose):
    '''
    transform pose based on parent_pose 
    '''
    return multiply_pose(parent_pose, pose)

def create_pose(trans, rot):
    '''
    create ROS pose msg from translation and quaternion vector
    '''
    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    return pose

def euler_to_quaternion(euler):
    return tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])

class ObjectCreator():

    def __init__(self, config_path):

        self.fridge_frame = 'QR:51'
        self.door_frame = 'QR:48'
        self.table_frame = 'QR:47'
        self.robot_frame = 'base_footprint'
        self.camera_frame = 'cam_e_link'
        self.map_frame = 'map'

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.config_path = config_path
        self.config_data = []
        self.dynamic_objects = []
        self.static_objects = []
        self.qr_tf = {}
        self.perception_objects = []
        self.dynamic_obj = {}
        self.stop_static_obj_publish = False
        self.door_isopen = True
        self.perception_updating = False
        rospy.set_param('fridge_isopen', False)
        rospy.set_param('user_age', 'old')
  
        self.pub_objs = rospy.Publisher("/socialrobot/qr_tracker/objects", Objects, queue_size=10)
        self.pub_marker = rospy.Publisher("/socialrobot/visualization/markers", MarkerArray, queue_size=10)   
        self.pub_grasp = rospy.Publisher("/socialrobot/visualization/grasp_points", PoseArray, queue_size=10) 
        self.sub_qr = rospy.Subscriber("/aruco_detector/detected_markers", Int32MultiArray, self.qr_callback)
        #self.sub_qr = rospy.Subscriber("/socialrobot/perception/objects", Objects, self.perception_callback)

        self.create_static_tf() 
        self.dynamic_objects = self.get_dynamic_object_msg()
    
    def qr_callback(self, arr):
        if int(self.door_frame[3:]) in arr.data:
            self.door_isopen = False
        else:
            self.door_isopen = True

    def perception_callback(self, objects):
        self.perception_objects = []
        self.perception_updating = True
        for obj in objects.detected_objects:
            if obj.id == 'obj_courier_box':
                bb3d = BoundingBox3D()
                bb3d.center = obj.bb3d.center
                bb3d.center.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                bb3d.size.x = 5.0000e-02
                bb3d.size.y = 1.2500e-01
                bb3d.size.z = 2.6500e-01

                box = Object()
                box.header.frame_id = 'base_footprint'
                box.bb3d = bb3d
                box.type = 'dynamic'
                box.id = obj.id
                self.perception_objects.append(box)           

        self.perception_updating = False

    def get_dynamic_object_msg(self):
        ''''''
        object_list = []
        self.dynamic_obj = {
                            'obj_white_gotica': self.creaet_boundingbox([+1.6061e+00, +4.9503e-01, +8.6870e-01],
                                                                [0,0,0.395,0.919],
                                                                [0.0649999976158, 0.0649999976158, 0.159999996424]),
                            'obj_red_gotica': self.creaet_boundingbox([+1.6453e+00, +3.9626e-01, +8.6870e-01],
                                                                [0,0,0.395,0.919],
                                                                [0.0630000010133, 0.0630000010133, 0.159999996424]),
                            'obj_tray': self.creaet_boundingbox([+5.6333e-02, -1.4268e-01, +7.4471e-01],
                                                                [0,0, 0.941, 0.340],
                                                                [2.3500e-01, 0.390000164509, 2.0000e-02]),
                            'obj_human': self.creaet_boundingbox([-1.2500e-01, -1.4500e+00, +1.1776e+00],
                                                                [0,0,0.423,0.906],
                                                                [2.5647e-01, 5.0394e-01, 5.9013e-01]),
                            # 'obj_courier_box': self.creaet_boundingbox([+1.2793e-01, -2.0205e-01, +8.6732e-01],
                            #                                     [0,0, 0.941, 0.339],
                            #                                     [5.0000e-02, 1.2500e-01, 2.6500e-01]),
                            # 'obj_human': self.creaet_boundingbox([+1.3511e+00, -1.3197e+00, +1.1776e+00],
                            #                                     [0,0,0.947,0.321],
                            #                                     [2.5647e-01, 5.0394e-01, 5.9013e-01]),
                            # 'obj_wall_1': self.creaet_boundingbox([+7.0000e-01, +8.4250e-01, +8.0000e-01],
                            #                                     [0,0,0.309,0.951],
                            #                                     [4.0009e-01, 1.4800e+00, 1.6000e+00]),
                            # 'obj_wall_2': self.creaet_boundingbox([-5.0000e-01, +8.6750e-01, +8.0000e-01],
                            #                                     [0,0,0.899,0.438],
                            #                                     [4.0009e-01, 1.4800e+00, 1.6000e+00]),
                            }

        obj_list = self.dynamic_obj.keys()
        for id in obj_list:
            obj = Object()
            obj.header.frame_id = self.map_frame
            obj.id = id
            obj.type = 'dynamic'
            if obj.id in ['obj_tray','obj_human']:
                obj.type = 'static'
            obj.bb3d = self.dynamic_obj[id]
            object_list.append(obj)

        return object_list

    def create_static_tf(self):
        '''create qf pose base on /map frame'''

        self.qr_tf = {
                        self.fridge_frame: create_pose([1.55667, 0.632887, +1.3558e+00],
                                                    [-0.279098, 0.649696, 0.649696, -0.279098]),
                        self.table_frame: create_pose([0.535159, 0.2621, 0.730014],
                                                    [0.0, 0.0, 0.425246, 0.905078]),
                        #door open/close
                        # self.door_frame: create_pose([0.555274, -0.149054, 1.37381],
                        #                             [-0.206207, 0.676378, 0.676371, -0.20619])
                    }        

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

    def get_tf(self, from_tf, to_tf):
        pos = ori = None        
        try:
            self.listener.waitForTransform(from_tf, to_tf, rospy.Time(), rospy.Duration(1.0))
            pos,ori = self.listener.lookupTransform(from_tf, to_tf, rospy.Time(0))
            #rospy.loginfo("found transform between %s and %s." %(from_tf, to_tf))
        except:
            rospy.logerr("cannot find transform between %s and %s." %(from_tf, to_tf))

        return pos, ori

    def transform_obj_msg(self, object_list, transform_mat):
        transformed = []

        for msg in object_list:                    
            mat_from_to = transform_mat            
            mat_from_obj = pose_to_mat(msg.bb3d.center)
            mat_to_obj = np.dot(np.linalg.inv(mat_from_to), mat_from_obj)
            msg.bb3d.center = mat_to_pose(mat_to_obj)
            msg.header.frame_id = self.robot_frame  
            transformed.append(msg)

        return transformed

    def objectPublish(self, objects):    
        self.pub_objs.publish(objects)        
        return True

    def markerPublish(self, marker_array, pose_array):
        self.pub_marker.publish(marker_array)
        #self.pub_grasp.publish(pose_array)
        return

    def getPose(self, trans_vec, rot_vec, parent_pose=None):
        '''
        get grasp direction pose 
        '''

        if parent_pose != None:
            # base_footprint to object
            pos = [parent_pose.position.x, 
                    parent_pose.position.y,
                    parent_pose.position.z]
            ori = [parent_pose.orientation.x, 
                    parent_pose.orientation.y,
                    parent_pose.orientation.z,
                    parent_pose.orientation.w]

            # base_footprint to grasp pose
            trans1_mat = tf.transformations.translation_matrix(pos)
            rot1_mat   = tf.transformations.quaternion_matrix(ori)
            mat1 = np.dot(trans1_mat, rot1_mat)

            trans2_mat = tf.transformations.translation_matrix(trans_vec)
            rot2_mat = tf.transformations.quaternion_matrix(rot_vec)
            mat2 = np.dot(trans2_mat, rot2_mat)

            mat3 = np.dot(mat1, mat2)
            trans_vec = tf.transformations.translation_from_matrix(mat3) 
            rot_vec = tf.transformations.quaternion_from_matrix(mat3)

        pose = Pose()
        pose.position.x = trans_vec[0]
        pose.position.y = trans_vec[1] 
        pose.position.z = trans_vec[2] 
        pose.orientation.x = rot_vec[0]
        pose.orientation.y = rot_vec[1]
        pose.orientation.z = rot_vec[2]
        pose.orientation.w = rot_vec[3]
        return pose

    def creaet_boundingbox(self, trans_vec, rot_vec, size, base_pose=None):
        '''create bounding box ROS msg'''
        if base_pose:            
            # center pose to matrix
            base_mat = pose_to_mat(base_pose)

            transform_mat = np.dot(tf.transformations.translation_matrix(trans_vec), 
                            tf.transformations.quaternion_matrix(rot_vec))

            bb3d_mat = np.dot(base_mat, transform_mat)
            trans_vec = tf.transformations.translation_from_matrix(bb3d_mat) 
            rot_vec = tf.transformations.quaternion_from_matrix(bb3d_mat)        

        bb3d = BoundingBox3D()
        bb3d.center.position.x = trans_vec[0]
        bb3d.center.position.y = trans_vec[1] 
        bb3d.center.position.z = trans_vec[2] 
        bb3d.center.orientation.x = rot_vec[0]
        bb3d.center.orientation.y = rot_vec[1]
        bb3d.center.orientation.z = rot_vec[2]
        bb3d.center.orientation.w = rot_vec[3]
        bb3d.size.x = size[0]
        bb3d.size.y = size[1]
        bb3d.size.z = size[2]
        return bb3d

    def create_obj_msg(self, object_info, object_base, parent_frame='map', child_frame=None):
        '''
        create Object.msg
        '''
        name =  object_info['name']
        size = object_info['size']
        trans = object_info['translation']
        rot = object_info['rotation']
        trans_vec = [trans['x'], trans['y'], trans['z']]
        rot_vec = [rot['x'], rot['y'], rot['z'], rot['w']]
        size_vec = [size['x'], size['y'], size['z']]
        aff = None

        object_list = []

        obj = Object()
        obj.header.frame_id = parent_frame
        obj.id = name
        obj.type = 'dynamic'
        obj.bb3d = self.creaet_boundingbox(trans_vec, rot_vec, size_vec, object_base)
        object_list.append(obj)

        # if object has grasp point
        if object_info.has_key('grasp_point'):
            gr_pts = object_info['grasp_point']
            for pt in gr_pts:
                gr_trans = pt['translation']
                gr_rot = pt['rotation']
                obj.grasp_point.append(self.getPose(gr_trans, gr_rot))

        # if object has affordance 
        if object_info.has_key('affordance'):
            for affordance in object_info['affordance']:
                aff = Affordance()
                aff.header.frame_id = obj.id                
                aff.id =  affordance['name']

                aff_size = affordance['size']
                aff_trans = affordance['translation']
                aff_rot = affordance['rotation']     
                aff_trans_vec = [aff_trans['x'], aff_trans['y'], aff_trans['z']]
                aff_rot_vec = [aff_rot['x'], aff_rot['y'], aff_rot['z'], aff_rot['w']]
                aff_size_vec = [aff_size['x'], aff_size['y'], aff_size['z']]      
                aff.bb3d = self.creaet_boundingbox(aff_trans_vec, aff_rot_vec, aff_size_vec)

                if affordance.has_key('grasp_point'):
                    aff_gr_pts = affordance['grasp_point']
                    for pt in aff_gr_pts:
                        gr_trans = pt['translation']
                        gr_rot = pt['rotation']
                        gr_trans_vec = [gr_trans['x'], gr_trans['y'], gr_trans['z']]
                        gr_rot_vec = [gr_rot['x'], gr_rot['y'], gr_rot['z'], gr_rot['w']]
                        aff.grasp_point.append(self.getPose(gr_trans_vec, gr_rot_vec))
                obj.affordance.append(aff)

                # create indivisual object instance
                if 'obj' in aff.id:
                    obj_aff = Object()
                    obj_aff.header.frame_id = parent_frame
                    obj_aff.id = aff.id
                    obj_aff.type = 'static'
                    # transform 
                    obj_aff.bb3d.size = aff.bb3d.size
                    obj_aff.bb3d.center = transform_pose(aff.bb3d.center, obj.bb3d.center)
                    object_list.append(obj_aff)

        return object_list

    def createMarkers(self, objectList, frame_id='base_footprint'):
        '''convert object msg to marker msg'''
        marker_array = MarkerArray()
        pose_array = PoseArray()

        for i, objectMsg in enumerate(objectList):
            T_map_to_obj = pose_to_mat(objectMsg.bb3d.center)
            
            # object base
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = frame_id
            marker.ns = 'obj'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.MODIFY

            marker.pose = objectMsg.bb3d.center
            marker.scale = objectMsg.bb3d.size

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.4
            marker.lifetime = rospy.Duration(0.7)
            marker_array.markers.append(marker)

            for gr_pt in objectMsg.grasp_point:
                T_obj_to_pt = pose_2_mat(gr_pt.position, gr_pt.orientation)                
                pose = mat_to_pose(np.dot(T_map_to_obj, T_obj_to_pt))                    
                pose_array.poses.append(pose)
   
        return marker_array, pose_array

    def rotate_door(self, object_list):        
        
        for i,obj in enumerate(object_list):
            # transform only door object 
            if 'door' in obj.id:
                closed_door_pose = obj.bb3d.center
                opened_joint_pose = transform_pose(create_pose([-4.4977e-02, +2.5000e-01, 0.0], rot_z(120)), closed_door_pose)
                opened_door_pose = transform_pose(create_pose([4.4977e-02, -2.5000e-01, 0.0], [0,0,0,1]), opened_joint_pose)                
                object_list[i].bb3d.center = opened_door_pose

            elif 'handle' in obj.id:
                closed_handle_pose = obj.bb3d.center
                opened_joint_pose = transform_pose(create_pose([-9.8475e-02, +4.5000e-01, 0.0], rot_z(120)), closed_handle_pose)
                opened_handle_pose = transform_pose(create_pose([9.8475e-02, -4.5000e-01, 0.0], [0,0,0,1]), opened_joint_pose)                
                object_list[i].bb3d.center = opened_handle_pose


    def publish_tf(self, tf_dict):
        tf_list = tf_dict.keys()
        for id in tf_list:
            tf_pose = tf_dict[id]
            pos = tf_pose.position
            ori = tf_pose.orientation
            self.broadcaster.sendTransform((pos.x, pos.y, pos.z),
                                            (ori.x, ori.y, ori.z, ori.w),
                                            rospy.Time.now(),
                                            id,
                                            self.map_frame)

        
    def run(self):
        # load transform data from qr to objects
        data = self.load(self.config_path)
        if not data:
            return

        br = tf.TransformBroadcaster()
        rate = rospy.Rate(30.0)
        
        # get map tf      
        pos = ori = None                     
        pos, ori = self.get_tf(self.map_frame, self.robot_frame)

        # if map found
        if pos != None and ori != None:
            pass
        else:
            self.map_frame = "/odom"
            pos, ori = self.get_tf(self.map_frame, self.robot_frame)

            if pos == None or ori == None:
                return

        while not rospy.is_shutdown(): 
            # publish qr marker tf
            self.publish_tf(self.qr_tf)
            
            # get transform between map to robot
            pos = ori = mat_map_to_robot = None
            pos, ori = self.get_tf(self.map_frame, self.robot_frame)
            if pos != None and ori != None:
                mat_map_to_robot = pose_2_mat(pos, ori)   

            # get objects info from DB
            objects = []
            for i, obj in enumerate(self.config_data['objects']):
                marker_frame = 'QR:'+str(obj['id'])
                object_name = obj['name']

                if self.qr_tf.has_key(marker_frame):    
                    qr_pose = self.qr_tf[marker_frame]
                    object_list = self.create_obj_msg(obj, qr_pose)

                    # exception for fridge
                    if object_name == 'obj_fridge':         
                        if rospy.get_param('fridge_isopen'):
                            self.door_isopen = True                                    
                            self.rotate_door(object_list)
                        else: 
                            self.door_isopen = False
                    
                    objects += object_list

            # add dynamic objects
            if self.door_isopen:
                dyn_obj = self.get_dynamic_object_msg()
                for obj in dyn_obj:
                    if obj.type == 'static':
                        objects.append(obj)
            else:
                objects+=self.get_dynamic_object_msg()

            # publish markers 
            cuboid_markers, pose_markers = self.createMarkers(objects, 'odom') 
            self.markerPublish(cuboid_markers, pose_markers)    

            # transform object frame to robot frame
            objects = self.transform_obj_msg(objects, mat_map_to_robot)


            # create marker for base object
            if len(objects)>0:   
                self.objectPublish(objects)    

            rate.sleep()
        return

if __name__ =='__main__':
    # Initialize ROS node
    rospy.init_node('fridge_detector', anonymous=True)
    rospy.loginfo('[fake_map] publishing...')
    r = rospkg.RosPack()
    pkg_path = r.get_path('qr_tracker')
    yaml_path = pkg_path + '/cfg/objects.yaml'
    oc = ObjectCreator(yaml_path)

    try:
        oc.run()
    except KeyboardInterrupt:
        print("Shutting down")
