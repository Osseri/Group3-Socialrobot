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
        trans_vec = [trans.x, trans.y, trans.z]
        rot_vec = [rot.x, rot.y, rot.z, rot.w]

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



class FridgeCreator:

    class FridgeInstance:
        def __init__(self):
            self.has_new = False
            self.data = Pose()

        def set_data(self, qr_pose, qr_to_center):
            self.data.position.x = qr_pose.position.x + qr_to_center.x
            self.data.position.y = qr_pose.position.y + qr_to_center.y
            self.data.position.z = qr_pose.position.z + qr_to_center.z
            self.data.orientation = qr_pose.orientation
    """
    | tl tr   ^ y
    | bl br   |->  x
    """
    qr_to_center = {
        "fridge_tl": Point(0.218, -0.1, 0.0),
        "fridge_tr": Point(-0.218, -0.1, 0.0),
        "fridge_bl": Point(0.218, 0.1, 0.0),
        "fridge_br": Point(-0.218, 0.1, 0.0),
    }
    def __init__(self, qr_list):
        self.fridge_info = None
        self.num_marker = len(self.qr_to_center)
        self.qr_list = {
            qr_list[0]: "fridge_tl",
            qr_list[1]: "fridge_tr",
            qr_list[2]: "fridge_bl",
            qr_list[3]: "fridge_br"
        }
        self.data = {
            "fridge_tl": self.FridgeInstance(),
            "fridge_tr": self.FridgeInstance(),
            "fridge_bl": self.FridgeInstance(),
            "fridge_br": self.FridgeInstance()
        }
        # fridge bbox3d
        self.prev_box = BoundingBox3D()

    def mark_flag(self, qr_id, qr_pose):
        if(self.prev_box == BoundingBox3D()):
            self.prev_box.center = qr_pose

        self.data[self.qr_list[qr_id]].has_new = True
        self.data[self.qr_list[qr_id]].set_data(qr_pose, self.qr_to_center[self.qr_list[qr_id]])

    def _update(self):
        count = 0
        point = Point(0.0, 0.0, 0.0)
        # higher pass
        idx = 0
        scores = [0.0] * self.num_marker
        closes = [Quaternion(0.0, 0.0, 0.0, 1.0)] * self.num_marker
        for name, table_inst in self.data.items():
            if table_inst.has_new:
                count += 1
                # position
                point.x += table_inst.data.position.x 
                point.y += table_inst.data.position.y
                point.z += table_inst.data.position.z
                # orientation
                close = close_quaternion(
                    self.prev_box.center.orientation,
                    table_inst.data.orientation,
                )
                closes[idx] = close
                scores[idx] = dot(close, self.prev_box.center.orientation)
            idx += 1

        # threshold
        scores = np.array(scores)
        threshold = np.cos(np.radians(45))
        th_pass = scores[scores > threshold]
        mean = np.mean(th_pass)

        qr_filter = scores >= mean
        is_quat_update = True in qr_filter

        naive_orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
        for idx in xrange(self.num_marker):
            if qr_filter[idx]:
                dot_score = scores[idx]
                close = closes[idx]
                naive_orientation.x += close.x * dot_score
                naive_orientation.y += close.y * dot_score
                naive_orientation.z += close.z * dot_score
                naive_orientation.w += close.w * dot_score

        if count > 0:
            # avg
            c = float(count)
            point.x = point.x / c
            point.y = point.y / c
            point.z = point.z / c
            if is_quat_update:
                unit_orientation = normalize_quaternion(naive_orientation)

            prev = self.prev_box.center
            # rospy.loginfo(self.prev_box.center.position)
            if prev.position.z > 0.4:
                # position
                gain = 0.1
                self.prev_box.center.position.x = gain * point.x + (1.0 - gain) * prev.position.x
                self.prev_box.center.position.y = gain * point.y + (1.0 - gain) * prev.position.y
                self.prev_box.center.position.z = gain * point.z + (1.0 - gain) * prev.position.z
                # orientation
                if is_quat_update:
                    self.prev_box.center.orientation = weighted_average_quat(
                        prev.orientation,
                        unit_orientation,
                        0.05
                    )
            else:
                self.prev_box.center.position = point
                if is_quat_update:
                    self.prev_box.center.orientation = unit_orientation

    def get(self):
        self._update()
        
        return self.prev_box.center
    

class ObjectCreator():

    def __init__(self, config_path):
        self.fridge_qr_list = ["QR:40","QR:45","QR:41","QR:46"] #[tl,tr,bl,br]
        #self.fridge_creator = FridgeCreator(self.fridge_qr_list)

        self.marker_frame = 'QR:51'
        self.door_frame = 'QR:48'
        self.base_frame = '/base_footprint'

        self.listener = tf.TransformListener()
        self.config_path = config_path
        self.config_data = []
        self.objects = []
  
        # self.pub_objects = rospy.Publisher("/perception/objects", socialrobot_perception_msgs.msg.Objects, queue_size=10)
        self.pub_objs = rospy.Publisher("/socialrobot/qr_tracker/objects", Objects, queue_size=10)
        self.pub_marker = rospy.Publisher("/socialrobot/visualization/markers", MarkerArray, queue_size=10)   
        self.pub_grasp = rospy.Publisher("/socialrobot/visualization/grasp_points", PoseArray, queue_size=10)  

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

    def markerPublish(self, marker_array, pose_array):
        self.pub_marker.publish(marker_array)
        self.pub_grasp.publish(pose_array)
        return

    def getPose(self, trans, rot, parent_pose=None):
        '''
        get grasp direction pose 
        '''
        trans_vec = [trans['x'], trans['y'], trans['z']]
        rot_vec = [rot['x'], rot['y'], rot['z'], rot['w']]

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

            trans2_mat = tf.transformations.translation_matrix([trans['x'], trans['y'], trans['z']])
            rot2_mat = tf.transformations.quaternion_matrix([rot['x'], rot['y'], rot['z'], rot['w']])
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

    def getBoundingBox(self, trans, rot, size, parent_frame='base_footprint', child_frame=None):
        '''
        get boundingbox msg
        '''
        trans_vec = [trans['x'], trans['y'], trans['z']]
        rot_vec = [rot['x'], rot['y'], rot['z'], rot['w']]
        
        if parent_frame:
            # base_footprint to marker frame
            pos,ori = self.listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))

            # base_footprint to object center
            trans1_mat = tf.transformations.translation_matrix(pos)
            rot1_mat   = tf.transformations.quaternion_matrix(ori)
            mat1 = np.dot(trans1_mat, rot1_mat)

            trans2_mat = tf.transformations.translation_matrix([trans['x'], trans['y'], trans['z']])
            rot2_mat = tf.transformations.quaternion_matrix([rot['x'], rot['y'], rot['z'], rot['w']])
            mat2 = np.dot(trans2_mat, rot2_mat)

            mat3 = np.dot(mat1, mat2)
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

        object_list = []
        obj = Object()
        obj.header.frame_id = parent_frame
        obj.id = name
        obj.type = 'static'
        obj.bb3d = self.getBoundingBox(trans, rot, size, parent_frame, child_frame)

        if object_info.has_key('grasp_point'):
            gr_pts = object_info['grasp_point']
            for pt in gr_pts:
                gr_trans = pt['translation']
                gr_rot = pt['rotation']
                obj.grasp_point.append(self.getPose(gr_trans, gr_rot))
        
        if object_info.has_key('affordance'):
            for affordance in object_info['affordance']:
                aff = Affordance()
                aff.header.frame_id = obj.id                
                aff.id =  affordance['name']

                aff_size = affordance['size']
                aff_trans = affordance['translation']
                aff_rot = affordance['rotation']           
                aff.bb3d = self.getBoundingBox(aff_trans, aff_rot, aff_size, parent_frame=None)

                if affordance.has_key('grasp_point'):
                    aff_gr_pts = affordance['grasp_point']
                    for pt in aff_gr_pts:
                        gr_trans = pt['translation']
                        gr_rot = pt['rotation']
                        aff.grasp_point.append(self.getPose(gr_trans, gr_rot))
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

        object_list.append(obj)
        return object_list

    def createObject(self, trans, rot, size, name, object_frame):
        '''
        publish object information as socialrobot_perception_msgs/Object format
        '''
        pos,ori = self.listener.lookupTransform(self.base_frame, object_frame, rospy.Time(0))

        trans1_mat = tf.transformations.translation_matrix(pos)
        rot1_mat   = tf.transformations.quaternion_matrix(ori)
        mat1 = np.dot(trans1_mat, rot1_mat)

        trans2_mat = tf.transformations.translation_matrix([trans['x'], trans['y'], trans['z']])
        rot2_mat = tf.transformations.quaternion_matrix([rot['x'], rot['y'], rot['z'], rot['w']])
        mat2 = np.dot(trans2_mat, rot2_mat)

        mat3 = np.dot(mat1, mat2)
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

    def createMarkers(self, objectInfo, objectList):      
        marker_frame = 'QR:'+str(objectInfo['id'])  
        marker_array = MarkerArray()
        pose_array = PoseArray()
        pose_array.header.frame_id = marker_frame

        for objectMsg in objectList:
            if 'obj_fridge_' not in objectMsg.id:
                # object base
                marker = Marker()
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = marker_frame
                marker.ns = objectMsg.id
                marker.id = 0
                marker.type = Marker.CUBE
                marker.action = Marker.MODIFY

                # transpose pose based on marker frame
                T_qr_to_obj = pose_2_mat(objectInfo['translation'], objectInfo['rotation']) 

                marker.pose.position.x = objectInfo['translation']['x']
                marker.pose.position.y = objectInfo['translation']['y']
                marker.pose.position.z = objectInfo['translation']['z']
                marker.pose.orientation.x = objectInfo['rotation']['x']
                marker.pose.orientation.y = objectInfo['rotation']['y']
                marker.pose.orientation.z = objectInfo['rotation']['z']
                marker.pose.orientation.w = objectInfo['rotation']['w']

                marker.scale = objectMsg.bb3d.size
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.4
                marker.lifetime = rospy.Duration(0.7)
                marker_array.markers.append(marker)
                for gr_pt in objectMsg.grasp_point:
                    T_obj_to_pt = pose_2_mat(gr_pt.position, gr_pt.orientation)                
                    pose_trans, pose_rot = mat_2_pose(np.dot(T_qr_to_obj, T_obj_to_pt))
                    pose = Pose()
                    pose.position.x = pose_trans['x'] 
                    pose.position.y = pose_trans['y'] 
                    pose.position.z = pose_trans['z'] 
                    pose.orientation.x = pose_rot['x'] 
                    pose.orientation.y = pose_rot['y'] 
                    pose.orientation.z = pose_rot['z'] 
                    pose.orientation.w = pose_rot['w'] 
                    pose_array.poses.append(pose)

                # object affordance
                for i, aff in enumerate(objectMsg.affordance):
                    marker = Marker()
                    marker.header.stamp = rospy.Time.now()
                    marker.header.frame_id = marker_frame
                    marker.ns = objectMsg.id
                    marker.id = i+1
                    marker.type = Marker.CUBE
                    marker.action = Marker.MODIFY
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.color.a = 0.6
                    marker.scale = aff.bb3d.size
                    marker.lifetime = rospy.Duration(0.7)

                    # affordance based on QR marker
                    T_obj_to_aff = pose_2_mat(aff.bb3d.center.position, aff.bb3d.center.orientation)                
                    aff_trans, aff_rot = mat_2_pose(np.dot(T_qr_to_obj, T_obj_to_aff))
                    marker.pose.position.x = aff_trans['x'] 
                    marker.pose.position.y = aff_trans['y'] 
                    marker.pose.position.z = aff_trans['z'] 
                    marker.pose.orientation.x = aff_rot['x'] 
                    marker.pose.orientation.y = aff_rot['y'] 
                    marker.pose.orientation.z = aff_rot['z'] 
                    marker.pose.orientation.w = aff_rot['w'] 
                    marker_array.markers.append(marker)  

                    for gr_pt in aff.grasp_point:
                        T_qr_to_aff = pose_2_mat(aff_trans, aff_rot)    
                        T_aff_to_grasp = pose_2_mat(gr_pt.position, gr_pt.orientation)              
                        gr_trans, gr_rot = mat_2_pose(np.dot(T_qr_to_aff, T_aff_to_grasp))
                        pose = Pose()
                        pose.position.x = gr_trans['x'] 
                        pose.position.y = gr_trans['y'] 
                        pose.position.z = gr_trans['z'] 
                        pose.orientation.x = gr_rot['x'] 
                        pose.orientation.y = gr_rot['y'] 
                        pose.orientation.z = gr_rot['z'] 
                        pose.orientation.w = gr_rot['w'] 
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

        # set ros param 
        rospy.set_param('fridge_isopen', 'True')
        
    def run(self):
        # load yaml
        data = self.load(self.config_path)
        if not data:
            return

        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        
        while not rospy.is_shutdown():        
            trans = rot = None    
            #frames = self.listener.allFramesAsString()

            # # get Fridge QR pose
            # for qr_id in self.fridge_qr_list:
            #     try:
            #         (trans,quat) = self.listener.lookupTransform(self.base_frame, qr_id, rospy.Time(0))
            #         qr_pose = Pose()
            #         qr_pose.position.x = trans[0]
            #         qr_pose.position.y = trans[1]
            #         qr_pose.position.z = trans[2]
            #         qr_pose.orientation.x = quat[0]
            #         qr_pose.orientation.y = quat[1]
            #         qr_pose.orientation.z = quat[2]
            #         qr_pose.orientation.w = quat[3]
            #         self.fridge_creator.mark_flag(qr_id, qr_pose)
            #     except:
            #         #rospy.logwarn("cannot detect fridge qr marker")
            #         pass      

            # # publish fridge tf
            # fridge_qr_pose = self.fridge_creator.get()

            # get Fridge qr pose from chessboard    
            try:           
                (trans,rot) = self.listener.lookupTransform(self.base_frame, "chessboard", rospy.Time(0))
            except:
                pass
            
            if trans and rot:
                chess_pose = Pose()
                chess_pose.position.x = trans[0]
                chess_pose.position.y = trans[1]
                chess_pose.position.z = trans[2]
                chess_pose.orientation.x = rot[0]
                chess_pose.orientation.y = rot[1]
                chess_pose.orientation.z = rot[2]
                chess_pose.orientation.w = rot[3]

                trans_pose = Pose()
                trans_pose.position.x = -3.5500e-01
                trans_pose.position.y = +1.3100e-01
                trans_pose.position.z = 0.0
                trans_pose.orientation.x = 0.0
                trans_pose.orientation.y = 0.0
                trans_pose.orientation.z = 0.707
                trans_pose.orientation.w = 0.707

                fridge_qr_pose = multiply_pose(chess_pose, trans_pose)

                # publish fridge QR pose
                pos = fridge_qr_pose.position
                ori = fridge_qr_pose.orientation                
                br.sendTransform((pos.x, pos.y, pos.z),
                                (ori.x, ori.y, ori.z, ori.w),
                                rospy.Time.now(),
                                self.marker_frame,
                                self.base_frame)
                            
            # get objects info from DB
            objects = []
            for i, obj in enumerate(self.config_data['objects']):
                marker_frame = 'QR:'+str(obj['id'])
                trans = rot = None

                try:
                    (trans,rot) = self.listener.lookupTransform(self.base_frame, marker_frame, rospy.Time(0))
                except:
                    #rospy.logerr("cannot get tf between %s and %s." %(self.base_frame, marker_frame))
                    pass
                
                object_list = []
                if trans and rot:
                    # create object instance
                    object_list = self.createObjectMsg(obj, parent_frame=self.base_frame, child_frame=marker_frame)

                    # exception for fridge
                    if obj['name'] == 'obj_fridge':
                        marker_trans = marker_rot = None
                        # if cannot detect door marker, door is opened
                        try:                        
                            #self.listener.waitForTransform(self.marker_frame, self.door_frame, rospy.Time(), rospy.Duration(1.0))
                            (marker_trans,marker_rot) = self.listener.lookupTransform(self.marker_frame, self.door_frame, rospy.Time(0))
                            if marker_trans:
                                dist = np.sqrt(pow(marker_trans[0],2) + pow(marker_trans[1],2) + pow(marker_trans[2],2))
                                if dist > 0.5:
                                    # transpose door angle
                                    self.rotate_door(object_list)
                                else:                                
                                    # set ros param 
                                    rospy.set_param('fridge_isopen', 'False')
                        except:
                            # transpose door angle                
                            self.rotate_door(object_list)  

                    cuboid_markers, pose_markers = self.createMarkers(obj, object_list) 
                    self.markerPublish(cuboid_markers, pose_markers) 

                    # add objects
                    objects += object_list      

            # create marker for base object
            if len(objects)>0:               
                self.objectPublish(objects)    

            rate.sleep()
        return

if __name__ =='__main__':
    # Initialize ROS node
    rospy.init_node('fridge_detector', anonymous=True)   
    r = rospkg.RosPack()
    pkg_path = r.get_path('qr_tracker')
    yaml_path = pkg_path + '/cfg/objects.yaml'
    oc = ObjectCreator(yaml_path)

    try:
        oc.run()
    except KeyboardInterrupt:
        print("Shutting down")