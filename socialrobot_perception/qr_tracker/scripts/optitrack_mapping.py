#!/usr/bin/env python
from calendar import c
import rospy
import rospkg
import rosparam
import tf
import tf.transformations as tfm
import yaml
import math
import os.path
from math import pi
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import BoundingBox3D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, PoseStamped, Quaternion, Twist, Vector3
from sensor_msgs.msg import Image, CameraInfo
from socialrobot_msgs.msg import Object, Objects, Affordance

from pose_filter import blend

rot_x = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (1, 0, 0))
rot_y = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 1, 0))
rot_z = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 0, 1))


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

def inverse_pose(pose):
    mat = pose_to_mat(pose)    
    return mat_to_pose(np.linalg.inv(mat))

def pose_to_vec(pose):
    trans = [pose.position.x, pose.position.y, pose.position.z]
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    return trans, quat

def vec_to_pose(trans, quat):
    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]

    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

def diff_pose_trans(a, b):
    diff = np.sqrt(pow((a.position.x - b.position.x),2) + pow((a.position.y - b.position.y),2) + pow((a.position.z - b.position.z),2))
    return diff

def diff_pose_quat(a, b):
    transform = diff_pose(a,b)
    trans, quat = pose_to_vec(transform)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
    return roll, pitch, yaw

def diff_pose(a, b):
    return multiply_pose(inverse_pose(a), b)


class ObjectInstance:
    def __init__(self, object_name, listener):
        self.listener = listener
        self.has_pose = False
        self.id = object_name
        self.map_based_pose = None
        self.robot_based_pose = None
        self.msgs = []
        self.config = None
        self.last_update_time = rospy.Time().now()

        topic = '/vrpn_client_node/' + self.id +'/pose'
        rospy.Subscriber(topic, PoseStamped, self.pose_callback)

    def pose_callback(self, pose):
        # adjust specific object's z position
        if 'robot' in self.id:
            pose.pose.position.z = 0.0
        elif 'fridge' in self.id:
            pose.pose.position.x -= 0.025
            pose.pose.position.z -= 0.03
        self.update_pose(pose.pose)
        self.has_pose = True

    def update_pose(self, pose):
        if pose is not None:
            # object must be parallel to the floor 
            corr_pose = self.correct_orientation(pose)
            if self.map_based_pose is not None:
                self.map_based_pose = blend(self.map_based_pose, corr_pose)
            else:
                self.map_based_pose = corr_pose
            self.last_update_time = rospy.Time().now()     
        # base_footprint based pose
        try:
            self.listener.waitForTransform('base_footprint', self.id, rospy.Time(), rospy.Duration(1.0))
            pos, ori = self.listener.lookupTransform('base_footprint', self.id, rospy.Time(0))
            self.robot_based_pose = vec_to_pose(pos, ori)
        except:
            pass

    def correct_orientation(self, pose):
        trans, quat = pose_to_vec(pose)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        corr_pose = vec_to_pose(trans, quat)
        return corr_pose

    """
    def update_pose(self, pose):
        if pose != None:
            # object must be parallel to the floor      
            corr_pose = self.correct_orientation(pose)
            has_pose_error = self.check_pose_error(pose)

            if corr_pose != None and not has_pose_error:
                self.map_based_pose = corr_pose
                self.last_update_time = rospy.Time().now()
                       
        # base_footprint based pose
        try:
            pos, ori = self.listener.lookupTransform('base_footprint', self.id, rospy.Time(0))
            self.robot_based_pose = vec_to_pose(pos, ori)
        except:
            pass
    
    def check_pose_error(self, pose):
        has_position_error = False
        has_orientation_error = False

        if self.map_based_pose != None:
            dist = diff_pose_trans(self.map_based_pose, pose)

            if abs(dist) > 0.1:
                has_position_error = True
            else:
                has_position_error = False

            roll, pitch, yaw = diff_pose_quat(self.map_based_pose, pose)
            thre = np.deg2rad(30)
            
            if abs(roll) > thre or abs(pitch) > thre or abs(yaw) > thre:
                has_orientation_error = True
            else:
                has_orientation_error = False

        if has_position_error or has_orientation_error:
            return True
        else:
            return False

    def correct_orientation(self, pose):
        thre = np.deg2rad(10)  #radian threshold
        trans, quat = pose_to_vec(pose)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)

        if roll > thre or pitch > thre:
            return None
        else:
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            corr_pose = vec_to_pose(trans, quat)
            return corr_pose
    """

        
class MapBuilder:
   
    def __init__(self,):
        self.data = {}
        self.config_data = []
        self.fridge_isopen = False
           
        self.rospkg = rospkg.RosPack()
        yaml_path = self.rospkg.get_path('qr_tracker') + '/cfg/mocap.yaml'
        rospy.set_param('fridge_isopen', False)

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        # publisher
        self.pub_objs = rospy.Publisher("/socialrobot/qr_tracker/objects", Objects, queue_size=10)
        self.pub_marker = rospy.Publisher("/socialrobot/visualization/markers", MarkerArray, queue_size=10) 
        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)

        self.load_config(yaml_path)
        self.create_object_instance()


    def load_config(self, yaml_path):
        with open(yaml_path, 'r') as stream:
            try:
                self.config_data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return False
            return True

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

    def getBoundingBox(self, trans, rot, size, parent_frame=None, child_frame=None):
        '''
        get boundingbox msg
        '''
        trans_vec = [trans['x'], trans['y'], trans['z']]
        rot_vec = [rot['x'], rot['y'], rot['z'], rot['w']]
        
        if parent_frame:
            # base_footprint to marker frame
            self.listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(1.0))
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

    def create_obj_msg(self, object_info, frame_id='base_footprint'):
        '''
        create Object.msg
        '''
        name = object_info['name']
        size = object_info['size']
        type = object_info['type']
        trans = object_info['translation']
        rot = object_info['rotation']
        aff = None
        base_frame = frame_id

        object_list = []
        obj = Object()
        obj.header.frame_id = base_frame
        obj.id = name
        obj.type = type
        obj.bb3d = self.getBoundingBox(trans, rot, size, base_frame, name)
        object_list.append(obj)

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
                aff.bb3d = self.getBoundingBox(aff_trans, aff_rot, aff_size)

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
                    obj_aff.header.frame_id = base_frame
                    obj_aff.id = aff.id
                    obj_aff.type = 'static'
                    # transform 
                    obj_aff.bb3d.size = aff.bb3d.size
                    obj_aff.bb3d.center = multiply_pose(obj.bb3d.center, aff.bb3d.center)
                    object_list.append(obj_aff)
        return object_list

    def createMarkers(self, obj_msg, id, frame_id='base_footprint'):   
        '''
        convert object msg to marker msg
        '''
        
        # object base
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = frame_id
        marker.ns = 'obj'
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.MODIFY
        marker.pose = obj_msg.bb3d.center
        marker.scale = obj_msg.bb3d.size
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.lifetime = rospy.Duration(2.0)

        # if object mesh is known
        mesh_dir = self.rospkg.get_path('socialrobot_motion') + '/mesh/moveit/' + obj_msg.id + '.stl'
        if os.path.isfile(mesh_dir):
            marker.type = marker.MESH_RESOURCE
            marker.action = Marker.MODIFY
            marker.mesh_resource = "package://socialrobot_motion/mesh/moveit/" + obj_msg.id + ".stl"
            marker.mesh_use_embedded_materials = True   
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.ns = 'mesh'                

        return marker

    def create_object_instance(self):
        for i, object_info in enumerate(self.config_data['objects']):
            obj_inst = ObjectInstance(object_info['name'], self.listener)  
            obj_inst.config = object_info
            self.data[object_info['name']] = obj_inst

    def rotate_door(self, object_list):        
        
        for i,obj in enumerate(object_list):
            # transform only door object 
            if 'door' in obj.id:
                closed_door_pose = obj.bb3d.center
                opened_joint_pose = multiply_pose(closed_door_pose, create_pose([-4.4977e-02, +2.5000e-01, 0.0], rot_z(120)))
                opened_door_pose = multiply_pose(opened_joint_pose, create_pose([4.4977e-02, -2.5000e-01, 0.0], [0,0,0,1]))                
                object_list[i].bb3d.center = opened_door_pose

            elif 'handle' in obj.id:
                closed_handle_pose = obj.bb3d.center
                opened_joint_pose = multiply_pose(closed_handle_pose, create_pose([-9.8475e-02, +4.5000e-01, 0.0], rot_z(120)))
                opened_handle_pose = multiply_pose(opened_joint_pose, create_pose([9.8475e-02, -4.5000e-01, 0.0], [0,0,0,1]))                
                object_list[i].bb3d.center = opened_handle_pose

    def publish_tf(self):
        # broadcast tf objects based on map
        for id in self.data.keys():
            pose = self.data[id].map_based_pose
            parent_frame = 'map'
            child_frame = id
            if pose != None:
                tran, quat = pose_to_vec(pose)
                self.broadcaster.sendTransform(tran, quat, rospy.Time.now(), child_frame, parent_frame)

    def publish_odom(self): 

        # broadcast odom and map 
        try:
            # tf 
            self.listener.waitForTransform('map', 'obj_robot', rospy.Time(), rospy.Duration(1.0))
            pos, ori = self.listener.lookupTransform('map', 'obj_robot', rospy.Time(0))
            self.broadcaster.sendTransform([pos[0], pos[1], 0], ori, rospy.Time().now(), 'base_footprint', 'odom')
            self.broadcaster.sendTransform([0,0,0], [0,0,0,1], rospy.Time().now(), 'odom', 'map')

            # odom
            odom = Odometry()
            odom.header.stamp = rospy.Time().now()
            odom.header.frame_id = "odom"
            odom.pose.pose = Pose(Point(pos[0], pos[1], 0.), Quaternion(*ori))
            
            odom.child_frame_id = "base_footprint"
            #odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            # publish the message
            self.pub_odom.publish(odom)
        except:
            pass

        # # broadcast tf between odom and map when using odom from robot
        # try:
        #     pos, ori = self.listener.lookupTransform('map', 'obj_robot', rospy.Time(0))
        #     map_to_robot = vec_to_pose(pos, ori)
        #     pos, ori = self.listener.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        #     odom_to_robot = vec_to_pose(pos, ori)
        #     map_to_odom = multiply_pose(map_to_robot, inverse_pose(odom_to_robot))
        #     tran, quat = pose_to_vec(map_to_odom)
        #     self.broadcaster.sendTransform(tran, quat, rospy.Time().now(), 'odom', 'map')
        # except:
        #     pass

    def publish_markers(self):
        marker_arr = MarkerArray()
        idx = 0
        for id in self.data.keys():
            obj_msgs = self.data[id].msgs  
            for obj in obj_msgs: 
                marker_arr.markers.append(self.createMarkers(obj, idx))
                idx += 1
        self.pub_marker.publish(marker_arr)

    def publish_objects(self):
        msg = Objects()

        for id in self.data.keys():
            if id != 'obj_robot' or 'obj_' not in id:
                msg.detected_objects += (self.data[id].msgs)
        self.pub_objs.publish(msg)

    def run(self):
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        rate = rospy.Rate(100)

        while(not rospy.is_shutdown()):
            current_time = rospy.Time.now()

            for id in self.data.keys():

                # create object message
                if self.data[id].robot_based_pose is not None:
                    rospy.logwarn_once('found robot base_footprint tf.')
                    obj_msgs = self.create_obj_msg(self.data[id].config)
                    # exception for fridge
                    if id == 'obj_fridge': 
                        if rospy.has_param('fridge_isopen'):
                            if rospy.get_param('fridge_isopen'):
                                self.door_isopen = True                                    
                                self.rotate_door(obj_msgs)
                            else: 
                                self.door_isopen = False
                    self.data[id].msgs = obj_msgs
                else:
                    rospy.logerr_once('cannot find robot base_footprint tf.')

            # publish
            self.publish_objects()
            self.publish_tf()
            self.publish_odom()
            self.publish_markers()

            self.last_time = self.current_time
            rate.sleep()
     

if __name__ =='__main__':
    # Initialize ROS node
    rospy.init_node('optitrack_mapping', anonymous=True)    
    rospy.loginfo("[Optitrack_mapping] start.")
    builder = MapBuilder()
    builder.run()
    rospy.logwarn("[Optitrack_mapping] Shutting down.")