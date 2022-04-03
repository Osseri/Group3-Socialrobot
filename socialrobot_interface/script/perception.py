#!/usr/bin/env python
import rospy
import tf
from tf.transformations import euler_from_quaternion
from mongodb_store.message_store import MessageStoreProxy
from std_msgs import msg as std_msg
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import BoundingBox3D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from socialrobot_msgs import msg as social_msg
from socialrobot_msgs import srv as social_srv
from interface import InterfaceBase
import copy
import numpy as np

def pose_2_mat(pose):
    q = pose.orientation
    pos = pose.position
    trans_mat = tf.transformations.translation_matrix([pos.x,pos.y,pos.z])
    rot_mat = tf.transformations.quaternion_matrix([q.x,q.y,q.z,q.w])
    T = np.dot(trans_mat, rot_mat) 
    return T

def mat_2_pose(T):
    trans = tf.transformations.translation_from_matrix(T) 
    rot = tf.transformations.quaternion_from_matrix(T)
    pose = Pose(position=Point(x=trans[0], y=trans[1], z=trans[2]), 
                orientation=Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3]))    
    return pose

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

def multiply_pose(pose1, pose2):
    mat1 = pose_2_mat(pose1)
    mat2 = pose_2_mat(pose2)
    mat3 = np.dot(mat1, mat2)
    return mat_2_pose(mat3)

def inverse_pose(pose):
    mat = pose_2_mat(pose)    
    return mat_2_pose(np.linalg.inv(mat))

def diff_pose(a, b):
    return multiply_pose(inverse_pose(a), b)

class PerceptionInterface(InterfaceBase):

    # set the data label name for DB
    VISUAL_DATA_NAME = "/vision/objects"
    VOICE_DATA_NAME = "/voice/command"

    def __init__(self):
        super(PerceptionInterface, self).__init__()
        rospy.loginfo("Initializing PerceptionInterface...")

        # init data format
        self.footprint_based_objects = []
        self.odom_based_objects = []
        self.map_based_objects = []
        self.detected_objects = []
        self.use_mongodb = False
        self.msg_store = None
        self.joint_states = JointState()
        self.is_update = False
        self.is_updating = False
        self.base_moving = False
        self.has_map = False
        self.odom_to_base = None
        self.map_to_base = None
        self.tf_listener = tf.TransformListener()

        if rospy.has_param("/use_mongodb"):
            self.use_mongodb = rospy.get_param("/use_mongodb")
        if self.use_mongodb:
            self.msg_store = MessageStoreProxy()
            self.msg_store.insert_named(self.VISUAL_DATA_NAME, social_msg.Objects())
            self.msg_store.update_named(self.VISUAL_DATA_NAME, social_msg.Objects())

        # Publish data
        self.pub_object_markers = rospy.Publisher(
            "/socialrobot/visualization/objects", MarkerArray, queue_size=10)   
        self.pub_objects = rospy.Publisher(
            "/socialrobot/knowledge/objects", social_msg.Objects, queue_size=10)   
        self.pub_odom_map = rospy.Publisher(
            "/odom_map", Odometry, queue_size=10)   

        # Subscribe data from PerceptionManager
        rospy.Subscriber(
            "/socialrobot/perception/objects", social_msg.Objects, self._callback_social_objects)
        rospy.Subscriber(
            "/socialrobot/qr_tracker/objects", social_msg.Objects, self._callback_external_objects)
        rospy.Subscriber(
            "/socialrobot/behavior/status", std_msg.String, self._callback_robot_state)
        rospy.Subscriber(
            "/odom", Odometry, self._callback_odom)

        joint_topic = '/hw_interface/joint_states'
        if rospy.has_param('/robot_hw'):
            if rospy.get_param('/robot_hw') == 'vrep':
                joint_topic = '/sim_interface/joint_states'
        rospy.Subscriber(
            joint_topic, JointState, self._callback_joint_states
        )
        # Service
        self.scene_srv = rospy.ServiceProxy("/motion_plan/get_scene_objects", social_srv.GetObjects)
        self.update_scene_srv = rospy.ServiceProxy("/motion_plan/update_scene_objects", social_srv.UpdateObjects)

    def get_robot_joint_states(self):
        return self.joint_states

    def get_detected_objects(self):
        return self.detected_objects

    def get_odom_based_objects(self):
        return self.odom_based_objects

    def get_map_based_objects(self):
        return self.map_based_objects

    def get_footprint_based_objects(self):
        return self.footprint_based_objects
        
    def convert_object_info_base_to_odom(self, base_obj):
        '''
        convert object information based on 'odom' frame 'base_footprint' frame
        '''

        odom_obj = copy.deepcopy(base_obj)
        odom_obj.header.frame_id = 'odom'
        odom_obj.bb3d = self.transform_boundingbox(base_obj.bb3d, 
                                                self.odom_to_base)
        return odom_obj

    def convert_object_info_odom_to_base(self, odom_obj):
        '''
        convert object information based on 'base_footprint' from 'odom' frame
        '''

        base_obj = copy.deepcopy(odom_obj)
        base_obj.header.frame_id = 'base_footprint'
        base_obj.bb3d = self.transform_boundingbox(odom_obj.bb3d, 
                                                self.odom_to_base, inverse=True)
        return base_obj
    
    def convert_object_frame(self, object_info, inverse=False):
        '''
        convert object frame between 'map' and 'base_footprint'
        inverse=True : base to map
        inverse=False : map to base
        '''
        converted_info = copy.deepcopy(object_info)
        if inverse:
            converted_info.header.frame_id = 'map'
        else:
            converted_info.header.frame_id = 'base_footprint'
        converted_info.bb3d = self.transform_boundingbox(object_info.bb3d, 
                                                self.map_to_base, inverse)
        return converted_info

    def transform_boundingbox(self, bb3d, transform, inverse=False):
        transformed_bb3d = copy.deepcopy(bb3d)

        base_mat = pose_2_mat(bb3d.center)
        ref_mat = pose_2_mat(transform)
        if inverse == True:
            ref_mat = np.linalg.inv(ref_mat)
        transformed_mat = np.dot(ref_mat, base_mat)

        transformed_bb3d.center = mat_2_pose(transformed_mat)

        return transformed_bb3d

    def get_scene_objects(self, object_ids=None):
        """request objects info from moveit scene

        Args:
            object_ids ([list]): desired object name lists
                                if empty list, request all objects info
        Returns:
            result [int32]: SUCCESS = 1
                            FAIL = 0
            objects [socialrobot_msgs/Object[]]: 
        """
        req = social_srv.GetObjectsRequest()
        if object_ids:
            req.object_ids = object_ids
        res = self.scene_srv(req)
        return res.objects

    def load_object_info(self, object_id):

        msg_object = self.msg_store.query_named(
            self.VISUAL_DATA_NAME, social_msg.Objects._type
        )
        for obj in msg_object[0].detected_objects:
            # specific object
            if obj.name.data == object_id:
                return obj

    def find_object(self, object_id):
        '''
        return detected object index
        '''
        for index,obj in enumerate(self.detected_objects):
            if object_id == obj.id:
                return index

        # if cannot find, return -1
        return -1

    def create_object_markers(self, frame_id=None):
        '''
        create marker array for odom based object information
        '''
        marker_array = MarkerArray()

        if frame_id == 'odom':
            objects = self.odom_based_objects
        elif frame_id == 'base_footprint':
            objects = self.footprint_based_objects
        elif frame_id == 'map':
            objects = self.map_based_objects
        else:
            objects = self.detected_objects
            frame_id = 'base_footprint'

        for i,obj in enumerate(objects):
            # object base
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = frame_id
            marker.ns = 'objects'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.MODIFY    
            marker.pose = obj.bb3d.center
            marker.scale = obj.bb3d.size
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.4
            marker.lifetime = rospy.Duration(0.7)
            marker_array.markers.append(marker)
          
        return marker_array

    def _callback_robot_state(self, data):
        robot_state = data.data
        if robot_state == "MOVING_BASE":
            self.base_moving = True
        else:
            self.base_moving = False

    def _callback_joint_states(self, data):
        self.joint_states = data
        joint_names = data.name
        joint_pos = data.position
        head_pos = waist_pos = None

        for i,name in enumerate(joint_names):
            if name == 'Head_Pitch':
                head_pos = joint_pos[i]
            elif name == 'Waist_Pitch':
                waist_pos = joint_pos[i]

        # estimate detect pose for robocare robot
        if head_pos != None and waist_pos !=None:
            if head_pos < -0.29 and waist_pos > 0.47:
                self.is_update = True
                rospy.set_param("/detect_mode", True)
            else:
                self.is_update = False
                rospy.set_param("/detect_mode", False)

    def update_objects_from_scene(self):     
        '''
        get object info from moveit scene
        '''   
        scene_objects = self.get_scene_objects()

        for obj in scene_objects:
            index = self.find_object(obj.id)
            # not detected before
            if index < 0:               
                # save object info based on base frame
                self.detected_objects.append(obj)

            # update object body only except affordances
            else:
                #update only dynamic objects from scene
                if self.detected_objects[index].type == 'dynamic':
                    self.detected_objects[index].bb3d.center = obj.bb3d.center
                
        return
    
    def remove_scene_objects(self):
        '''
        update moveit scene objects
        '''
        req = social_srv.UpdateObjectsRequest()
        req.command = req.REMOVE
        for obj in self.odom_based_objects:
            for aff in obj.affordance:
                req.object_ids.append(aff.id)

    def _callback_social_objects(self, data):
        """
        perception manager callback
        """        
        detected_objects = list(data.detected_objects)

        # if self.is_update:
        #     for obj in detected_objects:
        #         index = self.find_object(obj.id)
        #         # detected first time
        #         if index < 0:
        #             self.footprint_based_objects.append(obj)

        #         # if already known object, update
        #         else:
        #             self.footprint_based_objects[index] = obj
        #             pass
        self.footprint_based_objects = detected_objects

        if self.use_mongodb:
            # update data into DB
            self.msg_store.update_named(self.VISUAL_DATA_NAME, data)

            # get it back with a name
            msg_object = self.msg_store.query_named(
                self.VISUAL_DATA_NAME, social_msg.Objects._type
            )
    
    def _callback_external_objects(self, data):
        detected_objects = list(data.detected_objects)

        # for obj in detected_objects:
        #     index = self.find_object(obj.id)
        #     # detected first time
        #     if index < 0:               
        #             self.footprint_based_objects.append(obj)

        #     # if already known object, update
        #     else:
        #         self.footprint_based_objects[index] = obj
        #         pass
        self.footprint_based_objects = detected_objects

    def _callback_odom(self, data):
        '''
        get transform from map/odom to basefootprint
        '''
        # odom to base pose
        prev_odom = self.odom_to_base        
        self.odom_to_base = data.pose.pose
        if self.odom_to_base != None and prev_odom != None:
            #diff_odom = diff_pose(self.odom_to_base, prev_odom)

            #if robot base moving, transform object pose
            if self.base_moving:            
                for i, obj in enumerate(self.detected_objects):
                    prev_obj_pose = multiply_pose(prev_odom, obj.bb3d.center)
                    curr_obj_pose = multiply_pose(inverse_pose(self.odom_to_base), prev_obj_pose) 
                    self.detected_objects[i].bb3d.center = curr_obj_pose

        # self.update_objects()

    
    def publish_object_markers(self, frame_id='base_footprint'):
        marker_array = self.create_object_markers()
        self.pub_object_markers.publish(marker_array)

    def _obj2float_array(self, idx, obj):
        """
        convert msg format
        from: socialrobot_perception_msgs/Object 
        to  : std_msgs/Float32MultiArray 
        """

        float_array = std_msg.Float32MultiArray()
        # object name
        dim = std_msg.MultiArrayDimension()
        dim.label = obj.name.data
        float_array.layout.dim.append(dim)
        # object center position
        position = obj.bb3d.center.position
        float_array.data += [position.x, position.y, position.z]
        # object center orientation
        quaterion = obj.bb3d.center.orientation
        float_array.data += list(
            euler_from_quaternion([quaterion.x, quaterion.y, quaterion.z, quaterion.w])
        )
        float_array.data.append(idx)
        return float_array
    
    def update_objects(self):
        '''
        '''
        rospy.loginfo("[Socialrobot Perception] Update objects.")
        for obj in self.footprint_based_objects:
            print(obj.id)
            index = self.find_object(obj.id)
            # detected first time
            if index < 0:               
                self.detected_objects.append(obj)

            # if already known object, update
            else:
                self.detected_objects[index] = obj
                pass


    def publish_objects(self):
        # publish messages
        self.pub_objects.publish(self.footprint_based_objects)

        # publish markers
        self.publish_object_markers('base_footprint')