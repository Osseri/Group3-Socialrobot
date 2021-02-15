#!/usr/bin/env python
import os.path
import copy
import yaml
import rospy
import rospkg

from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_perception_msgs.srv import *
from socialrobot_perception_msgs.msg import *
from socialrobot_hardware.msg import *
from obj_msg.msg import *
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection3DArray

import tf
import tf.transformations as tfm
import tf2_ros
import numpy as np 

class PerceptionManager():
    CONFIG_PATH = rospkg.RosPack().get_path('perception_interface') + '/config/'

    def __init__(self):
        self.dataset = []
        self.base_to_camera_mat = {'mat':None, 'trans':None, 'rot':None}
        self.head_to_camera_mat = {'mat':None, 'trans':None, 'rot':None}
        self.detected_object = []
        self.listener = tf.TransformListener()
        self.current_obstacles = {}

        # Subscriber
        #rospy.Subscriber('/obj_info', ObjInfoArrayMsg, self._callback_objects)
        #rospy.Subscriber('/gr_info', GraspArray, self._callback_grasp)
        rospy.Subscriber('/bbox_3d', Detection3DArray, self._callback_bbox_3d)

        # Publisher
        self.pub_objects = rospy.Publisher("/perception/objects", Objects, queue_size=10)
        self.pub_marker = rospy.Publisher('/perception/objects_marker', Marker, queue_size=10)
        self.pub_trans_marker = rospy.Publisher('/perception/transformed_marker', Marker, queue_size=10)

        # read configs
        self.read_camera_matrix()
        self.read_dataset()

        # get static tf from head to camera link
        self.calculate_head_to_camera_matrix()

    def read_dataset(self):
        fname = self.CONFIG_PATH + 'dataset.txt'
        f = open(fname, "r")
        data = f.readlines()
        
        for x in data:
            self.dataset.append(x[:-1])        

    def read_camera_matrix(self):        
        fname = self.CONFIG_PATH + "camera_matrix.yaml"
        with open(fname) as f:
            data = yaml.load(f)

        mat = data['camera_matrix']
        self.base_to_camera_mat['mat'] = mat
        self.base_to_camera_mat['trans'] = tf.transformations.translation_from_matrix(mat) 
        self.base_to_camera_mat['rot'] = tf.transformations.quaternion_from_matrix(mat)

    def calculate_head_to_camera_matrix(self):
        listener = tf.TransformListener()
        depth_to_camera_trans = None
        depth_to_camera_rot = None
        head_to_camera_trans = None
        head_to_camera_rot = None
        # get trasform between depth to camera
        while(depth_to_camera_trans == None or depth_to_camera_rot == None 
                or head_to_camera_trans == None or head_to_camera_rot == None):
            try:      
                self.publish_camera_tf('/base_footprint', '/perception_link', 
                                    self.base_to_camera_mat['trans'], self.base_to_camera_mat['rot'])
                depth_to_camera_trans, depth_to_camera_rot = listener.lookupTransform('/camera_depth_optical_frame', '/camera_link', rospy.Time(0))
                head_to_camera_trans, head_to_camera_rot = listener.lookupTransform('/Head_Pitch', '/perception_link', rospy.Time(0))
            except:
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
        self.publish_camera_tf('/Head_Pitch', '/camera_link', 
                                self.head_to_camera_mat['trans'], 
                                self.head_to_camera_mat['rot'])

    def publish_camera_tf(self, from_tf, to_tf, trans, rot):
        br = tf.TransformBroadcaster()
        quat_norm = rot / np.linalg.norm(rot)
        
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = from_tf
        static_transformStamped.child_frame_id = to_tf
        
        static_transformStamped.transform.translation.x = trans[0]
        static_transformStamped.transform.translation.y = trans[1]
        static_transformStamped.transform.translation.z = trans[2]
        static_transformStamped.transform.rotation.x = quat_norm[0]
        static_transformStamped.transform.rotation.y = quat_norm[1]
        static_transformStamped.transform.rotation.z = quat_norm[2]
        static_transformStamped.transform.rotation.w = quat_norm[3]
        broadcaster.sendTransform(static_transformStamped)

    def publish_marker(self, objects):
        for i, obj in enumerate(objects.detected_objects):
            # publish cube marker
            robotMarker = Marker()
            robotMarker.header.frame_id = obj.header.frame_id
            robotMarker.header.stamp    = rospy.get_rostime()
            robotMarker.ns = "object"
            robotMarker.id = i
            robotMarker.type = Marker().CUBE
            robotMarker.action = Marker().ADD
            robotMarker.pose.position = obj.bb3d.center.position
            robotMarker.pose.orientation = obj.bb3d.center.orientation
            robotMarker.scale.x = obj.bb3d.size.x
            robotMarker.scale.y = obj.bb3d.size.y
            robotMarker.scale.z = obj.bb3d.size.z
            robotMarker.color.r = 0.0
            robotMarker.color.g = 1.0
            robotMarker.color.b = 0.0
            robotMarker.color.a = 1.0

            robotMarker.lifetime = rospy.Duration(1)
            self.pub_marker.publish(robotMarker)    	

            # publish text marker
            textMarker = Marker()
            textMarker.header.frame_id = obj.header.frame_id
            textMarker.ns = ""
            textMarker.id = i
            textMarker.text = obj.name.data
            textMarker.type = Marker().TEXT_VIEW_FACING
            textMarker.action = Marker().ADD
            textMarker.pose.position = obj.bb3d.center.position
            textMarker.pose.orientation = obj.bb3d.center.orientation
            textMarker.scale.z = 0.1
            textMarker.color.r = 1.0
            textMarker.color.g = 1.0
            textMarker.color.b = 1.0
            textMarker.color.a = 1.0
            textMarker.lifetime = rospy.Duration(10)
            self.pub_marker.publish(textMarker)   

    # def _callback_objects(self, data):
    # 	'''
    # 	data.pt : Obj3dInfo[] 
	# 	data.header : std_msgs/Header 
	# 	Obj3dInfo :
	# 		int16 id
	# 		float32 score

	# 		geometry_msgs/Point[8] bb_pts
	# 		geometry_msgs/Vector3[3] bb_uv
	# 		geometry_msgs/Vector3 bb_sc

	# 		float32 bv_min_x
	# 		float32 bv_min_y
	# 		float32 bv_min_z
	# 		float32 bv_max_x
	# 		float32 bv_max_y
	# 		float32 bv_max_z
    # 	'''
    #     # object matching
    #     for pt in data.pt:
    #         idx = 999
    #         for i, d3d in enumerate(self.detected_object):
    #             if d3d.header.frame_id == str(pt.id):
    #                 idx = i
    #         if(idx != 999):
    #             self.detected_object[idx].bbox.size.x = pt.bb_sc.x
    #             self.detected_object[idx].bbox.size.y = pt.bb_sc.y
    #             self.detected_object[idx].bbox.size.z = pt.bb_sc.z       
    #     pass        

    def _callback_bbox_3d(self, data):
        self.detected_object = []
        for d3d in data.detections:
            d3d.bbox.size.z += 0.04
            d3d.bbox.center.position.z += 0.01 + 0.02
            self.detected_object.append(d3d)   

        # add static object(table)
        d3d = Detection3D()   
        d3d.tracking_id = int(0)
        d3d.bbox.center.position.x = 0.550006
        d3d.bbox.center.position.y = 0
        d3d.bbox.center.position.z = 0.4 + 0.01
        d3d.bbox.center.orientation.x = 0
        d3d.bbox.center.orientation.y = 0
        d3d.bbox.center.orientation.z = 0
        d3d.bbox.center.orientation.w = 1    
        d3d.bbox.size.x = 0.7088739275932312
        d3d.bbox.size.y = 1.2642161893844604
        d3d.bbox.size.z = 0.80   
        self.detected_object.append(d3d)
         
    # def _callback_grasp(self, data):
    # 	'''
    # 	data.header : std_msgs/Header
	# 	data.gr : GraspMsg[]
	# 	GraspMsg :
	# 		int8 num_type
	# 		int8 id
	# 		float32 grasp_cx
	# 		float32 grasp_cy
	# 		float32 grasp_cz
	# 		GraspElem[] gr_elements		
	# 	'''
    #     self.detected_object = []
    #     detected_object = []
    #     for gr in data.gr:
    #         bbox3d = BoundingBox3D()
    #         d3d = Detection3D()   
    #         d3d.header.frame_id = str(gr.id)
    #         d3d.bbox.center.position.x = gr.grasp_cx
    #         d3d.bbox.center.position.y = gr.grasp_cy
    #         d3d.bbox.center.position.z = gr.grasp_cz + 0.08
    #         d3d.bbox.center.orientation.x = 0
    #         d3d.bbox.center.orientation.y = 0
    #         d3d.bbox.center.orientation.z = 0
    #         d3d.bbox.center.orientation.w = 1
    #         detected_object.append(d3d)
    #     self.detected_object = detected_object            
        
    def transform_boundingbox_frame(self, objects, from_tf, to_tf):
        transformed_objects = Objects()
        trans = None
        rot = None
        rate = rospy.Rate(10.0)
        while(trans == None or rot == None):
            try:                                 
                trans, rot = self.listener.lookupTransform(from_tf, to_tf, rospy.Time(0))
            except:
                rate.sleep()
                pass
            
        for i,obj in enumerate(objects.detected_objects):
            transformed_object = Object()             
            transformed_object.name = obj.name

            bb = transformed_object.bb3d  
            # transform
            obj_trans_mat = tf.transformations.translation_matrix([obj.bb3d.center.position.x, obj.bb3d.center.position.y, obj.bb3d.center.position.z])
            obj_rot_mat   = tf.transformations.quaternion_matrix([obj.bb3d.center.orientation.x, obj.bb3d.center.orientation.y, obj.bb3d.center.orientation.z, obj.bb3d.center.orientation.w])
            obj_mat = np.dot(obj_trans_mat, obj_rot_mat)

            trans_mat = tf.transformations.translation_matrix(trans)
            rot_mat = tf.transformations.quaternion_matrix(rot)
            transform_mat = np.dot(trans_mat, rot_mat)                
            # get inverse transform
            transform_mat_inv = np.linalg.inv(transform_mat)

            transformed_mat = np.dot(obj_mat, transform_mat)
            pos = tf.transformations.translation_from_matrix(transformed_mat) 
            ori = tf.transformations.quaternion_from_matrix(transformed_mat)

            bb.center.position.x = pos[0]
            bb.center.position.y = pos[1]
            bb.center.position.z = pos[2]
            bb.center.orientation.x = ori[0]
            bb.center.orientation.y = ori[1]
            bb.center.orientation.z = ori[2]
            bb.center.orientation.w = ori[3]

            transformed_object.header.frame_id = to_tf
            transformed_object.bb3d.size = obj.bb3d.size

            transformed_objects.detected_objects.append(transformed_object)
        return transformed_objects
    
    def transform_pose(self, objects, from_tf, to_tf):
        transformed_objects = Objects()

        for obj in objects.detected_objects:
            transformed_object = Object()             
            transformed_object.name = obj.name

            #transform
            p = geometry_msgs.msg.PoseStamped()
            p.pose = obj.bb3d.center
            p.header.frame_id = from_tf
            transed_p = self.listener.transformPose(to_tf, p)
            
            transformed_object.header.frame_id = to_tf
            transformed_object.bb3d.center = transed_p.pose
            transformed_object.bb3d.center.orientation = p.pose.orientation # static orientation
            transformed_object.bb3d.size = obj.bb3d.size
            transformed_object.bb3d.center.position.z += 0.02
            transformed_objects.detected_objects.append(transformed_object)
        
        return transformed_objects
                    
    def convert_msg(self, d3d): 
        obj = Object()
        obj.header.stamp = rospy.Time.now()
        obj.header.frame_id = 'base_footprint'
        obj.name.data = self.dataset[int(d3d.tracking_id)]
        obj.bb3d.center.position = d3d.bbox.center.position
        obj.bb3d.center.orientation = d3d.bbox.center.orientation
        obj.bb3d.size = d3d.bbox.size
        return obj

    def update(self):  
        # convert message format
        objs = Objects()
        
        for d3d in self.detected_object:
            objs.detected_objects.append(self.convert_msg(d3d))
        if len(objs)>0:
            self.publish_data(objs) 

        # if(objs.detected_objects):
        #     # change frame 
        #     objs_in_perception = self.transform_pose(objs, "/base_footprint", "/perception_link")
        #     objs_in_camera = self.transform_pose(objs_in_perception, "/camera_depth_optical_frame", "/base_footprint")
        #     self.publish_data(objs_in_camera) 

    def publish_data(self, objects):
        self.publish_marker(objects)
        self.pub_objects.publish(objects)
        
##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('perception_interface')        

    # perception manager
    pm = PerceptionManager()

    # Start
    rospy.loginfo('[PerceptionInterface] Started!')


    loop_freq = 10 # 10hz
    r = rospy.Rate(loop_freq)
    while not rospy.is_shutdown():
        pm.update()
        r.sleep
