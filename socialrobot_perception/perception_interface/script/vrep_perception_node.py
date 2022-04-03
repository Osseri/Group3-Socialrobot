#!/usr/bin/env python
import rospy
import tf

from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray
from socialrobot_perception_msgs import msg as social_perception_msg
from socialrobot_hardware.msg import ObjectInfo
from socialrobot_msgs import msg as social_robot_msg
import numpy as np

def split_at(s, c, n):
    words = s.split(c)
    return c.join(words[:n]), c.join(words[n:])

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

class PerceptionManager():
    def __init__(self):

        self.objects_world = {}
        self.objects_robot = {}
        self.affordance_world = {}
        self.affordance_robot = {}

        #rospy.Subscriber('/sim_interface/objects', ObjectInfo, self._callback_objects)
        rospy.Subscriber('/sim_interface/objects_base', ObjectInfo, self._callback_objects_base)
        self.pub_objects = rospy.Publisher("/perception/objects", social_perception_msg.Objects, queue_size=10)
        self.pub_social_objects = rospy.Publisher("/socialrobot/perception/objects", social_robot_msg.Objects, queue_size=10)
        self.pub_marker = rospy.Publisher("/socialrobot/visualization/objects", MarkerArray, queue_size=10)   

    def publish(self):
        objs = social_perception_msg.Objects()
        social_objs = social_robot_msg.Objects()
        obj_list = []
        marker_array = MarkerArray()

        # publish object info based on robot base
        for name, bb3d in self.objects_robot.items():
            obj = social_perception_msg.Object()
            obj.header.stamp = rospy.Time.now()
            obj.header.frame_id = 'base_footprint'

            social_obj = social_robot_msg.Object()
            social_obj.header = obj.header
            social_obj.id = obj.name.data = name
            bb3d.size.x *= 1.0
            bb3d.size.y *= 1.0
            bb3d.size.z *= 1.0

            social_obj.bb3d = obj.bb3d = bb3d     
            objs.detected_objects.append(obj)   
            social_objs.detected_objects.append(social_obj)
            obj_list.append(name)

        # publish socialrobot_perception_msgs format
        #self.pub_objects.publish(objs)

        # assign affordance into object
        self.assign_affordance(social_objs, obj_list)

        # publish socialrobot_msgs format
        self.pub_social_objects.publish(social_objs)

        # publish marker
        for i in social_objs.detected_objects:
            marker_array.markers += self.createMarker(i) 
        self.pub_marker.publish(marker_array)

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

            # # transform affordance based on object 
            # mat1 = pose_2_mat(objectMsg.bb3d.center)
            # mat2 = pose_2_mat(aff.bb3d.center)
            # mat3 = np.dot(mat1, mat2)
            # pose = mat_2_pose(mat3)
            # marker.pose = pose

            marker_array.append(marker)            
        return marker_array

    def assign_affordance(self, social_objects, obj_list):

        for aff_name, aff_bb3d in self.affordance_robot.items():
            aff_id, obj_id = split_at(aff_name, '_', 2)
            obj_id = 'obj_'+ obj_id

            # parent object index
            idx = obj_list.index(obj_id)
            obj = social_objects.detected_objects[idx]

            # # transform affordance based on parent object
            # T_robot_to_aff = pose_2_mat(aff_bb3d.center)
            # T_robot_to_obj = pose_2_mat(obj.bb3d.center)

            # T_obj_to_aff = np.dot(np.linalg.inv(T_robot_to_obj), T_robot_to_aff)
            # pose = mat_2_pose(T_obj_to_aff)

            aff = social_robot_msg.Affordance()
            aff.id = aff_id
            # aff.bb3d.center = pose
            # aff.bb3d.size = aff_bb3d.size
            aff.bb3d = aff_bb3d
            aff.header.frame_id = 'base_footprint'

            #
            social_objects.detected_objects[idx].affordance.append(aff)

    def update(self): 
        self.publish()   

    def _callback_objects(self, data):
        self.objects_world = {}
        for idx, name in enumerate(data.names):
            if 'aff_' in name:
                self.affordance_world[name] = data.obstacles[idx]
            else:
                self.objects_world[name] = data.obstacles[idx]

    def _callback_objects_base(self, data):
        self.objects_robot = {}
        for idx, name in enumerate(data.names):
            if 'aff_' in name:
                self.affordance_robot[name] = data.obstacles[idx]
            else:
                self.objects_robot[name] = data.obstacles[idx]

##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('perception')        

    # perception manager
    pm = PerceptionManager()

    # Start
    rospy.loginfo('[PerceptionManager] Service Started!')


    loop_freq = 10 # 10hz
    r = rospy.Rate(loop_freq)
    while not rospy.is_shutdown():
        pm.update()
        r.sleep()
