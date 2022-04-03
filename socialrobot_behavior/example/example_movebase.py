#!/usr/bin/env python
import roslib
roslib.load_manifest('socialrobot_behavior')

import os
import os.path
import sys
import signal
import rospy
import rosparam
import tf
import numpy as np
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
from visualization_msgs.msg import *
from socialrobot_motion.srv import *
from socialrobot_hardware.srv import *
from socialrobot_behavior.srv import *
from socialrobot_msgs import msg as social_robot_msg

       
class AppraochArmTest():

    def __init__(self):
        obj_topic = "/socialrobot/perception/objects"
        self.detected_objects = []
        self.objects_from_robot = []
        self.objects_from_camera = []

        # 
        rospy.Subscriber("/socialrobot/perception/objects", social_robot_msg.Objects, self._callback_objects)
        rospy.Subscriber("/socialrobot/qr_tracker/objects", social_robot_msg.Objects, self._callback_ex_objects)
        rospy.sleep(1.0)

        #
        # pubisher
        self.path_pub = rospy.Publisher('/mobile_path', Path, queue_size=10)
        self.pose_pub = rospy.Publisher('/mobile_pose', PoseArray, queue_size=10)
        self.pub_marker = rospy.Publisher("/socialrobot/perception/markers", MarkerArray, queue_size=10)  

    def _callback_objects(self, data):        
        if data.detected_objects != []:
            self.objects_from_robot = data.detected_objects
        return

    def _callback_ex_objects(self, data):        
        if data.detected_objects != []:
            self.objects_from_camera = data.detected_objects
        return

    def res_to_msg(self, plan_result):
        path = Path()
        pose_array = PoseArray()

        path.header.stamp = pose_array.header.stamp = rospy.Time().now()
        path.header.frame_id = pose_array.header.frame_id = 'base_footprint'        

        for pt in plan_result.points:
            mobile_pose = pt

            pose = PoseStamped()
            pose.pose.position.x = mobile_pose.x
            pose.pose.position.y = mobile_pose.y
            pose.pose.position.z = 0.0
            quaternion = tf.transformations.quaternion_from_euler(0, 0, mobile_pose.theta)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            path.poses.append(pose)
            pose_array.poses.append(pose.pose)

        return path, pose_array

    def get_motion(self, request):
        # 
        if self.objects_from_robot != None:
            self.detected_objects = self.objects_from_camera + self.objects_from_robot
        if self.detected_objects == None:
            self.add_objects()

        # create marker
        marker_array = self.createMarkers(self.detected_objects)
        self.pub_marker.publish(marker_array)
        
        for obj in self.detected_objects:
            request.requirements.dynamic_object.append(obj)
        
        # get motion
        motion_srv = rospy.ServiceProxy('/behavior/get_motion', GetMotion)
        #print(request)
        res = motion_srv(request)
        return res

    def set_motion(self, plan):
        # set behavior
        if plan.result:
            behavior_srv = rospy.ServiceProxy('/behavior/set_behavior', SetBehavior)
            behavior_req = SetBehaviorRequest()
            behavior_req.behavior_name = plan_req.requirements.name
            behavior_req.path = plan.motion.pathTrajectory
            behavior_res = behavior_srv(behavior_req)
            return behavior_res
    
    def add_objects(self):        
        # red_gotica
        object1 = social_robot_msg.Object()
        object1.id = "obj_fridge"
        obs1 = BoundingBox3D()
        c1 = Pose()
        c1.position.x = +7.3238e-01
        c1.position.y = -7.4062e-01
        c1.position.z = +6.8503e-01
        c1.orientation.x = 0
        c1.orientation.y = 0
        c1.orientation.z = 0.967
        c1.orientation.w = 0.253
        obs1.center = c1
        v1 = Vector3()
        v1.x = 5.0008e-01
        v1.y = 5.8955e-01
        v1.z = 1.3600e+00
        obs1.size = v1
        object1.bb3d = obs1

        # gotica
        object2 = social_robot_msg.Object()
        object2.id = "obj_fridge_door"
        obs2 = BoundingBox3D()
        c2 = Pose()
        c2.position.x = +5.6938e-01
        c2.position.y = -1.8577e-01
        c2.position.z = +4.6997e-01
        c2.orientation.x = 0.0
        c2.orientation.y = 0.0
        c2.orientation.z = -1.0
        c2.orientation.w = 0.0
        obs2.center = c2
        v2 = Vector3()
        v2.x = 8.0000e-02
        v2.y = 5.0000e-01
        v2.z = 9.2000e-01
        obs2.size = v2
        object2.bb3d = obs2

        self.detected_objects = [object1, object2]

    def createMarkers(self, detected_objects):        
        marker_array = MarkerArray()

        for obj in detected_objects:
            # object base
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = '/base_footprint'
            marker.ns = obj.id
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.MODIFY

            marker.pose = obj.bb3d.center
            marker.scale = obj.bb3d.size
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)
            
        # robot mobile
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = '/base_footprint'
        marker.ns = 'robot'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.MODIFY

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.55
        marker.scale.y = 0.55
        marker.scale.z = 0.01

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)
        marker_array.markers.append(marker)

        return marker_array

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('behavior_example')
    robot_name = rosparam.get_param("/robot_name")
    example = AppraochArmTest()
    rospy.sleep(1.0)

    plan_req = GetMotionRequest()
    plan_req.requirements.name = "movebase"

    # robot group
    plan_req.requirements.robot_group = [plan_req.requirements.MOBILE_BASE]

    # goal pose
    position = social_robot_msg.Position()
    position.pose = Pose()
    position.pose.position.x = 0.0
    position.pose.position.y = 0.0
    yaw = np.deg2rad(-45)

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    position.pose.orientation.x = quaternion[0]
    position.pose.orientation.y = quaternion[1]
    position.pose.orientation.z = quaternion[2]
    position.pose.orientation.w = quaternion[3]

    plan_req.requirements.goal_position = [position]

    # get mobile path
    motion_plan = example.get_motion(plan_req)        
    print(motion_plan)
    rospy.logwarn("========== Done! ==========")
    
    # publish results
    iter=0
    while(not rospy.is_shutdown() and iter<100):
        example.path_pub.publish(motion_plan.motion.pathTrajectory)
        iter+=1

    if motion_plan.result:
        input = raw_input("Execute=y Pass=n : ")
        if input == 'y':
            example.set_motion(motion_plan)
        else:
            pass
