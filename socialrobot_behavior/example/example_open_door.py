#!/usr/bin/env python
import roslib
roslib.load_manifest('socialrobot_behavior')

import os
import os.path
import sys
import signal
import rospy
import rosparam

from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
from socialrobot_motion.srv import *
from socialrobot_hardware.srv import *
from socialrobot_behavior.srv import *
from socialrobot_msgs import msg as social_robot_msg

       
class AppraochArmTest():

    def __init__(self):
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

    def get_motion(self, target_object, request):
        # 
        self.detected_objects = []
        if self.objects_from_robot != None:
            self.detected_objects += self.objects_from_robot        
        if self.objects_from_robot != None:
            self.detected_objects += self.objects_from_camera
        if self.detected_objects == []:
            self.add_objects()
        
        for obj in self.detected_objects:
            request.requirements.dynamic_object.append(obj)
            if obj.id == target_object:
                request.requirements.target_object.append(obj)
        
        # get motion
        motion_srv = rospy.ServiceProxy('/behavior/get_motion', GetMotion)
        print(request)
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
        object1.id = "obj_red_gotica"
        obs1 = BoundingBox3D()
        c1 = Pose()
        c1.position.x = +3.0000e-01
        c1.position.y = +0.15
        c1.position.z = +8.2750e-01
        c1.orientation.x = 0
        c1.orientation.y = 0
        c1.orientation.z = 0
        c1.orientation.w = 1
        obs1.center = c1
        v1 = Vector3()
        v1.x = 0.0618015
        v1.y = 0.059508
        v1.z = 0.23814
        obs1.size = v1
        object1.bb3d = obs1

        self.detected_objects = [object1]

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('behavior_example')
    robot_name = rosparam.get_param("/robot_name")
    example = AppraochArmTest()
    rospy.sleep(1.0)

    plan_req = GetMotionRequest()
    plan_req.requirements.name = "opendoor"
    target_object = 'obj_fridge'

    # arm type
    plan_req.requirements.robot_group = [plan_req.requirements.MOBILE_BASE]

    # add obstacles from topic
    motion_plan = example.get_motion(target_object, plan_req)    
    
    # publish results
    iter=0
    while(not rospy.is_shutdown() and iter<100):
        example.path_pub.publish(motion_plan.motion.pathTrajectory)
        iter+=1

    if motion_plan.result:
        example.set_motion(motion_plan)
        rosparam.set_param('fridge_isopen', True)
        pass
    
