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
from socialrobot_motion.srv import *
from socialrobot_hardware.srv import *
from socialrobot_behavior.srv import *
from socialrobot_msgs import msg as social_robot_msg
import moveit_commander

class AppraochArmTest():

    def __init__(self):
        obj_topic = "/socialrobot/perception/objects"
        self.detected_objects = []
        self.objects_from_robot = []
        self.objects_from_camera = []
    
        rospy.sleep(1.0)  

        # 
        rospy.Subscriber("/socialrobot/perception/objects", social_robot_msg.Objects, self._callback_objects)
        rospy.Subscriber("/socialrobot/qr_tracker/objects", social_robot_msg.Objects, self._callback_ex_objects)
        rospy.sleep(1.0)

    def _callback_objects(self, data):        
        if data.detected_objects != []:
            self.objects_from_robot = data.detected_objects
        return

    def _callback_ex_objects(self, data):        
        if data.detected_objects != []:
            self.objects_from_camera = data.detected_objects
        return
    def get_motion(self, target_object, request):
        # 
        self.detected_objects = []
        if self.objects_from_robot != None:
            self.detected_objects += self.objects_from_camera
        if self.objects_from_robot != None:
            self.detected_objects += self.objects_from_robot
        
        for obj in self.detected_objects:
            request.requirements.dynamic_object.append(obj)

            if obj.id == target_object:
                request.requirements.target_object.append(obj)
                
            # if obj.id == target_object:
            #     # obj.bb3d.center.position.y = 0.0
            #     # obj.bb3d.center.position.z -= 0.04
            #     # obj.bb3d.center.orientation.x = 0.0
            #     # obj.bb3d.center.orientation.y = 0.0
            #     # obj.bb3d.center.orientation.z = 0.0
            #     # obj.bb3d.center.orientation.w = 1.0
            #     request.requirements.target_object.append(obj)

            # object_pose = geometry_msgs.msg.PoseStamped()
            # object_pose.header.frame_id = '/base_footprint'
            # object_pose.pose = obj.bb3d.center
            
            # scale = [obj.bb3d.size.x, obj.bb3d.size.y, obj.bb3d.size.z]
            # self.scene.add_box(obj.id, object_pose, size=scale)
        
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
            behavior_req.trajectory = plan.motion.jointTrajectory
            behavior_res = behavior_srv(behavior_req)
            return behavior_res
    
    def add_objects(self):        
        # box
        object1 = social_robot_msg.Object()
        object1.id = "obj_courier_box"
        obs1 = BoundingBox3D()
        c1 = Pose()
        c1.position.x = 0.29177
        c1.position.y = 0.0
        c1.position.z = 0.841748
        c1.orientation.x = 0.0
        c1.orientation.y = 0.0
        c1.orientation.z = 0.0
        c1.orientation.w = 1.0
        obs1.center = c1
        v1 = Vector3()
        v1.x = 0.05  
        v1.y = 0.125
        v1.z = 0.265
        obs1.size = v1
        object1.bb3d = obs1

        # table
        object4 = social_robot_msg.Object()
        object4.id = "obj_table"
        obs4 = BoundingBox3D()
        c4 = Pose()
        c4.position.x = 0.550006
        c4.position.y = 0.0
        c4.position.z = 0.36
        c4.orientation.x = 0
        c4.orientation.y = 0
        c4.orientation.z = 0.707
        c4.orientation.w = 0.707
        obs4.center = c4
        v4 = Vector3()
        v4.x = 1.1342161893844604
        v4.y = 0.7088739275932312
        v4.z = 0.72
        obs4.size = v4
        object4.bb3d = obs4


        self.detected_objects = [object1, object4]

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('behavior_example')
    robot_name = rosparam.get_param("/robot_name")
    example = AppraochArmTest()
    rospy.sleep(1.0)

    plan_req = GetMotionRequest()
    plan_req.requirements.name = "approachdualarm"
    target_object = 'obj_tray'

    # arm type
    plan_req.requirements.robot_group = [plan_req.requirements.BOTH_ARM]

    # add obstacles from topic
    motion_plan = example.get_motion(target_object, plan_req)

    if motion_plan.result:
        input = raw_input("Execute=y Pass=n : ")
        if input == 'y':
            example.set_motion(motion_plan)
        else:
            pass