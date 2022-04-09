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

class TransferObjectTest():

    def __init__(self):
        self.detected_objects = []
        self.objects_from_robot = []
        self.objects_from_camera = []

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

    def get_motion(self, target_object, goal_position, request):
        self.detected_objects = []
        if self.objects_from_robot != None:
            self.detected_objects += self.objects_from_camera
        if self.objects_from_robot != None:
            self.detected_objects += self.objects_from_robot

        # if self.detected_objects == None:
        #     self.add_objects()
        
        for obj in self.detected_objects:
            request.requirements.dynamic_object.append(obj)
            if obj.id == target_object:
                request.requirements.target_object.append(obj)
        for obj in self.detected_objects:
            request.requirements.dynamic_object.append(obj)
            if obj.id == goal_position:
                request.requirements.target_object.append(obj)
        
        # get motion
        motion_srv = rospy.ServiceProxy('/behavior/get_motion', GetMotion)
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
        
        object1 = social_robot_msg.Object()
        object1.id = "obj_juice"
        obs1 = BoundingBox3D()
        c1 = Pose()
        c1.position.x = 0.374999523163
        c1.position.y = -0.315002858639
        c1.position.z = 0.826801657677
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

        object2 = social_robot_msg.Object()
        object2.id = "obj_human"
        obs2 = BoundingBox3D()
        c2 = Pose()
        c2.position.x = 1.1299674511
        c2.position.y = -0.000323602405842
        c2.position.z = 1.17559015751
        c2.orientation.x = 0.0
        c2.orientation.y = 0.0
        c2.orientation.z = 1.0
        c2.orientation.w = 0.0
        obs2.center = c2
        v2 = Vector3()
        v2.x = 0.256468266249
        v2.y = 0.503938555717
        v2.z = 0.590130090714
        obs2.size = v2
        object2.bb3d = obs2

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


        self.detected_objects = [object1, object2, object3, object4]

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('behavior_example')
    robot_name = rosparam.get_param("/robot_name")
    example = TransferObjectTest()
    rospy.sleep(1.0)

    plan_req = GetMotionRequest()
    plan_req.requirements.name = "transferobject"
    target_object = 'obj_gotica'
    goal_position = 'obj_human'

    # arm type
    plan_req.requirements.robot_group = [plan_req.requirements.RIGHT_ARM]

    # set velocity constraint
    #plan_req.requirements.constraints.append('slow')

    # add obstacles from topic
    motion_plan = example.get_motion(target_object, goal_position, plan_req)
    example.set_motion(motion_plan)

    print(motion_plan.motion.jointTrajectory.joint_names)
    for pos in motion_plan.motion.jointTrajectory.points:
        print(pos.positions)