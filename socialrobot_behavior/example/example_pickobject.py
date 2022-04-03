#!/usr/bin/env python
import roslib
roslib.load_manifest('socialrobot_behavior')

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
from tf.transformations import quaternion_from_euler, quaternion_matrix

from math import *
import numpy as np

def calc_pose_offset(pose, quaternion, offset):
    # find tf matrix by pose's orientation
    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    transf_m = quaternion_matrix(quaternion)

    # find hand z axis
    # When gripper is forward, z-axis is [0, 0, -1].
    # hand z_vector is [0, 0, -1].
    # transf_m(3x3) * [0, 0, -1] = -transf_m[*][2]
    # So, the length of z_vector is 1.
    z_vector = np.array([-transf_m[0][2], -transf_m[1][2], -transf_m[2][2]])
    offset_vector = offset * z_vector

    offset_pose = Pose()
    offset_pose.position.x = pose.position.x + offset_vector[0]
    offset_pose.position.y = pose.position.y + offset_vector[1]
    offset_pose.position.z = pose.position.z + offset_vector[2]
    offset_pose.orientation = pose.orientation
    return offset_pose

class AppraochArmTest():

    def __init__(self):
        obj_topic = "/socialrobot/perception/objects"
        self.detected_objects = None

        # 
        rospy.Subscriber(obj_topic, social_robot_msg.Objects, self._callback_objects)
        rospy.sleep(1.0)

    def _callback_objects(self, data):        
        if data.detected_objects != []:
            self.detected_objects = data.detected_objects
        return

    def get_motion(self, target_object, request):
        # if detected object replace
        if self.detected_objects != None:
            for obj in self.detected_objects:
                request.requirements.dynamic_object.append(obj)
                if obj.id == target_object:
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

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('behavior_example')
    robot_name = rosparam.get_param("/robot_name")
    example = AppraochArmTest()

    plan_req = GetMotionRequest()
    plan_req.requirements.name = "pickobject"
    target_object = 'obj_red_gotica'

    # arm type
    plan_req.requirements.robot_group = [plan_req.requirements.LEFT_ARM]

    # add obstacles from topic
    motion_plan = example.get_motion(target_object, plan_req)
    example.set_motion(motion_plan)
