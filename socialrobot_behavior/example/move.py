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
from socialrobot_perception_msgs import msg as perception_msg

class MoveArmTest():

    def __init__(self):
        obj_topic = "/perception/objects"
        self.detected_objects = None

        # 
        rospy.Subscriber(obj_topic, perception_msg.Objects, self._callback_objects)
        rospy.sleep(3.0)

    def _callback_objects(self, data):        
        if data.detected_objects != []:
            self.detected_objects = list(data.detected_objects)        

    def get_motion(self, request):
        # if detected object replace
        if self.detected_objects != None:
            request.inputs.obstacles = []
            request.inputs.obstacle_ids = []
            request.inputs.targetObject = []
            # for obj in self.detected_objects:
            #     request.inputs.obstacle_ids.append(obj.name.data)
            #     request.inputs.obstacles.append(obj.bb3d)

        # get motion
        motion_srv = rospy.ServiceProxy('/behavior/get_motion', GetMotion)
        res = motion_srv(request)
        return res

    def set_motion(self, plan):
        # set behavior
        if plan.result:
            behavior_srv = rospy.ServiceProxy('/behavior/set_behavior', SetBehavior)
            behavior_req = SetBehaviorRequest()
            behavior_req.header.frame_id = plan_req.planner_name
            behavior_req.trajectory = plan.motion.jointTrajectory
            behavior_res = behavior_srv(behavior_req)
            return behavior_res


##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('behavior_example')
    robot_name = rosparam.get_param("/robot_name")
    example = MoveArmTest()


    plan_req = GetMotionRequest()
        
    # goal type
    goalType = "CARTESIAN_SPACE_GOAL"
    # goalType = "JOINT_SPACE_GOAL"

    if robot_name == 'skkurobot':
        # table
        obs = BoundingBox3D()
        c = Pose()
        c.position.x = 0.65000551939
        c.position.y = 8.80658626556e-06
        c.position.z = 0.36501121521
        c.orientation.x = 0
        c.orientation.y = 0
        c.orientation.z = 0.707
        c.orientation.w = 0.707
        obs.center = c
        v = Vector3()
        v.x = 1.1342161893844604
        v.y = 0.7088739275932312
        v.z = 0.6899999976158142
        obs.size = v
        plan_req.inputs.obstacles.append(obs)

        # juice
        obs1 = BoundingBox3D()
        c1 = Pose()
        c1.position.x = 0.514751791954
        c1.position.y = -0.057709980756
        c1.position.z = 0.828674316406
        c1.orientation.x = 1.31276510729e-05
        c1.orientation.y = 2.76817457845e-10
        c1.orientation.z = 6.49862139805e-17
        c1.orientation.w = 1.0
        obs1.center = c1
        v1 = Vector3()
        v1.x = 0.0824019983411
        v1.y = 0.079343996942
        v1.z = 0.238139986992
        obs1.size = v1
        plan_req.inputs.obstacles.append(obs1)

        # milk
        obs2 = BoundingBox3D()
        c2 = Pose()
        c2.position.x = 0.75
        c2.position.y = -0.0827030837536
        c2.position.z = 0.827354133129
        c2.orientation.x = 1.31329e-05
        c2.orientation.y = 2.77181e-10
        c2.orientation.z = -2.91633e-18
        c2.orientation.w = 1.0
        obs2.center = c2
        v2 = Vector3()
        v2.x = 0.0752059966326
        v2.y = 0.074767999351
        v2.z = 0.235440000892
        obs2.size = v2
        plan_req.inputs.obstacles.append(obs2)

        # target pose
        if goalType == "CARTESIAN_SPACE_GOAL":
            # cartesian space goal
            target_pose = Pose()
            target_pose.position.x = 0.35
            target_pose.position.y = -0.0110983
            target_pose.position.z = 0.813977
            target_pose.orientation.x = 0
            target_pose.orientation.y = -0.707
            target_pose.orientation.z = 0
            target_pose.orientation.w = 0.707
            plan_req.inputs.poseGoal = target_pose

        else:
            target_joint_state = JointState()
            target_joint_state.name = [
                'j1_joint', 'j2_joint', 'j3_joint', 'j4_joint', 'j5_joint', 'j6_joint', 'j7_joint'
            ]

            target_joint_state.position = [
                2.029279854423499, 1.0646964200569589, -0.17102309698723056, -1.9245484903916832, 0.46603118745835337,
                -1.6544888709705696, -1.0367903166736518
            ]
            plan_req.inputs.jointGoal = target_joint_state

    elif robot_name == 'social_robot':
        # red_gotica
        obs1 = BoundingBox3D()
        c1 = Pose()
        c1.position.x = 0.375
        c1.position.y = 0.214997
        c1.position.z = 0.828779
        c1.orientation.x = 1.31936e-05
        c1.orientation.y = 2.20794e-10
        c1.orientation.z = 6.07222e-07
        c1.orientation.w = 1
        obs1.center = c1
        v1 = Vector3()
        v1.x = 0.0618015
        v1.y = 0.059508
        v1.z = 0.23814
        obs1.size = v1

        # gotica
        obs2 = BoundingBox3D()
        c2 = Pose()
        c2.position.x = 0.385
        c2.position.y = 0
        c2.position.z = 0.827404
        c2.orientation.x = 1.31627e-05
        c2.orientation.y = 2.26816e-10
        c2.orientation.z = -1.15535e-18
        c2.orientation.w = 1.0
        obs2.center = c2
        v2 = Vector3()
        v2.x = 0.065
        v2.y = 0.065
        v2.z = 0.23544
        obs2.size = v2
        
        # table
        obs3 = BoundingBox3D()
        c = Pose()
        c.position.x = 0.550006
        c.position.y = 8.80659e-06
        c.position.z = 0.365011
        c.orientation.x = 0
        c.orientation.y = 0
        c.orientation.z = 0.707
        c.orientation.w = 0.707
        obs3.center = c
        v = Vector3()
        v.x = 1.1342161893844604
        v.y = 0.7088739275932312
        v.z = 0.6899999976158142
        obs3.size = v
        plan_req.inputs.obstacles.append(obs1)
        plan_req.inputs.obstacles.append(obs2)
        plan_req.inputs.obstacles.append(obs3)

        # target pose
        if goalType == "CARTESIAN_SPACE_GOAL":
            # cartesian space goal
            # LHand_Base frame!!
            target_pose = Pose()
            # target_pose.position.x = 0.35
            # target_pose.position.y = -0.05265
            # target_pose.position.z = 0.847
            # target_pose.orientation.x = 0.707
            # target_pose.orientation.y = 0.0
            # target_pose.orientation.z = 0.707
            # target_pose.orientation.w = 0.0
            target_pose.position.x = 0.385361867708
            target_pose.position.y = -0.0250000310345
            target_pose.position.z = 0.801790819556
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 0.707106781187
            target_pose.orientation.z = -8.65956056235e-17
            target_pose.orientation.w = 0.707106781187
            plan_req.inputs.poseGoal = target_pose

        else:
            target_joint_state = JointState()
            target_joint_state.name = [
                'Waist_Roll', 'Waist_Pitch', 'Shoulder_Pitch', 'Shoulder_Roll', 'Elbow_Pitch', 'Elbow_Yaw',
                'Wrist_Pitch', 'Wrist_Roll'
            ]

            target_joint_state.position = [0, 0, 0, 1.45, 0, 0, 0, 0]
            plan_req.inputs.jointGoal = target_joint_state

    # arm type
    plan_req.inputs.targetBody = plan_req.inputs.RIGHT_ARM
    plan_req.planner_name = "movearm"


    # add obstacles 
    #plan_req.inputs.obstacle_ids = ['obj_red_gotica','obj_gotica', 'obj_table']
    # if robot grasping the object
    #plan_req.inputs.targetObject = ['obj_red_gotica']

    motion_plan = example.get_motion(plan_req)
    example.set_motion(motion_plan)


