#!/usr/bin/env python
import geometry_msgs
import rospy
import sys
from socialrobot_motion.srv import *
from vision_msgs.msg import *
from geometry_msgs.msg import *

class ArmMotionClient:
    left_group = ['Waist_Roll', 'Waist_Pitch', 'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll']
    right_group = ['Waist_Roll', 'Waist_Pitch', 'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll']

    def __init__(self):

        self.joint_states = None
        self.left_joint_states = None
        self.right_joint_states = None

            
    def _callback_joints(self, data):
        self.joint_states = data
        self.left_joint_states = sensor_msgs.msg.JointState()
        self.right_joint_states = sensor_msgs.msg.JointState()

        for i in self.left_group:
            idx = data.name.index(i)
            self.left_joint_states.header = data.header
            self.left_joint_states.name.append(i)
            self.left_joint_states.position.append(data.position[idx])

        
        for i in self.right_group:
            idx = data.name.index(i)
            self.right_joint_states.header = data.header
            self.right_joint_states.name.append(i)
            self.right_joint_states.position.append(data.position[idx])

    def plan(self):

        rospy.loginfo("wait for single arm motion planner.")
        rospy.wait_for_service("/motion_plan/move_arm")
        rospy.loginfo("single arm motion planner server is connected.")
        plan_srv = rospy.ServiceProxy("/motion_plan/move_arm", MotionPlan)

        #
        req = MotionPlanRequest()

        req.currentJointState.name = self.left_group
        req.currentJointState.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        req.targetPose = geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(0.287267, 0.187007, 0.828976),
                orientation=geometry_msgs.msg.Quaternion(
                    0.612095, -0.353509, 0.612709, 0.353497)
            )

        # obstacles
        # juice
        obs1 = BoundingBox3D()
        c1 = Pose()
        c1.position.x = +3.0000e-01 
        c1.position.y = +1.6500e-01
        c1.position.z = +8.2886e-01
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

        # milk
        obs2 = BoundingBox3D()
        c2 = Pose()
        c2.position.x = +3.2500e-01
        c2.position.y = -3.0762e-06
        c2.position.z = +8.2750e-01
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
        
        req.obstacle_ids = ['obj_juice', 'obj_milk', 'obj_table']
        req.obstacles = [obs1, obs2, obs3]

        plan_srv(req)

if __name__ == '__main__':
    rospy.init_node('single_arm_planner_example', anonymous=True)
    client = ArmMotionClient()

    #
    # rospy.loginfo("wait for joint states")
    # while(not rospy.is_shutdown() and not client.left_joint_states):
    #     continue    
    # rospy.loginfo("got the joint states")

    #
    client.plan()

