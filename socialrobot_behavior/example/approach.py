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

class AppraochArmTest():

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
            for obj in self.detected_objects:
                request.inputs.obstacle_ids.append(obj.name.data)
                request.inputs.obstacles.append(obj.bb3d)

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
    example = AppraochArmTest()

    plan_req = GetMotionRequest()
    plan_req.planner_name = "approach"

    # add obstacles
    #SKKU robot
    if robot_name == 'skkurobot':
        # juice
        obs1 = BoundingBox3D()
        c1 = Pose()
        c1.position.x = +5.0000e-01
        c1.position.y = +4.0000e-03
        c1.position.z = +8.5935e-01
        c1.orientation.x = 0
        c1.orientation.y = 0
        c1.orientation.z = 0
        c1.orientation.w = 1.0
        obs1.center = c1
        v1 = Vector3()
        v1.x = 0.082402
        v1.y = 0.079344
        v1.z = 0.23814
        obs1.size = v1

        # milk
        obs2 = BoundingBox3D()
        c2 = Pose()
        c2.position.x = +4.9600e-01
        c2.position.y = +2.7000e-01
        c2.position.z = +8.6935e-01
        c2.orientation.x = 0
        c2.orientation.y = 0
        c2.orientation.z = 0
        c2.orientation.w = 1.0
        obs2.center = c2
        v2 = Vector3()
        v2.x = 0.082402
        v2.y = 0.079344
        v2.z = 0.23814
        obs2.size = v2

        # cup
        obs4 = BoundingBox3D()
        c4 = Pose()
        c4.position.x = +7.7000e-01
        c4.position.y = +1.1000e-02
        c4.position.z = +8.0967e-01
        c4.orientation.x = 0
        c4.orientation.y = 0
        c4.orientation.z = 0
        c4.orientation.w = 1.0
        obs4.center = c4
        v4 = Vector3()
        v4.x = 8.8176e-02
        v4.y = 8.8176e-02
        v4.z = 1.2334e-01
        obs4.size = v4        
        
        # mug
        obs5 = BoundingBox3D()
        c5 = Pose()
        c5.position.x = +7.7500e-01
        c5.position.y = +2.5100e-01
        c5.position.z = +8.0633e-01
        c5.orientation.x = 0
        c5.orientation.y = 0
        c5.orientation.z = 0
        c5.orientation.w = 1.0
        obs5.center = c5
        v5 = Vector3()
        v5.x = 0.082402
        v5.y = 0.079344
        v5.z = 0.23814
        obs5.size = v5

        # table
        obs3 = BoundingBox3D()
        c = Pose()
        c.position.x = +6.4201e-01
        c.position.y = +8.8066e-06
        c.position.z = +3.8001e-01
        c.orientation.x = 0
        c.orientation.y = 0
        c.orientation.z = 0.707
        c.orientation.w = 0.707
        obs3.center = c
        v = Vector3()
        v.x = 1.2000e+00
        v.y = 7.4997e-01
        v.z = 7.3000e-01
        obs3.size = v

        plan_req.inputs.obstacles.append(obs1)
        plan_req.inputs.obstacles.append(obs2)
        plan_req.inputs.obstacles.append(obs4)
        plan_req.inputs.obstacles.append(obs5)
        plan_req.inputs.obstacles.append(obs3)

        plan_req.inputs.obstacle_ids = ['obj_juice', 'obj_milk','obj_cup','obj_mug','obj_table']
        plan_req.inputs.targetObject = ['obj_juice']

    #Social_robot
    elif robot_name == 'social_robot':
        # red_gotica
        obs1 = BoundingBox3D()
        c1 = Pose()
        c1.position.x = +3.0000e-01
        c1.position.y = +2.0000e-01
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

        # gotica
        obs2 = BoundingBox3D()
        c2 = Pose()
        c2.position.x = +4.0000e-01
        c2.position.y = -1.5003e-02
        c2.position.z = +8.2886e-01
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

        # obj_bakey
        obs3 = BoundingBox3D()
        c3 = Pose()
        c3.position.x = +3.0000e-01
        c3.position.y = -9.9997e-02
        c3.position.z = +8.2886e-01
        c3.orientation.x = 1.31936e-05
        c3.orientation.y = 2.20794e-10
        c3.orientation.z = 6.07222e-07
        c3.orientation.w = 1
        obs3.center = c3
        v3 = Vector3()
        v3.x = 0.0618015
        v3.y = 0.059508
        v3.z = 0.23814
        obs3.size = v3

        # table
        obs4 = BoundingBox3D()
        c4 = Pose()
        c4.position.x = 0.550006
        c4.position.y = 8.80659e-06
        c4.position.z = 0.365011
        c4.orientation.x = 0
        c4.orientation.y = 0
        c4.orientation.z = 0.707
        c4.orientation.w = 0.707
        obs4.center = c4
        v4 = Vector3()
        v4.x = 1.1342161893844604
        v4.y = 0.7088739275932312
        v4.z = 0.6899999976158142
        obs4.size = v4

    # arm type
    plan_req.inputs.targetBody = plan_req.inputs.LEFT_ARM

    # set approach direction
    plan_req.inputs.approachDirection = plan_req.inputs.APPROACH_SIDE
    plan_req.inputs.approachPoint = plan_req.inputs.APPROACH_SIDE

    # add obstacles

    motion_plan = example.get_motion(plan_req)
    example.set_motion(motion_plan)
