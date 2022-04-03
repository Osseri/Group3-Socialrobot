#!/usr/bin/env python
import numpy as np
import rospy
from tf import transformations

# move base
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
from socialrobot_msgs import msg as social_robot_msg

# pure_hybrid_astar
from pure_hybrid_astar.srv import PureHybridAstar, PureHybridAstarRequest, PureHybridAstarResponse

class PlanTest():

    def __init__(self):
        self.detected_objects = None
        self.objects_from_robot = None
        self.objects_from_camera = None

        self.srv_name = "/pure_hybrid_astar/find_path"
        rospy.wait_for_service(self.srv_name)

        # perception topics
        rospy.Subscriber("/socialrobot/perception/objects", social_robot_msg.Objects, self._callback_objects)
        rospy.Subscriber("/socialrobot/qr_tracker/objects", social_robot_msg.Objects, self._callback_ex_objects)
        rospy.sleep(1.0)

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

    def get_motion(self):
        ## 
        if self.objects_from_robot != None:
            self.detected_objects = self.objects_from_camera + self.objects_from_robot
        if self.detected_objects == []:
            self.add_objects()
        
        obstacles = []
        for obj in self.detected_objects:
            obstacles.append(obj.bb3d)

        return self.request(obstacles)

    def request(self, obstacles=None):
        req = PureHybridAstarRequest()
        req.motion_model = req.HOLONOMIC
        # req.motion_model = req.MONOCYCLE
        # req.motion_model = req.BICYCLE
        req.backward_motion = True
        req.timeout_in_sec = 30.0
        req.ws_xmin = -1.0
        req.ws_xmax = 2.0
        req.ws_ymin = -1.0
        req.ws_ymax = 1.0

        req.robot_width_meter = 0.55

        req.goal.x = +1.1523e+00
        req.goal.y = -3.0888e-01
        r,p,y = transformations.euler_from_quaternion([0, 0, -0.805, 0.593])
        req.goal.theta = y
        req.ignore_goal_orientation = False

        if obstacles:
            req.obstacles = obstacles

        try:
            pusher_proxy = rospy.ServiceProxy(self.srv_name, PureHybridAstar)
            resp = pusher_proxy(req)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return PureHybridAstarResponse()


if __name__ == "__main__":
    rospy.init_node("hybrid_astar_example")

    planner = PlanTest()
    res = planner.get_motion()

    # publish results
    iter=0
    while(not rospy.is_shutdown() and iter<100):
        planner.path_pub.publish(res.robot_path)
        iter+=1