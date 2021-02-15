#!/usr/bin/env python
import roslib
roslib.load_manifest('socialrobot_behavior')

import rospy
import rosparam
from tf.transformations import quaternion_from_euler
from vision_msgs.msg import BoundingBox3D
from geometry_msgs.msg import Pose
from socialrobot_behavior import srv as behavior_srv


def make_ros_pose(position, euler):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2], axes="rzxy")
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose


def make_obstacle(position, euler, size):
    obs = BoundingBox3D()
    obs.center = make_ros_pose(position, euler)
    obs.size.x = size[0]
    obs.size.y = size[1]
    obs.size.z = size[2]
    return obs


def make_obstacle_dict():
    # position, euler, size
    # return {
    #     "obj_juice": make_obstacle(
    #         (0.40, 0.14, 0.825),
    #         (30 * math.pi / 180, 0, 0),
    #         (0.082402, 0.079344, 0.23814),
    #     ),
    #     "obj_milk": make_obstacle(
    #         (0.40, -0.24, 0.825),
    #         (0, 0, 0),
    #         (0.082402, 0.079344, 0.23814),
    #     ),
    #     "obj_table": make_obstacle(
    #         (0.575006, 0.0, 0.68),
    #         (0, 0, 0),
    #         (0.708903, 1.13422, 0.02),
    #     ),
    # }
    # red_gotica
    obs1 = BoundingBox3D()
    obs1.center.position.x = +3.0000e-01
    obs1.center.position.y = +9.9997e-02
    obs1.center.position.z = +8.2886e-01
    obs1.center.orientation.x = 1.31936e-05
    obs1.center.orientation.y = 2.20794e-10
    obs1.center.orientation.z = 6.07222e-07
    obs1.center.orientation.w = 1
    obs1.size.x = 0.0618015
    obs1.size.y = 0.059508
    obs1.size.z = 0.23814
    # gotica
    obs2 = BoundingBox3D()
    obs2.center.position.x = +4.0000e-01
    obs2.center.position.y = -1.5003e-02
    obs2.center.position.z = +8.2886e-01
    obs2.center.orientation.x = 1.31627e-05
    obs2.center.orientation.y = 2.26816e-10
    obs2.center.orientation.z = -1.15535e-18
    obs2.center.orientation.w = 1.0
    obs2.size.x = 0.065
    obs2.size.y = 0.065
    obs2.size.z = 0.23544
    # table
    obs3 = BoundingBox3D()
    obs3.center.position.x = 0.550006
    obs3.center.position.y = 8.80659e-06
    obs3.center.position.z = 0.365011
    obs3.center.orientation.x = 0
    obs3.center.orientation.y = 0
    obs3.center.orientation.z = 0.707
    obs3.center.orientation.w = 0.707
    obs3.size.x = 1.1342161893844604
    obs3.size.y = 0.7088739275932312
    obs3.size.z = 0.6899999976158142
    return {"obj_red_gotica": obs1, "obj_gotica": obs2, "obj_table": obs3}


if __name__ == '__main__':
    rospy.init_node('behavior_example')
    robot_name = rosparam.get_param("/robot_name")

    plan_req = behavior_srv.GetMotionRequest()
    plan_req.planner_name = "push"

    # set gripper and action
    plan_req.inputs.targetBody = plan_req.inputs.LEFT_ARM
    plan_req.inputs.approachDirection = plan_req.inputs.APPROACH_SIDE

    if robot_name == 'social_robot':
        obstacles = make_obstacle_dict()
        plan_req.inputs.obstacle_ids = obstacles.keys()
        plan_req.inputs.obstacles = obstacles.values()
        plan_req.inputs.targetObject = ["obj_red_gotica"]
    else:
        raise NotImplementedError

    # Get motion
    motion_proxy = rospy.ServiceProxy('/behavior/get_motion', behavior_srv.GetMotion)
    res = motion_proxy(plan_req)

    # Set behavior
    if res.result:
        behavior_proxy = rospy.ServiceProxy('/behavior/set_behavior', behavior_srv.SetBehavior)
        behavior_req = behavior_srv.SetBehaviorRequest()
        behavior_req.header.frame_id = plan_req.planner_name
        behavior_req.trajectory = res.motion.jointTrajectory
        behavior_res = behavior_proxy(behavior_req)


