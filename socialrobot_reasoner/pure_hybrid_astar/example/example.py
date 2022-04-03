#!/usr/bin/env python
import csv
import time
import rospkg

import numpy as np
import rospy
from tf import transformations

# move arm
from socialrobot_msgs import msg as so_msg

# move base
from geometry_msgs import msg as geo_msg
from nav_msgs import msg as nav_msg
from socialrobot_hardware import srv as ha_srv

# pure_hybrid_astar
from pure_hybrid_astar.srv import PureHybridAstar, PureHybridAstarRequest
"""
[SERVICE RUN]
roslaunch pure_hybrid_astar pure_hybrid_astar.launch

[REQUEST EXAMPLE]
rosrun pure_hybrid_astar example.py
"""



def save_logging(robot_path):

    def navPath_to_XYQuat(path):
        xyqs = []  # x, y, qx, qy, qz, qw
        for pose_stamped in path.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            qx = pose_stamped.pose.orientation.x
            qy = pose_stamped.pose.orientation.y
            qz = pose_stamped.pose.orientation.z
            qw = pose_stamped.pose.orientation.w
            xyqs.append((x, y, qx, qy, qz, qw))
        return xyqs

    rospack = rospkg.RosPack()
    dstr = rospack.get_path('pure_hybrid_astar') + '/example/log/'
    tstr = time.strftime("%Y%m%d_%I-%M%p")
    with open('{}{}(hybrid_astar)xyquat.csv'.format(dstr, tstr), 'w') as f:
        writer = csv.writer(f)
        writer.writerows(navPath_to_XYQuat(robot_path))


def XYTs_to_navPath(xyrad_list, frame_id):
    nav_path = nav_msg.Path()
    nav_path.header.frame_id = frame_id
    for x, y, rad in xyrad_list:
        ps = geo_msg.PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = x
        ps.pose.position.y = y
        quat = transformations.quaternion_about_axis(rad, (0, 0, 1))
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        nav_path.poses.append(ps)
    return nav_path


def request():
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

    req.goal.x = 1.0
    req.goal.y = 0.0
    req.goal.theta = np.radians(0)
    req.ignore_goal_orientation = False

    srv_name = "/pure_hybrid_astar/find_path"
    rospy.wait_for_service(srv_name)
    try:
        pusher_proxy = rospy.ServiceProxy(srv_name, PureHybridAstar)
        resp = pusher_proxy(req)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    rospy.init_node("hybrid_astar_example")

    resp = request()
    path_length = resp.path_length
    r_path = resp.robot_path
    save_logging(r_path)
    rospy.loginfo(r_path)
    rospy.loginfo("------------")
    rospy.loginfo("Complete!")



# TODO: reasonar branch to master