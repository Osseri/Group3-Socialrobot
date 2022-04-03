#!/usr/bin/env python
import csv
import time
import rospkg

import numpy as np
import rospy
from tf import transformations

# move arm
from socialrobot_behavior import srv as be_srv
from socialrobot_msgs import msg as so_msg

# move base
from geometry_msgs import msg as geo_msg
from nav_msgs import msg as nav_msg
from socialrobot_hardware import srv as ha_srv

# pusher
from pusher_node.srv import PlanePushing, PlanePushingRequest
"""
[SERVICE RUN]
roslaunch pusher_node pusher_node.launch

[REQUEST EXAMPLE]
rosrun pusher_node example.py
"""

plan_proxy = rospy.ServiceProxy("/behavior/get_motion", be_srv.GetMotion)
act_proxy = rospy.ServiceProxy("/behavior/set_behavior", be_srv.SetBehavior)


def behavior_do(motion_request):
    def _behavior_get(motion_request):
        rospy.wait_for_service('/behavior/get_motion')
        res = plan_proxy(motion_request)
        rospy.loginfo("[/behavior/get_motion  ] Response: %s", res.result)
        return res

    def _behavior_set(behavior_get_response, planner_name):
        resp = behavior_get_response
        if resp.result:
            req = be_srv.SetBehaviorRequest()
            req.header.frame_id = planner_name
            req.behavior_name = planner_name
            req.trajectory = resp.motion.jointTrajectory
            set_resp = act_proxy(req)
            _waiting = 1.5
            rospy.loginfo("[/behavior/set_behavior] Wait %s secs.", _waiting)
            rospy.sleep(_waiting)
            result = set_resp.result == be_srv.SetBehaviorResponse.OK
            rospy.loginfo("[/behavior/set_behavior] Response: %s", result)
            return result
        else:
            return False

    get_result = _behavior_get(motion_request)
    rospy.logwarn(get_result.motion.jointTrajectory)
    planner_name = motion_request.requirements.name
    return _behavior_set(get_result, planner_name)

def set_arm_deg(left_joints, right_joints):
    lshoulder_pitch, lshoulder_roll, lelbow_pitch, lelbow_yaw, lwrist_pitch, lwrist_roll = left_joints
    rshoulder_pitch, rshoulder_roll, relbow_Pitch, relbow_Yaw, rwrist_Pitch, rwrist_Roll = right_joints

    req = be_srv.GetMotionRequest()
    req.requirements.name = "movearm"
    req.requirements.robot_group = [req.requirements.BOTH_ARM]

    _j = so_msg.Position()
    _j.joint_state.name = [
        'Waist_Roll',
        'Waist_Pitch',
        'LShoulder_Pitch',
        'LShoulder_Roll',
        'LElbow_Pitch',
        'LElbow_Yaw',
        'LWrist_Pitch',
        'LWrist_Roll',
        'RShoulder_Pitch', # -: fw
        'RShoulder_Roll',  # +: closer
        'RElbow_Pitch',    # +: closer
        'RElbow_Yaw',      # +: up
        'RWrist_Pitch',
        'RWrist_Roll',
    ]
    _j.joint_state.position = [
        0.0,
        0.0,
        np.radians(lshoulder_pitch),
        np.radians(lshoulder_roll),
        np.radians(lelbow_pitch),
        np.radians(lelbow_yaw),
        np.radians(lwrist_pitch),
        np.radians(lwrist_roll),
        np.radians(rshoulder_pitch),
        np.radians(rshoulder_roll),
        np.radians(relbow_Pitch),
        np.radians(relbow_Yaw),
        np.radians(rwrist_Pitch),
        np.radians(rwrist_Roll),
    ]
    req.requirements.goal_position = [_j]
    behavior_do(req)

def set_hand_deg(left_hand, right_hand):
    lf1, lf2, lf3 = left_hand
    rf1, rf2, rf3 = right_hand

    req = be_srv.GetMotionRequest()
    # req.requirements.name = "openhand"
    req.requirements.name = "closehand"
    req.requirements.robot_group = [req.inputs.BOTH_GRIPPER]
    req.requirements.constraints.append("POSITION")

    _j = so_msg.Position()
    _j.joint_state.name = [
        'LFinger_1', 'LFinger_2', 'LFinger_3',
        'RFinger_1', 'RFinger_2', 'RFinger_3',
    ]
    _j.joint_state.position = [
        np.radians(lf1), np.radians(lf2),  np.radians(lf3),
        np.radians(rf1), np.radians(rf2), np.radians(rf3),
    ]
    req.requirements.goal_position = [_j]
    behavior_do(req)

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

def reset_motion(back=None):
    # init arm, hand
    set_arm_deg([0, -73, 0, 0, 0, 0], [0, 73, 0, 0, 0, 0])
    set_hand_deg([0, 0, 0], [0, 0, 0])
    if back is not None:
        path = XYTs_to_navPath([(back, 0, 0)], "base_footprint")
        path_following(path)

def initial_motion(grasp=True):
    reset_motion()

    # TODO: go to initial base position
    # Right now, make sure the object is in front of the robot.

    # arm ready
    set_arm_deg([-9, -60, 10, -83, -40, -40], [-9, 60, 10, 83, -40, 40])
    # open
    set_hand_deg([-20, -20, -20], [-20, -20, 20])

    # go foward
    path = XYTs_to_navPath([(0.15, 0, 0)], "base_footprint")
    path_following(path)

    # arm closer
    set_arm_deg([-9, -65, 10, -83, -40, -40], [-9, 65, 10, 83, -40, 40])
    # grasp
    if grasp:
        set_hand_deg([12, 12, -16], [12, 12, 16])

def back_to_initial_motion():
    # open
    set_hand_deg([-20, -20, -20], [-20, -20, 20])
    # arm ready
    set_arm_deg([-9, -60, 10, -83, -40, -40], [-9, 60, 10, 83, -40, 40])

    # go backward
    path = XYTs_to_navPath([(-0.3, 0, 0)], "base_footprint")
    path_following(path)
    reset_motion()

def path_following(robot_path):
    """robot_path: nav_msgs/Path"""
    proxy = rospy.ServiceProxy("/set_path", ha_srv.SetPathTrajectory)

    req = ha_srv.SetPathTrajectoryRequest()
    req.trajectory = robot_path

    res = proxy(req)
    return res

def request():
    req = PlanePushingRequest()
    req.robot_width_meter = 0.55
    req.object_width_meter = 0.44

    from_robot_to_object = (req.robot_width_meter + req.object_width_meter) / 2.0
    print(from_robot_to_object)

    req.object_start.x = from_robot_to_object
    req.object_start.y = 0.0
    req.object_start.theta = 0.0
    req.object_goal.x = 0.
    req.object_goal.y = 2.0
    req.object_goal.theta = np.radians(180)
    req.ignore_goal_orientation = False

    srv_name = "/pusher_node/find_path"
    rospy.wait_for_service(srv_name)
    try:
        pusher_proxy = rospy.ServiceProxy(srv_name, PlanePushing)
        resp = pusher_proxy(req)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def save_logging(object_path, robot_path):

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
    dstr = rospack.get_path('pusher_node') + '/example/log/'
    tstr = time.strftime("%Y%m%d_%I-%M%p")
    with open('{}{}(chair_path)xyquat.csv'.format(dstr, tstr), 'w') as f:
        writer = csv.writer(f)
        writer.writerows(navPath_to_XYQuat(object_path))
    with open('{}{}(chair_path)xyquat.csv'.format(dstr, tstr), 'w') as f:
        writer = csv.writer(f)
        writer.writerows(navPath_to_XYQuat(robot_path))


if __name__ == "__main__":
    rospy.init_node("pusher_example")

    # reset_motion(back=-1.0)
    # initial_motion(grasp=False)
    initial_motion(grasp=True)

    # path = XYTs_to_navPath([(0., 0, np.radians(-45))], "base_footprint")
    # path_following(path)

    resp = request()
    path_length = resp.path_length
    o_path = resp.object_path
    r_path = resp.robot_path
    save_logging(o_path, r_path)

    res = path_following(r_path)
    rospy.logwarn(res)

    back_to_initial_motion()

    # resp = request()
    rospy.logwarn(len(resp.robot_path.poses))
    rospy.spin()


# TODO: reasonar branch to master