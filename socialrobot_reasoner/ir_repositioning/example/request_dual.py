#!/usr/bin/env python
import numpy as np
import rospy
from vision_msgs.msg import BoundingBox2D
from ir_repositioning.srv import Repositioning, RepositioningRequest
"""
[SERVICE RUN]
roslaunch ir_repositioning ir_server.launch

[REQUEST EXAMPLE]
rosrun ir_repositioning request.py
"""
"""
def pose_to_se2(pose):
    if type(pose) is Pose:
        _x = pose.position.x
        _y = pose.position.y
        _quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    elif type(pose) is Transform:
        _x = pose.translation.x
        _y = pose.translation.y
        _quat = [
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z,
            pose.rotation.w,
        ]
    else:
        raise TypeError("Unknown type!!")
    _theta = euler_from_quaternion(_quat, axes="rzxy")[0]
    return (_x, _y, _theta)


def bb3d_to_bb2d(bb3d):
    bb2d = vis_msg.BoundingBox2D()
    _x, _y, _theta = pose_to_se2(bb3d.center)
    bb2d.center.x = _x
    bb2d.center.y = _y
    bb2d.center.theta = _theta
    bb2d.size_x = bb3d.size.x
    bb2d.size_y = bb3d.size.y
    return bb2d
"""

NORMAL = False


def request():
    req = RepositioningRequest()
    req.hand_type = req.DUAL_HAND
    req.dual_hand_width = 0.2

    req.Pt.x = 0.0
    req.Pt.y = 0.0

    object_direction = -15.0  # deg

    box = BoundingBox2D()
    box.center.x = 0.0
    box.center.y = 0.0
    box.center.theta = np.radians(object_direction)
    box.size_x = 0.1
    box.size_y = 0.1
    req.Obs = [box]

    req.section_definition.x = 0.05
    req.section_definition.y = 1.0
    req.section_definition.z = 0.02
    req.max_dist = 1024.0
    req.collision_offset = 0.2

    # req.Cr.x = np.radians(0.0)
    # req.Cr.y = np.radians(30.1)

    if NORMAL:
        req.Ct.x = np.radians(object_direction - 10)
        req.Ct.y = np.radians(object_direction + 10)
        req.strict_dual = False
    else:  # STRICT
        req.Ct.x = np.radians(object_direction)
        req.strict_dual = True

    rospy.wait_for_service("/ir_server/find_positions")
    try:
        feasi_proxy = rospy.ServiceProxy("/ir_server/find_positions", Repositioning)
        resp = feasi_proxy(req)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    resp = request()
    print(resp)