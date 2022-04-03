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


def request():
    req = RepositioningRequest()
    req.hand_type = req.LEFT_HAND

    req.Pt.x = 0.0
    req.Pt.y = 0.0

    box = BoundingBox2D()
    box.center.x = 0.0
    box.center.y = 0.0
    box.center.theta = np.radians(0.0)
    box.size_x = 0.3
    box.size_y = 0.5
    req.Obs = [box]

    req.Cr.x = np.radians(0.0)
    req.Cr.y = np.radians(30.1)

    # req.Ct.x = np.radians(-180)
    # req.Ct.y = np.radians(180)
    req.Ct.x = np.radians(-3)
    req.Ct.y = np.radians(3)

    req.section_definition.x = 0.05
    req.section_definition.y = 1.0
    req.section_definition.z = 0.02

    req.max_dist = 1024.0
    req.collision_offset = 0.3
    req.strict_dual = False

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
