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
    req.hand_type = req.RIGHT_HAND

    sink = BoundingBox2D()
    sink.center.x = -1.2
    sink.center.y = 0.25
    sink.center.theta = np.radians(90.0)
    sink.size_x = 0.5
    sink.size_y = 2.0

    fridge = BoundingBox2D()
    fridge.center.x = 0.25
    fridge.center.y = 0.25
    fridge.center.theta = np.radians(90.0)
    fridge.size_x = 0.5
    fridge.size_y = 0.5

    juice = BoundingBox2D()
    juice.center.x = 0.25
    juice.center.y = 0.1
    juice.center.theta = np.radians(90.0)
    juice.size_x = 0.07
    juice.size_y = 0.07

    req.Obs = [sink, fridge, juice]
    # req.Obs = [fridge, juice]
    req.Pt.x = juice.center.x
    req.Pt.y = juice.center.y

    # IRM Config
    req.Cr.x = np.radians(-3.0)
    req.Cr.y = np.radians(3.0)
    req.Ct.x = np.radians(90.0 - 3.0)
    req.Ct.y = np.radians(90.0 + 3.0)

    req.section_definition.x = 0.05
    req.section_definition.y = 1.0
    req.section_definition.z = 0.02
    req.max_dist = 1024.0
    req.collision_offset = 0.35
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