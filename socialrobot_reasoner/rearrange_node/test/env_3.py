#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs import msg as geo_msg
from visualization_msgs.msg import MarkerArray, Marker

from rearrange_node import msg as rearr_msg
from rearrange_node import srv as rearr_srv

RED = ColorRGBA(1.0, 0.0, 0.0, 1.0)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 1.0)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)
t_BLUE = ColorRGBA(0.0, 0.0, 1.0, 0.5)

DEFAULT_QUAT = (0.0, 0.0, 0.0, 1.0)
DEFAULT_SCALE = (6.5000e-02, 6.5000e-02, 2.3544e-01)


def get_env_object_info(name, position, orientation, scale):
    return rearr_msg.env_object_info_msg(
        object_name=[name,],
        object_position=geo_msg.Point(position[0], position[1], position[2]),
        object_orientation=geo_msg.Quaternion(
            x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]
        ),
        object_scale=geo_msg.Vector3(scale[0], scale[1], scale[2]),
    )


def get_request():
    req = rearr_srv.rearrange_env_srvRequest()
    # end-effector x-y direction
    req.robot_pose = [0.0, 0.6245] # left eef x-y position = 0.6245
                                   # right eef x-y position = 0.6245
    # Grasp target
    req.target = get_env_object_info(
        "obj_milk", (0.63989675045, -0.119378715754, 0.939548969269), DEFAULT_QUAT, DEFAULT_SCALE
    )
    # The obstacles on the workspace object
    req.objects = [
        get_env_object_info(
            "obj_juice", (0.54824757576, -0.204381078482, 0.940912604332), 
            DEFAULT_QUAT, 
            DEFAULT_SCALE
        )
    ]
    # The workspace object
    req.workspace = get_env_object_info(
        "table", (0.660645604134, -0.202193677425, 0.818049907684), 
        (0.0, 0.0, 0.995976030827, 0.0896202921867), 
        (0.310000002384, 0.395000010729, 0.00800000037998)
    )
    return req


def create_marker(position, orientation, scale, color_rgba, shape):
    import random

    # http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
    marker = Marker()
    marker.header.frame_id = "base_footprint"
    marker.header.stamp = rospy.Time()
    marker.ns = "my_namespace"
    marker.id = random.randint(0, 2048)
    marker.lifetime = rospy.Duration(1)

    marker.type = shape
    marker.action = marker.ADD
    marker.color = color_rgba

    marker.pose.position = position
    marker.pose.orientation = orientation
    marker.scale = scale
    return marker


def env_object_to_marker(info, shape, color_rgba):
    return create_marker(
        info.object_position,
        info.object_orientation,
        info.object_scale,
        color_rgba,
        shape,
    )


def srv_visualize(request, response):
    # request
    req_pub = rospy.Publisher(
        "/rearrange_node/markers", MarkerArray, queue_size=10, latch=True
    )
    req_markers = MarkerArray()
    req_markers.markers.append(
        env_object_to_marker(request.workspace, Marker.CUBE, WHITE)
    )
    req_markers.markers.append(
        env_object_to_marker(request.target, Marker.CYLINDER, GREEN)
    )
    for obstacle in request.objects:
        req_markers.markers.append(env_object_to_marker(obstacle, Marker.CYLINDER, RED))

    # response
    resp_pub = rospy.Publisher(
        "/rearrange_node/markers", MarkerArray, queue_size=10, latch=True
    )
    resp_markers = MarkerArray()
    for point in response.rearrange_positions:
        resp_markers.markers.append(
            create_marker(
                point,
                geo_msg.Quaternion(0, 0, 0, 1),
                geo_msg.Vector3(0.06, 0.06, 0.2),
                t_BLUE,
                Marker.CYLINDER,
            )
        )

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        req_pub.publish(req_markers)
        resp_pub.publish(resp_markers)
        r.sleep()


def example():
    rospy.wait_for_service("rearrange_srv")
    try:
        f_check_srv = rospy.ServiceProxy("rearrange_srv", rearr_srv.rearrange_env_srv)
        req = get_request()
        print(req)
        resp = f_check_srv(req)
        print resp
        srv_visualize(req, resp)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == "__main__":
    rospy.init_node("rearrange_test")
    print "Example"
    example()
    rospy.spin()
