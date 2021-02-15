#!/usr/bin/env python

import random
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs import msg as geo_msg
from visualization_msgs.msg import MarkerArray, Marker

from relocation_node import srv as reloc_srv


RED = ColorRGBA(1.0, 0.0, 0.0, 1.0)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 1.0)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)
t_BLUE = ColorRGBA(0.0, 0.0, 1.0, 0.5)

DEFAULT_QUAT = geo_msg.Quaternion(0.0, 0.0, 0.0, 1.0)
DEFAULT_SCALE = geo_msg.Vector3(0.06, 0.06, 0.2)

TABLE_SCALE = geo_msg.Vector3(1.1342, 0.70887, 0.69) # W x D x H
TABLE_POS = geo_msg.Vector3(5.5001e-01, 8.8066e-06, 3.6501e-01) # x y z
TABLE_QUAT = geo_msg.Quaternion(0.0, 0.0, 0.0, 1.0)

def _create_marker_init():
    marker = Marker()
    marker.header.frame_id = "base_footprint"
    marker.header.stamp = rospy.Time()
    marker.ns = "my_namespace"
    marker.id = random.randint(0, 2048)
    marker.lifetime = rospy.Duration(5)
    return marker


def create_line(point_list, color_rgba):
    marker = _create_marker_init()
    marker.header.stamp = rospy.Time()
    # marker.type = Marker.LINE_LIST
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.color = color_rgba
    marker.pose.position = geo_msg.Point(0, 0, 0)
    marker.pose.orientation = DEFAULT_QUAT
    marker.points = [geo_msg.Point(p[0], p[1], p[2]) for p in point_list]
    marker.scale = geo_msg.Vector3(0.005, 0, 0)
    return marker


def create_marker(position, orientation, scale, color_rgba, shape):
    # http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
    marker = _create_marker_init()
    marker.type = shape
    marker.action = Marker.ADD
    marker.color = color_rgba
    marker.pose.position = position
    marker.pose.orientation = orientation
    marker.scale = scale
    return marker


class RelocObject(object):
    def __init__(self, radius, height, x, y):
        self.radius = radius
        self.height = height
        self.x = x
        self.y = y


def make_reqeust(obj_list, target_id=8):
    pub_msg = reloc_srv.relocate_env_srvRequest()
    pub_msg.robot_height = 0.075
    pub_msg.robot_pose = [0.05, -0.1065] #base (0,0), left(0, 0.1065), right(0,-0.1065)
    pub_msg.target_id = target_id
    pub_msg.N = len(obj_list)
    pub_msg.R = [obj.radius for obj in obj_list]
    pub_msg.H = [obj.height for obj in obj_list]
    pub_msg.X = [obj.x for obj in obj_list]
    pub_msg.Y = [obj.y for obj in obj_list]
    pub_msg.x_min = 0#-TABLE_SCALE.y/2 + TABLE_POS.x
    pub_msg.x_max = 0.8#TABLE_SCALE.y/2 + TABLE_POS.x
    pub_msg.y_min = -0.5#-TABLE_SCALE.x/2
    pub_msg.y_max = 0.5#TABLE_SCALE.x/2
    return pub_msg


def feasible_check_client():
    obj_list = [
        RelocObject(6.5000e-02, 2.3544e-01, +4.0000e-01, -1.5003e-02), #gotica
        RelocObject(6.5000e-02, 2.3544e-01, +3.0000e-01, +9.9997e-02), #red_gotica
        # RelocObject(0.030, 0.068, 0.475, 0.36975),
        # RelocObject(0.028, 0.070, 0.364, 0.26625),
        # RelocObject(0.026, 0.071, 0.306, 0.40975),
        # RelocObject(0.025, 0.069, 0.171, 0.16075),
        # RelocObject(0.030, 0.070, 0.430, 0.27425),
        # RelocObject(0.030, 0.073, 0.204, 0.28725),
        # RelocObject(0.029, 0.066, 0.209, 0.04225),
        # RelocObject(0.025, 0.066, 0.171, 0.38375),
    ]
    target = 0
    rospy.wait_for_service("relocation_srv")
    try:
        f_check_srv = rospy.ServiceProxy("relocation_srv", reloc_srv.relocate_env_srv)
        req = make_reqeust(obj_list, target_id=target)
        resp = f_check_srv(req)
        print('target_id:', target)
        print(resp)
        srv_visualize(req, resp, obj_list[target])
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def srv_visualize(request, response, original_target):
    # request
    req_pub = rospy.Publisher("relocation/request_view", MarkerArray, queue_size=10)
    req_markers = MarkerArray()
    for i in xrange(request.N):
        req_markers.markers.append(
            create_marker(
                geo_msg.Point(request.X[i], request.Y[i], TABLE_SCALE.z + request.H[i] / 2.0),
                DEFAULT_QUAT,
                geo_msg.Vector3(request.R[i], request.R[i], request.H[i]),
                WHITE,
                Marker.CYLINDER,
            )
        )
    req_markers.markers.append(
        create_marker(
            geo_msg.Point(request.x_min, request.y_min, TABLE_SCALE.z),
            DEFAULT_QUAT,
            geo_msg.Vector3(0.01, 0.01, 0.01),
            GREEN,
            Marker.CUBE,
        )
    )
    req_markers.markers.append(
        create_line(
            [
                (request.x_min, request.y_min, TABLE_SCALE.z),
                (request.x_min, request.y_max, TABLE_SCALE.z),
                (request.x_max, request.y_max, TABLE_SCALE.z),
                (request.x_max, request.y_min, TABLE_SCALE.z),
                (request.x_min, request.y_min, TABLE_SCALE.z),
            ],
            RED,
        )
    )

    # response
    resp_pub = rospy.Publisher("relocation/response_view", MarkerArray, queue_size=10)
    resp_markers = MarkerArray()
    resp_markers.markers.append(
        create_marker(
            geo_msg.Point(
                response.relocate_coordinates[0], response.relocate_coordinates[1], TABLE_SCALE.z
            ),
            DEFAULT_QUAT,
            geo_msg.Vector3(0.1, 0.1, 0.01),
            t_BLUE,
            Marker.CUBE,
        )
    )
    resp_markers.markers.append(
        create_marker(
            geo_msg.Point(original_target.x, original_target.y, TABLE_SCALE.z),
            DEFAULT_QUAT,
            geo_msg.Vector3(0.07, 0.07, 0.015),
            RED,
            Marker.CUBE,
        )
    )
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        req_pub.publish(req_markers)
        resp_pub.publish(resp_markers)
        r.sleep()


if __name__ == "__main__":
    rospy.init_node("TestingNode")
    print "Requesting for feasibility check in moveIT"
    feasible_check_client()
    rospy.spin()
