import numpy as np
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs import msg as geo_msg
from visualization_msgs.msg import Marker

RED = ColorRGBA(1.0, 0.0, 0.0, 1.0)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 1.0)
BLUE = ColorRGBA(0.0, 0.0, 1.0, 1.0)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 1.0)
PURPLE = ColorRGBA(0.78, 0.2, 1.0, 1.0)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)

t_DARK = ColorRGBA(0.1, 0.1, 0.1, 0.5)
t_RED = ColorRGBA(1.0, 0.0, 0.0, 0.5)
t_GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.5)
t_BLUE = ColorRGBA(0.0, 0.0, 1.0, 0.5)
t_YELLOW = ColorRGBA(1.0, 1.0, 0.0, 0.5)
t_PURPLE = ColorRGBA(0.78, 0.2, 1.0, 0.5)
t_WHITE = ColorRGBA(1.0, 1.0, 1.0, 0.5)
t_NO_COLOR = ColorRGBA(1.0, 1.0, 1.0, 0.0)

DEFAULT_QUAT = geo_msg.Quaternion(0.0, 0.0, 0.0, 1.0)


def _create_marker_init(_id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time()
    marker.ns = "my_namespace"
    marker.id = _id
    marker.action = marker.MODIFY
    return marker


def create_marker(_id, position, orientation, scale, color_rgba, shape):
    # http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
    marker = _create_marker_init(_id)
    if type(color_rgba) != ColorRGBA:
        color_rgba = ColorRGBA(color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3])
    if type(position) != geo_msg.Point:
        position = geo_msg.Point(position[0], position[1], position[2])
    if type(orientation) != geo_msg.Quaternion:
        orientation = geo_msg.Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
    if type(scale) != geo_msg.Vector3:
        scale = geo_msg.Vector3(scale[0], scale[1], scale[2])
    marker.type = shape
    marker.color = color_rgba
    marker.pose.position = position
    marker.pose.orientation = orientation
    marker.scale = scale
    return marker


def _create_special_marker(_id, points, colors_rgba):
    """
    special_marker is not a single marker. It is a list.
    colors_rgba = a list of ColorRGBA
    """
    colors_rgba_ = colors_rgba    
    if isinstance(colors_rgba, list) or isinstance(colors_rgba, np.ndarray):
        # Array likes
        if type(colors_rgba[0]) != ColorRGBA:
            colors_rgba_ = [ColorRGBA(c[0], c[1], c[2], c[3]) for c in colors_rgba]
    else:
        # Single value
        if type(colors_rgba) != ColorRGBA:
            r, g, b, a = colors_rgba
            colors_rgba_ = [ColorRGBA(r, g, b, a) for _ in points]
        else:
            colors_rgba_ = [colors_rgba for _ in points]    
    marker = _create_marker_init(_id)
    marker.pose.position = geo_msg.Point(0, 0, 0)
    marker.pose.orientation = DEFAULT_QUAT
    marker.colors = colors_rgba_
    marker.points = [geo_msg.Point(p[0], p[1], p[2]) for p in points]
    return marker


def create_line(_id, points, colors_rgba, size=0.005):
    """
    only scale.x is used and it controls the width of the line segments.
    """
    marker = _create_special_marker(_id, points, colors_rgba)
    marker.type = Marker.LINE_STRIP
    marker.scale = geo_msg.Vector3(size, 0, 0)
    return marker


def create_points(_id, points, colors_rgba, size=0.01):
    """
    scale.x is point width, scale.y is point height
    """
    marker = _create_special_marker(_id, points, colors_rgba)
    marker.type = Marker.POINTS
    marker.scale = geo_msg.Vector3(size, size, 0)
    return marker
