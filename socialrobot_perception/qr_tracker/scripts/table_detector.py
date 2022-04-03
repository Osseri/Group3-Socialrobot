#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs import msg as std_msg
from vision_msgs import msg as vision_msg
from geometry_msgs import msg as geo_msg
from socialrobot_msgs import msg as social_msg


def dot(a, b):
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w


def close_quaternion(anchor, query):
    if dot(anchor, query) < 0.0:  # not close
        query.x *= -1.0
        query.y *= -1.0
        query.z *= -1.0
        query.w *= -1.0
    # close
    return query


def normalize_quaternion(orientation):
    length = np.sqrt(
        orientation.x * orientation.x +
        orientation.y * orientation.y +
        orientation.z * orientation.z +
        orientation.w * orientation.w
    )
    n = 1.0 / length
    orientation.x *= n
    orientation.y *= n
    orientation.z *= n
    orientation.w *= n
    return orientation


def weighted_average_quat(prev, current, ratio_of_current):
    """orientation type"""
    new_o = close_quaternion(prev, current)
    prev.x += ratio_of_current * new_o.x
    prev.y += ratio_of_current * new_o.y
    prev.z += ratio_of_current * new_o.z
    prev.w += ratio_of_current * new_o.w
    return normalize_quaternion(prev)


class TableFilter:

    class TableInstance:
        def __init__(self):
            self.has_new = False
            self.data = geo_msg.Pose()
            self.data.orientation.x = 0.0
            self.data.orientation.y = 0.0
            self.data.orientation.z = 0.0
            self.data.orientation.w = 1.0

        def set_data(self, qr_pose, qr_to_center):
            self.data.position.x = qr_pose.position.x + qr_to_center.x
            self.data.position.y = qr_pose.position.y + qr_to_center.y
            self.data.position.z = qr_pose.position.z + qr_to_center.z
            self.data.orientation.x = qr_pose.orientation.x
            self.data.orientation.y = qr_pose.orientation.y
            self.data.orientation.z = qr_pose.orientation.z
            self.data.orientation.w = qr_pose.orientation.w

    table_width = 0.5
    table_length = 0.8
    table_height = 0.75

    # Key에는 "/socialrobot/perception/objects"에서 QR들의 object name이 사용되어야 합니다.
    # Value는 각 QR에서 table 중심까지의 XYZ translation 입니다.
    #     이때, value의 기준 좌표계는 메서드 mark_flag(obj)에 입력되는 obj의 좌표계를 사용합니다.
    #     예를 들어, "/socialrobot/perception/objects"로 들어오는 obj 값이 base_footprint 기준이면
    #     qr_to_center의 값도 base_footprint 기준입니다.
    qr_to_center = {
        "QR0": geo_msg.Point(-0.2, -0.25, -0.1 - (table_height/2.0)),
        "QR1": geo_msg.Point(-0.2, -0.15, -0.1 - (table_height/2.0)),
        "QR2": geo_msg.Point(-0.2, -0.05, -0.1 - (table_height/2.0)),
        "QR3": geo_msg.Point(-0.2,  0.05, -0.1 - (table_height/2.0)),
        "QR4": geo_msg.Point(-0.2,  0.15, -0.1 - (table_height/2.0)),
        "QR5": geo_msg.Point(-0.2,  0.25, -0.1 - (table_height/2.0)),
    }
    def __init__(self):
        self.bb3d = vision_msg.BoundingBox3D()
        self.qr_list = tuple(self.qr_to_center.keys())
        self.data = {
            "QR0": self.TableInstance(),
            "QR1": self.TableInstance(),
            "QR2": self.TableInstance(),
            "QR3": self.TableInstance(),
            "QR4": self.TableInstance(),
            "QR5": self.TableInstance(),
        }
        # Filtered table
        self.prev_table = vision_msg.BoundingBox3D()
        self.prev_table.center.orientation.x = 0.0
        self.prev_table.center.orientation.y = 0.0
        self.prev_table.center.orientation.z = 0.0
        self.prev_table.center.orientation.w = 1.0
        self.prev_table.size.x = self.table_width
        self.prev_table.size.y = self.table_length
        self.prev_table.size.z = self.table_height

    def mark_flag(self, obj):
        """obj: social_msg.Object[]
        std_msgs/Header header
        string id
        string type
        shape_msgs/Mesh mesh
        vision_msgs/BoundingBox2D bb2d
        vision_msgs/BoundingBox3D bb3d
        geometry_msgs/Pose[] grasp_point

        #affordance
        Affordance[] affordance
        """
        self.data[obj.id].has_new = True
        self.data[obj.id].set_data(obj.bb3d.center, self.qr_to_center[obj.id])

    def _update(self):
        count = 0
        point = geo_msg.Point(0.0, 0.0, 0.0)
        # higher pass
        idx = 0
        scores = [0.0] * 6
        closes = [geo_msg.Quaternion(0.0, 0.0, 0.0, 1.0)] * 6
        for name, table_inst in self.data.items():
            if table_inst.has_new:
                count += 1
                # position
                point.x += table_inst.data.position.x
                point.y += table_inst.data.position.y
                point.z += table_inst.data.position.z
                # orientation
                # close = close_quaternion(self.prev_table.center.orientation, table_inst.data.orientation)
                # closes[idx] = close
                # scores[idx] = dot(close, self.prev_table.center.orientation)
            idx += 1

        # threshold
        scores = np.array(scores)
        threshold = np.cos(np.radians(45))
        th_pass = scores[scores > threshold]
        mean = np.mean(th_pass)

        # qr_filter = scores >= mean
        # is_quat_update = True in qr_filter

        # naive_orientation = geo_msg.Quaternion(0.0, 0.0, 0.0, 0.0)
        # for idx in xrange(6):
        #     if qr_filter[idx]:
        #         dot_score = scores[idx]
        #         close = closes[idx]
        #         naive_orientation.x += close.x * dot_score
        #         naive_orientation.y += close.y * dot_score
        #         naive_orientation.z += close.z * dot_score
        #         naive_orientation.w += close.w * dot_score

        if count > 0:
            # Make avgerage
            c = float(count)
            point.x = point.x / c
            point.y = point.y / c
            point.z = point.z / c

            prev = self.prev_table.center
            # rospy.loginfo(self.prev_table.center.position)
            if prev.position.z > 0.4:
                # position
                gain = 0.1
                self.prev_table.center.position.x = gain * point.x + (1.0 - gain) * prev.position.x
                self.prev_table.center.position.y = gain * point.y + (1.0 - gain) * prev.position.y
                self.prev_table.center.position.z = gain * point.z + (1.0 - gain) * prev.position.z
                # orientation
                # if is_quat_update:
                #     self.prev_table.center.orientation = weighted_average_quat(
                #         prev.orientation, normalize_quaternion(naive_orientation), 0.05)
            else:
                self.prev_table.center.position = point
                # if is_quat_update:
                #     self.prev_table.center.orientation = normalize_quaternion(naive_orientation)

    def get(self):
        self._update()
        """return: social_msg.Object
        std_msgs/Header header
        string id
        string type
        shape_msgs/Mesh mesh
        vision_msgs/BoundingBox2D bb2d
        vision_msgs/BoundingBox3D bb3d
        geometry_msgs/Pose[] grasp_point

        #affordance
        Affordance[] affordance
        """
        result = social_msg.Object()
        result.id = "obj_table"
        result.bb3d = self.prev_table
        return result


class Example:
    def __init__(self):
        self.table_filter = TableFilter()
        rospy.Subscriber("/socialrobot/perception/objects", social_msg.Objects, self._callback_objects)

    def _callback_objects(self, data):
        # Update table QRs
        for obj in data.detected_objects:
            if obj.name in self.table_filter.qr_list:
                self.table_filter.mark_flag(obj)
        # Get filtered bb3d (socialrobot_msgs.msg.Object)
        table_bb3d = self.table_filter.get()
