#!/usr/bin/env python
import numpy as np
import rospy
import tf
from tf.transformations import euler_from_quaternion
from vision_msgs.msg import BoundingBox2D
from ir_repositioning.srv import Repositioning, RepositioningRequest
from geometry_msgs.msg import PoseArray, PoseStamped

"""
[SERVICE RUN]
roslaunch ir_repositioning ir_server.launch

[REQUEST EXAMPLE]
rosrun ir_repositioning request.py
"""

def load_objects():
    objects = []
    table = BoundingBox2D()
    table.center.x = 0.889643788338
    table.center.y = -0.221748813987
    rot_z = euler_from_quaternion([0, 0, 0.537307202816, 0.843386650085])[2]
    table.center.theta = rot_z
    table.size_x = 1.13421618938
    table.size_y = 0.708873927593
    objects.append({'name':'table', 'bbox':table})

    box = BoundingBox2D()
    box.center.x = 0.689339280128
    box.center.y = -0.128358691931
    rot_z = euler_from_quaternion([0, 0, -0.216439634562, 0.976296007633])[2]
    box.center.theta = rot_z
    box.size_x = 0.0500000007451
    box.size_y = 0.125
    objects.append({'name':'box', 'bbox':box})

    return objects

def request():
    objects = load_objects()

    req = RepositioningRequest()
    req.hand_type = req.DUAL_HAND #LEFT_HAND, RIGHT_HAND, DUAL_HAND
    req.dual_hand_width = 0.2

    # set objects
    target_name = 'box'  

    for i, obj in enumerate(objects):
        if obj['name'] == target_name:
            target = obj['bbox']
        else:
            req.Obs.append(obj['bbox'])

    req.Pt.x = target.center.x
    req.Pt.y = target.center.y

    req.Cr.x = np.radians(0.0)
    req.Cr.y = np.radians(30.1)

    req.Ct.x = np.radians(-15)
    req.Ct.y = np.radians(15)

    req.section_definition.x = 0.05
    req.section_definition.y = 1.0
    req.section_definition.z = 0.02

    req.max_dist = 1024.0
    req.collision_offset = 0.0
    req.strict_dual = True

    rospy.wait_for_service("/ir_server/find_positions")
    try:
        feasi_proxy = rospy.ServiceProxy("/ir_server/find_positions", Repositioning)
        print(req)
        resp = feasi_proxy(req)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def res_to_msg(data):
    pose_array = PoseArray()
    pose_array.header.frame_id = 'base_footprint'
    for i, pt in enumerate(data.candidates):
        if i<1:
            mobile_pose = pt

            pose = PoseStamped()
            pose.pose.position.x = mobile_pose.x
            pose.pose.position.y = mobile_pose.y
            pose.pose.position.z = 0.0
            quaternion = tf.transformations.quaternion_from_euler(0, 0, mobile_pose.theta)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            pose_array.poses.append(pose.pose)
    return pose_array

if __name__ == "__main__":
    rospy.init_node('ir_test')
    pose_pub = rospy.Publisher('/mobile_pose', PoseArray, queue_size=10)

    resp = request()     
    print('number of candidates: ', resp.num_candidates)
    if(resp.num_candidates>0):
        pose_array = res_to_msg(resp)
        iter = 0
        rate = rospy.Rate(10)
        while(not rospy.is_shutdown()):
            pose_pub.publish(pose_array)
            iter += 1   
            rate.sleep()
        print(resp.candidates[0])