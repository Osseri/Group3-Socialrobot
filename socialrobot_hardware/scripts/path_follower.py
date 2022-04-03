#!/usr/bin/env python
import rospy

import math
import numpy as np
import tf
import actionlib

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import JointState
from socialrobot_hardware.msg import *
from socialrobot_hardware.srv import *
from yaml.nodes import SequenceNode

def normalize_angle(rad):
        if rad < -math.pi:
            rad = 2.0*math.pi + rad
        if rad > math.pi:
            rad = -2.0*math.pi + rad
        return rad

class MobileController():
    def __init__(self):

        # ROS action server
        self.nav_action_server = actionlib.SimpleActionServer('mobile_controller/follow_path_trajectory', FollowPathTrajectoryAction, self.callback_nav_action)
        self.action_feedback = FollowPathTrajectoryActionFeedback()
        self.action_result = FollowPathTrajectoryActionResult()

        # ROS services
        self.set_path = rospy.Service('/set_path', SetPathTrajectory, self.callback_set_path)

        # ROS topics
        self.mobile_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/mobile_path', Path, queue_size=10)
        self.pose_pub = rospy.Publisher('/mobile_pose', PoseArray, queue_size=10)
        self.odom_pub = rospy.Publisher('/current_pose', PoseArray, queue_size=10)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.cur_odom = None
        self.cur_vel = None
        self.max_lin_vel = rospy.get_param("~max_lin_vel", default="0.05")
        self.max_ang_vel = rospy.get_param("~max_ang_vel", default="0.08")

        self.dist_thre = 0.01   # meter
        self.ori_thre = np.deg2rad(2.0)   # rad
        self.joint_states = []
        rospy.spin()

    def callback_nav_action(self, goal):
        path_trajectory = goal.trajectory

        if self.run(path_trajectory, action_call=True):
            self.action_result.result = FollowPathTrajectoryResult.OK
        else:
            self.action_result.result = FollowPathTrajectoryResult.ERROR
            
        return self.nav_action_server.set_succeeded(self.action_result)

    def callback_set_path(self, req):
        res = SetPathTrajectoryResponse()
        path_trajectory = req.trajectory

        if self.run(path_trajectory):
            res.result = SetPathTrajectoryResponse.OK
        else:
            res.result = SetPathTrajectoryResponse.ERROR
        return res

    def callback_odom(self, msg):
        self.cur_vel = msg.twist.twist
        self.cur_odom = msg.pose.pose

        # pose_array = PoseArray()
        # pose_array.header.frame_id = 'map'
        # pose_array.header.stamp = rospy.Time.now()
        # pose = PoseStamped()
        # pose.header.stamp = rospy.Time.now()
        # pose.header.frame_id = 'map'
        # pose.pose.position.x = msg.pose.pose.position.x
        # pose.pose.position.y = msg.pose.pose.position.y
        # pose.pose.position.z = 0.0
        # pose.pose.orientation.x = msg.pose.pose.orientation.x
        # pose.pose.orientation.y = msg.pose.pose.orientation.y
        # pose.pose.orientation.z = msg.pose.pose.orientation.z
        # pose.pose.orientation.w = msg.pose.pose.orientation.w
        # pose_array.poses.append(pose.pose)
        # self.odom_pub.publish(pose_array)

    def odom_to_posearray(self, odom):
        pose_array = PoseArray()
        pose_array.header.frame_id = 'odom'
        pose_array.header.stamp = rospy.Time.now()
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = odom.position.x
        pose.pose.position.y = odom.position.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = odom.orientation.x
        pose.pose.orientation.y = odom.orientation.y
        pose.pose.orientation.z = odom.orientation.z
        pose.pose.orientation.w = odom.orientation.w
        pose_array.poses.append(pose.pose)
        return pose_array

    def transform_path(self, transform, path_trajectory):
        # transform path trajectory based on odom frame
        transformed_path = Path()
        transformed_path.header.frame_id = 'odom'
        transformed_path.header.stamp = rospy.Time.now()

        for path in path_trajectory.poses:
            mat_path = np.dot(tf.transformations.translation_matrix([path.pose.position.x,
                                                                    path.pose.position.y,
                                                                    path.pose.position.z]),
                            tf.transformations.quaternion_matrix([path.pose.orientation.x,
                                                                    path.pose.orientation.y,
                                                                    path.pose.orientation.z,
                                                                    path.pose.orientation.w]))
            mat_transformed_path = np.dot(transform, mat_path)
            trans = tf.transformations.translation_from_matrix(mat_transformed_path)
            rot = tf.transformations.quaternion_from_matrix(mat_transformed_path)

            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]

            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            transformed_path.poses.append(pose)

        return transformed_path

    def run(self, path_trajectory, action_call=False):
        # wait for odom publisher
        while(not self.cur_odom ):
            pass

        # transformation between start pose and current odom
        mat_odom_to_start = np.dot(tf.transformations.translation_matrix([self.cur_odom.position.x,
                                                                        self.cur_odom.position.y,
                                                                        self.cur_odom.position.z]),
                                tf.transformations.quaternion_matrix([self.cur_odom.orientation.x,
                                                                        self.cur_odom.orientation.y,
                                                                        self.cur_odom.orientation.z,
                                                                        self.cur_odom.orientation.w]))
        mat_start_to_odom = np.linalg.inv(mat_odom_to_start)

        # transform path based on odom
        transformed_path = self.transform_path(mat_odom_to_start, path_trajectory)

        rate = rospy.Rate(100)
        i=0
        goal_pose = transformed_path.poses[i].pose
        seq=0
        while(not rospy.is_shutdown()):
            twist = Twist()

            # check final goal
            if self.goal_reached(self.cur_odom, transformed_path.poses[-1].pose):
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.mobile_pub.publish(twist)
                print('goal reached')
                return True

            # check waypoint reached
            if self.goal_reached(self.cur_odom, goal_pose):
                #update next waypoint
                i+=1
                goal_pose = transformed_path.poses[i].pose

            # calculate motor velocity
            self.extract_velocity(self.cur_odom, goal_pose, twist)

            # publish velocity command
            self.mobile_pub.publish(twist)

            # publish current pose for visualization
            self.path_pub.publish(transformed_path)
            current_pose = self.odom_to_posearray(self.cur_odom)
            self.pose_pub.publish(current_pose)

            #publish action feedback
            # if action_call:
            #     self.action_feedback.feedback.current_pose = current_pose
            #     self.nav_action_server.publish_feedback(self.action_feedback.feedback)
            rate.sleep()

    def goal_reached(self, cur_pose, goal_pose):
        # calculate position difference
        dx = cur_pose.position.x - goal_pose.position.x
        dy = cur_pose.position.y - goal_pose.position.y
        dist = math.sqrt(dx**2 + dy**2)

        # calculate orientation difference
        curr_yaw = tf.transformations.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, self.cur_odom.orientation.w])[2]
        goal_yaw = tf.transformations.euler_from_quaternion([goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w])[2]
        orientdiff = normalize_angle(goal_yaw - curr_yaw)

        if dist < self.dist_thre and abs(orientdiff) < self.ori_thre:
            return True
        else:
            return False

    def extract_velocity(self, cur_pose, goal_pose, twist):
        dt = 1.0/30 # 30hz
        dx = goal_pose.position.x - cur_pose.position.x
        dy = goal_pose.position.y - cur_pose.position.y
        dist = math.sqrt(dx**2 + dy**2)

        pose1_yaw = tf.transformations.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])[2]
        pose2_yaw = tf.transformations.euler_from_quaternion([goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w])[2]
        orientdiff = normalize_angle(pose2_yaw - pose1_yaw)

        # print('=======================')
        # print('--goal position')
        # print(goal_pose.position.x, goal_pose.position.y)
        # print('--current position')
        # print(cur_pose.position.x, cur_pose.position.y)
        # print('--diff')
        # print(dx, dy, dist, orientdiff)

        cos_theta1 = math.cos(pose1_yaw)
        sin_theta1 = math.sin(pose1_yaw)
        p1_dx = cos_theta1*dx + sin_theta1*dy
        p1_dy = -sin_theta1*dx + cos_theta1*dy
        vx = p1_dx / dt
        vy = p1_dy / dt
        omega = orientdiff / dt

        # print('--velocity')
        # print(vx, vy, omega)

        ratio_x = 1.0
        ratio_y = 1.0
        ratio_omega = 1.0
        if(vx > self.max_lin_vel or vx < -self.max_lin_vel):
            ratio_x = abs(self.max_lin_vel / vx)
        if(vy > self.max_lin_vel or vy < -self.max_lin_vel):
            ratio_y = abs(self.max_lin_vel / vy)
        if(omega > self.max_ang_vel or omega < -self.max_ang_vel):
            ratio_omega = abs(self.max_ang_vel / omega)

        ratio = min(ratio_x, ratio_y, ratio_omega)

        twist.linear.x = vx * ratio
        twist.linear.y = vy * ratio
        twist.angular.z = omega * ratio

        # print('--twist')
        # print(twist.linear.x)
        # print(twist.linear.y)
        # print(twist.angular.z)

if __name__ == "__main__":
    rospy.init_node("path_follower")
    mc = MobileController()