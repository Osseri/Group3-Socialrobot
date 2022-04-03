#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import tf.transformations as tfm
import math
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion

rot_x = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (1, 0, 0))
rot_y = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 1, 0))
rot_z = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 0, 1))

def pose_2_mat(pose):
    trans_vec = [pose.position.x, pose.position.y, pose.position.z]
    rot_vec = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    trans_mat = tf.transformations.translation_matrix(trans_vec)
    rot_mat = tf.transformations.quaternion_matrix(rot_vec)
    T = np.dot(trans_mat, rot_mat) 
    return T

def mat_2_pose(T):
    trans = tf.transformations.translation_from_matrix(T) 
    rot = tf.transformations.quaternion_from_matrix(T)

    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    return pose

def multiply_pose(pose1, pose2):
    mat1 = pose_2_mat(pose1)
    mat2 = pose_2_mat(pose2)
    mat3 = np.dot(mat1, mat2)
    return mat_2_pose(mat3)

def create_pose(trans, rot):
    '''
    create ROS pose msg from translation and quaternion vector
    '''
    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    return pose

##############################
# Main function
##############################
if __name__ == '__main__':
    print("============ Starting tutorial setup")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("right_arm")
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

    print ("============ planning_frame: \n%s" % group.get_planning_frame())
    print ("============ end_effector_link: \n%s" % group.get_end_effector_link())
    print ("============ Robot Groups:")
    print (robot.get_group_names())

    waypoints = []

    # start with the current pose
    current_pose = group.get_current_pose().pose

    waypoints.append(current_pose)

    transform = create_pose([0, 0, 0], rot_z(-30))

    # first orient gripper and move (+z)
    wpose = copy.deepcopy(current_pose)
    wpose.position.x -= 0.02
    wpose = multiply_pose(wpose, transform)
    waypoints.append(copy.deepcopy(wpose))

    # second move 
    wpose.position.x -= 0.02
    wpose = multiply_pose(wpose, transform)
    waypoints.append(copy.deepcopy(wpose))

    # third move to the side
    wpose.position.x -= 0.02
    wpose = multiply_pose(wpose, transform)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.00)         # jump_threshold

    print("============ Waiting while RVIZ displays plan...")
    rospy.sleep(5)