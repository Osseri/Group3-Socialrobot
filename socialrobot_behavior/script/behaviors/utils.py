import tf
import tf.transformations as tfm
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion
import rospy
import numpy as np
import random
import math
import copy
from scipy.spatial import ConvexHull
from socialrobot_msgs.msg import Object
from socialrobot_behavior.msg import PlannerInputs
from geometry_msgs.msg import Pose, PoseArray, Pose2D, PoseStamped
from vision_msgs.msg import BoundingBox2D, BoundingBox3D

rot_x = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (1, 0, 0))
rot_y = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 1, 0))
rot_z = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 0, 1))

def degToRad(deg):
    rad = deg / 180.0 * math.pi
    return rad

def bbox_3d_to_2d(self, bbox3d):
    bbox2d = BoundingBox2D()
    bbox2d.center.x = bbox3d.center.position.x
    bbox2d.center.y = bbox3d.center.position.y
    rot_z = euler_from_quaternion([bbox3d.center.orientation.x, 
                                    bbox3d.center.orientation.y, 
                                    bbox3d.center.orientation.z, 
                                    bbox3d.center.orientation.w])[2]
    bbox2d.center.theta = rot_z
    bbox2d.size_x = bbox3d.size.x
    bbox2d.size_y = bbox3d.size.y
    return bbox2d

def pose_2_pose2d(pose_3d):
    '''
    convert ROS 3D pose into 2D pose
    '''
    quat = pose_3d.orientation
    pose2d = Pose2D(x=pose_3d.position.x, 
                    y=pose_3d.position.y, 
                    theta=euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2])
    return pose2d

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

def transform_object(obj_msg):
    '''
    transform object's affordance and grasp points into robot base frame
    obj_msg [social_robot_msgs/Object] : object frame
    return [social_robot_msgs/Object] : base_footprint frame
    '''
    tr_obj = copy.deepcopy(obj_msg)
    for i,gr_pt in enumerate(obj_msg.grasp_point):
        tr_obj.grasp_point[i] = transform_pose(gr_pt, obj_msg.bb3d.center)

    for i,aff in enumerate(obj_msg.affordance):
        tr_obj.affordance[i].header.frame_id = '/base_footprint'
        tr_obj.affordance[i].bb3d.center = transform_pose(aff.bb3d.center, tr_obj.bb3d.center)

        for j, gr_pt in enumerate(aff.grasp_point):
            tr_obj.affordance[i].grasp_point[j] = transform_pose(gr_pt, tr_obj.affordance[i].bb3d.center)

    return tr_obj

def transform_pose(pose, parent_pose):
    '''
    transform pose based on parent_pose 
    '''
    return multiply_pose(parent_pose, pose)

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

def get_grasp_pose(grasp_point, robot_group=None):
    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0

    if robot_group in [PlannerInputs.LEFT_ARM, PlannerInputs.LEFT_ARM_WITHOUT_WAIST, PlannerInputs.LEFT_GRIPPER]:
        pose.orientation.x = 0.707
        pose.orientation.y = 0.0
        pose.orientation.z = 0.707
        pose.orientation.w = 0.0
       
    elif robot_group in [PlannerInputs.RIGHT_ARM, PlannerInputs.RIGHT_ARM_WITHOUT_WAIST, PlannerInputs.RIGHT_GRIPPER]:
        pose.orientation.x = 0.0
        pose.orientation.y = 0.707
        pose.orientation.z = 0.0
        pose.orientation.w = 0.707

    return transform_pose(pose, grasp_point)

def connect_trajectories(first_traj, second_traj):
    '''
    trajectory_msgs/JointTrajectory
    '''
    merged_traj = copy.copy(first_traj)
    last_time = first_traj.points[-1].time_from_start + rospy.Time(0.2)
    # merge next trajectory
    for pt in second_traj.points:
        pt.time_from_start = rospy.Time().from_sec(pt.time_from_start.to_sec() + last_time.to_sec())
        merged_traj.points.append(pt)

    return merged_traj

def connect_motion_plan(first_plan, second_plan, duration=None):
    merged_plan = copy.deepcopy(first_plan)

    # get intarval
    if not duration:
        duration = merged_plan.jointTrajectory.points[-2:][1].time_from_start - merged_plan.jointTrajectory.points[-2:][0].time_from_start
    time_from_start = merged_plan.jointTrajectory.points[-1].time_from_start + duration

    for pt in second_plan.jointTrajectory.points:
        pt.time_from_start += time_from_start
        merged_plan.jointTrajectory.points.append(pt)

    return merged_plan

def joint_trajectory_deg2rad(joint_trajectory):
    for p in joint_trajectory.points:
        p.positions = [math.radians(t) for t in p.positions]
    return joint_trajectory

def get_vertex_from_bb3d(bb3d):
    vertex = []

    pos = bb3d.center.position
    quat = bb3d.center.orientation

    pts = [None]*8
    pts[0] = [-bb3d.size.x/2.0, -bb3d.size.y/2.0, -bb3d.size.z/2.0]
    pts[1] = [-bb3d.size.x/2.0, bb3d.size.y/2.0, -bb3d.size.z/2.0]
    pts[2] = [bb3d.size.x/2.0, -bb3d.size.y/2.0, -bb3d.size.z/2.0]
    pts[3] = [bb3d.size.x/2.0, bb3d.size.y/2.0, -bb3d.size.z/2.0]
    pts[4] = [-bb3d.size.x/2.0, -bb3d.size.y/2.0, bb3d.size.z/2.0]
    pts[5] = [-bb3d.size.x/2.0, bb3d.size.y/2.0, bb3d.size.z/2.0]
    pts[6] = [bb3d.size.x/2.0, -bb3d.size.y/2.0, bb3d.size.z/2.0]
    pts[7] = [bb3d.size.x/2.0, bb3d.size.y/2.0, bb3d.size.z/2.0]

    for pt in pts:
        tran_mat = tf.transformations.translation_matrix(pt)
        rot_vec = [quat.x, quat.y, quat.z, quat.w]
        rot_mat = tf.transformations.quaternion_matrix(rot_vec)
        T = np.dot(rot_mat, tran_mat) 

        trans_pt = tf.transformations.translation_from_matrix(T) 
        trans_pt = [trans_pt[0]+pos.x, trans_pt[1]+pos.y, trans_pt[2]+pos.z]
        vertex.append(trans_pt)
    return vertex

def get_vertex_from_bb2d(bb2d, minmax=False):
    vertex = []

    pos = bb2d.center

    pts = [None]*4
    pts[0] = [-bb2d.size_x/2.0, -bb2d.size_y/2.0, 0.0]
    pts[1] = [-bb2d.size_x/2.0, bb2d.size_y/2.0, 0.0]
    pts[2] = [bb2d.size_x/2.0, -bb2d.size_y/2.0, 0.0]
    pts[3] = [bb2d.size_x/2.0, bb2d.size_y/2.0, 0.0]

    for pt in pts:
        tran_mat = tf.transformations.translation_matrix(pt)
        rot_vec = quaternion_from_euler(0, 0, pos.theta)
        rot_mat = tf.transformations.quaternion_matrix(rot_vec)
        T = np.dot(rot_mat, tran_mat) 

        trans_pt = tf.transformations.translation_from_matrix(T) 
        trans_pt = [trans_pt[0]+pos.x, trans_pt[1]+pos.y, 0]
        vertex.append(trans_pt)
    
    if minmax:
        return [vertex[0], vertex[3]]
    return vertex

def get_bb2d_from_hull(rectangle_center, length_x, length_y, unit_vector_angle):
    '''
    hull: rectangle_center, length_x, length_y, unit_vector_angle
    '''
    bb2d = BoundingBox2D()
    bb2d.center.x = rectangle_center[0]
    bb2d.center.y = rectangle_center[1]
    bb2d.center.theta = unit_vector_angle
    bb2d.size_x = length_x
    bb2d.size_y = length_y
    return bb2d

########## Calculate convexhull from points
# https://bitbucket.org/william_rusnack/minimumboundingbox/src/master/MinimumBoundingBox.py

def unit_vector(pt0, pt1):
    # returns an unit vector that points in the direction of pt0 to pt1
    dis_0_to_1 = math.sqrt((pt0[0] - pt1[0])**2 + (pt0[1] - pt1[1])**2)
    return (pt1[0] - pt0[0]) / dis_0_to_1, \
           (pt1[1] - pt0[1]) / dis_0_to_1


def orthogonal_vector(vector):
    # from vector returns a orthogonal/perpendicular vector of equal length
    return -1 * vector[1], vector[0]


def bounding_area(index, hull):
    unit_vector_p = unit_vector(hull[index], hull[index+1])
    unit_vector_o = orthogonal_vector(unit_vector_p)

    dis_p = tuple(np.dot(unit_vector_p, pt) for pt in hull)
    dis_o = tuple(np.dot(unit_vector_o, pt) for pt in hull)

    min_p = min(dis_p)
    min_o = min(dis_o)
    len_p = max(dis_p) - min_p
    len_o = max(dis_o) - min_o

    return {'area': len_p * len_o,
            'length_parallel': len_p,
            'length_orthogonal': len_o,
            'rectangle_center': (min_p + len_p / 2, min_o + len_o / 2),
            'unit_vector': unit_vector_p,
            }


def to_xy_coordinates(unit_vector_angle, point):
    # returns converted unit vector coordinates in x, y coordinates
    angle_orthogonal = unit_vector_angle + math.pi / 2
    return point[0] * math.cos(unit_vector_angle) + point[1] * math.cos(angle_orthogonal), \
           point[0] * math.sin(unit_vector_angle) + point[1] * math.sin(angle_orthogonal)


def rotate_points(center_of_rotation, angle, points):
    # Requires: center_of_rotation to be a 2d vector. ex: (1.56, -23.4)
    #           angle to be in radians
    #           points to be a list or tuple of points. ex: ((1.56, -23.4), (1.56, -23.4))
    # Effects: rotates a point cloud around the center_of_rotation point by angle
    rot_points = []
    ang = []
    for pt in points:
        diff = tuple([pt[d] - center_of_rotation[d] for d in range(2)])
        diff_angle = math.atan2(diff[1], diff[0]) + angle
        ang.append(diff_angle)
        diff_length = math.sqrt(sum([d**2 for d in diff]))
        rot_points.append((center_of_rotation[0] + diff_length * math.cos(diff_angle),
                           center_of_rotation[1] + diff_length * math.sin(diff_angle)))

    return rot_points

def rectangle_corners(rectangle):
    # Requires: the output of mon_bounding_rectangle
    # Effects: returns the corner locations of the bounding rectangle
    corner_points = []
    for i1 in (.5, -.5):
        for i2 in (i1, -1 * i1):
            corner_points.append((rectangle['rectangle_center'][0] + i1 * rectangle['length_parallel'],
                            rectangle['rectangle_center'][1] + i2 * rectangle['length_orthogonal']))

    return rotate_points(rectangle['rectangle_center'], rectangle['unit_vector_angle'], corner_points)

def get_hull_from_points(points):
    # Requires: points to be a list or tuple of 2D points. ex: ((5, 2), (3, 4), (6, 8))
    #           needs to be more than 2 points

    if len(points) <= 2: raise ValueError('More than two points required.')

    hull_ordered = [points[index] for index in ConvexHull(points).vertices]
    hull_ordered.append(hull_ordered[0])
    hull_ordered = tuple(hull_ordered)

    min_rectangle = bounding_area(0, hull_ordered)
    for i in range(1, len(hull_ordered)-1):
        rectangle = bounding_area(i, hull_ordered)
        if rectangle['area'] < min_rectangle['area']:
            min_rectangle = rectangle

    min_rectangle['unit_vector_angle'] = math.atan2(min_rectangle['unit_vector'][1], min_rectangle['unit_vector'][0])
    min_rectangle['rectangle_center'] = to_xy_coordinates(min_rectangle['unit_vector_angle'], min_rectangle['rectangle_center'])

    # this is ugly but a quick hack and is being changed in the speedup branch
    # area = min_rectangle['area'],
    length_parallel = min_rectangle['length_parallel']
    length_orthogonal = min_rectangle['length_orthogonal']
    rectangle_center = min_rectangle['rectangle_center']
    # unit_vector = min_rectangle['unit_vector'],
    unit_vector_angle = min_rectangle['unit_vector_angle'],
    corner_points = rectangle_corners(min_rectangle)

    return [rectangle_center, length_parallel, length_orthogonal, unit_vector_angle[0]]