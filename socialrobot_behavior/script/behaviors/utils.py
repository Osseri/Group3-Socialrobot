import tf
import tf.transformations as tfm
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion
import rospy
import rospkg
import numpy as np
import random
import math
import copy
import yaml
import itertools
from scipy.spatial import ConvexHull
from socialrobot_msgs.msg import Object, Affordance
from socialrobot_behavior.msg import PlannerInputs
from geometry_msgs.msg import Pose, PoseArray, Pose2D, PoseStamped, Point32
from vision_msgs.msg import BoundingBox2D, BoundingBox3D

rot_x = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (1, 0, 0))
rot_y = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 1, 0))
rot_z = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 0, 1))

# load object database
r = rospkg.RosPack()
pkg_path = r.get_path('perception_interface')
yaml_path = pkg_path + '/config/objects.yaml'
yaml_data = None
with open(yaml_path, 'r') as stream:
    try:
        yaml_data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print('[Behavior utils] cannot load object database.')

def update_object_info(object):
    object_msg = get_object_info(object.id)
    if object_msg != None:
        object.grasp_point = object_msg.grasp_point
        object.approach_direction = object_msg.approach_direction
        object.affordance = object_msg.affordance

def get_object_info(object_id=None):
    '''
    convert object yaml to socialrobot_msgs/Object and return
    '''
    object_info = None
    objects = []

    if object_id != None:   # return only one object
        for i, obj in enumerate(yaml_data['objects']):
            if object_id == obj['name']:
                object_info = obj
                break

        if object_info:
            obj_msg = yaml_to_msg(object_info)
            return obj_msg
        else: 
            return None
    else:   # return all objects in database
        for i, obj in enumerate(yaml_data['objects']):
            obj_msg = yaml_to_msg(obj)
            objects.append(obj_msg)
        return objects

def yaml_to_msg(object_info):
        '''
        create Object.msg
        '''
        name =  object_info.get('name', '')
        size = object_info.get('size', {'x':0, 'y':0, 'z':0})
        shape = object_info.get('shape', '')
        trans = object_info.get('translation', {'x':0, 'y':0, 'z':0})
        rot = object_info.get('rotation', {'x':0, 'y':0, 'z':0, 'w':0})
        trans_vec = [trans['x'], trans['y'], trans['z']]
        rot_vec = [rot['x'], rot['y'], rot['z'], rot['w']]
        size_vec = [size['x'], size['y'], size['z']]
        aff = None

        obj = Object()
        obj.id = name
        obj.shape = shape
        obj.type = object_info.get('type', None)
        obj.bb3d = create_boundingbox3d(trans_vec, rot_vec, size_vec)

        # arm grasp direction
        gr_pt = get_object_pose_from_yaml(obj, object_info, 'grasp_point')
        # mobile approach direction
        app_dir = get_object_pose_from_yaml(obj, object_info, 'approach_direction')

        if gr_pt != None:
            obj.grasp_point = gr_pt
        if app_dir != None:
            obj.approach_direction = app_dir
        
        # object affordance
        get_object_affordance_from_yaml(obj, object_info)

        # TODO: hard-coding for demo
        if obj.id == 'obj_fridge':
            for i,aff in enumerate(obj.affordance):
                if aff.id == 'obj_fridge_bottom_door':
                    if rospy.has_param('fridge_isopen'):
                        if rospy.get_param('fridge_isopen'):
                            obj.affordance[i].bb3d.center.position.x = +4.5400e-01
                            obj.affordance[i].bb3d.center.position.y = +3.9664e-01
                            obj.affordance[i].bb3d.center.position.z = -2.1505e-01
                            obj.affordance[i].bb3d.center.orientation.x = 0.0
                            obj.affordance[i].bb3d.center.orientation.y = 0.0
                            obj.affordance[i].bb3d.center.orientation.z = 0.866
                            obj.affordance[i].bb3d.center.orientation.w = 0.5

        return obj

def get_object_affordance_from_yaml(object_msg, object_info):

    if object_info.has_key('affordance'):
        for i, affordance in enumerate(object_info['affordance']):
            aff = Affordance()
            aff.header.frame_id = object_msg.id                
            aff.id =  affordance.get('name', '')
            aff.shape = affordance.get('shape', '')
            aff_size = affordance.get('size', {'x':0, 'y':0, 'z':0})
            aff_trans = affordance.get('translation', {'x':0, 'y':0, 'z':0})
            aff_rot = affordance.get('rotation', {'x':0, 'y':0, 'z':0, 'w':0})
            aff_trans_vec = [aff_trans['x'], aff_trans['y'], aff_trans['z']]
            aff_rot_vec = [aff_rot['x'], aff_rot['y'], aff_rot['z'], aff_rot['w']]
            aff_size_vec = [aff_size['x'], aff_size['y'], aff_size['z']]      
            aff.bb3d = create_boundingbox3d(aff_trans_vec, aff_rot_vec, aff_size_vec)
            
            gr_pt = get_object_pose_from_yaml(aff, affordance, 'grasp_point')
            app_dir = get_object_pose_from_yaml(aff, affordance, 'approach_direction')
            if gr_pt != None:
                aff.grasp_point = gr_pt
            if app_dir != None:
                aff.approach_direction = app_dir

            # object affordance
            get_object_affordance_from_yaml(aff, affordance)
            object_msg.affordance.append(aff)
  

def get_object_pose_from_yaml(object_msg, object_info, key=None):
    '''
    get key from yaml file
    '''
    
    if key != None:
        if object_info.has_key(key):
            value = object_info[key]

            if type(value) is list: # return array
                arr = []
                for element in value:
                    trans = element['translation']
                    rot = element['rotation']
                    arr.append(create_pose([trans['x'], trans['y'], trans['z']],
                                    [rot['x'], rot['y'], rot['z'], rot['w']]))
                return arr
            elif type(value) is dict:
                trans = value['translation']
                rot = value['rotation']
                return create_pose([trans['x'], trans['y'], trans['z']],
                                    [rot['x'], rot['y'], rot['z'], rot['w']])
        else:
            return None
    else:
        return None

def transform_object(obj_msg):
    '''
    transform object's affordance and grasp points into robot base frame
    obj_msg [social_robot_msgs/Object] : object frame
    return [social_robot_msgs/Object] : base_footprint frame
    '''
    tr_obj = copy.deepcopy(obj_msg)

    # robot based object grasp point
    for i,gr_pt in enumerate(obj_msg.grasp_point):
        tr_obj.grasp_point[i] = transform_pose(gr_pt, obj_msg.bb3d.center)

    # robot based object affordance
    for i,aff in enumerate(obj_msg.affordance):
        tr_obj.affordance[i].header.frame_id = '/base_footprint'
        tr_obj.affordance[i].bb3d.center = transform_pose(aff.bb3d.center, tr_obj.bb3d.center)

        for j, gr_pt in enumerate(aff.grasp_point):
            tr_obj.affordance[i].grasp_point[j] = transform_pose(gr_pt, tr_obj.affordance[i].bb3d.center)

        # robot based affordance's affordance
        for k,subaff in enumerate(aff.affordance):
            tr_obj.affordance[i].affordance[k].header.frame_id = '/base_footprint'
            tr_obj.affordance[i].affordance[k].bb3d.center = transform_pose(subaff.bb3d.center, transform_pose(aff.bb3d.center, tr_obj.bb3d.center))

            for l, aff_gr_pt in enumerate(subaff.grasp_point):
                tr_obj.affordance[i].affordance[k].grasp_point[l] = transform_pose(aff_gr_pt, tr_obj.affordance[i].affordance[k].bb3d.center)

    return tr_obj

def create_boundingbox3d(trans_vec, rot_vec, size):
    '''create bounding box ROS msg'''
    
    bb3d = BoundingBox3D()
    bb3d.center.position.x = trans_vec[0]
    bb3d.center.position.y = trans_vec[1] 
    bb3d.center.position.z = trans_vec[2] 
    bb3d.center.orientation.x = rot_vec[0]
    bb3d.center.orientation.y = rot_vec[1]
    bb3d.center.orientation.z = rot_vec[2]
    bb3d.center.orientation.w = rot_vec[3]
    bb3d.size.x = size[0]
    bb3d.size.y = size[1]
    bb3d.size.z = size[2]
    return bb3d

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

def pose2d_2_pose(pose2d):
    quat = tf.transformations.quaternion_from_euler(0, 0, pose2d.theta)
    pose = Pose()
    pose.position.x = pose2d.x
    pose.position.y = pose2d.y
    pose.position.z = 0.0
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

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

def inverse_pose(pose):
    mat = pose_2_mat(pose)    
    return mat_2_pose(np.linalg.inv(mat))

def multiply_pose(pose1, pose2):
    mat1 = pose_2_mat(pose1)
    mat2 = pose_2_mat(pose2)
    mat3 = np.dot(mat1, mat2)
    return mat_2_pose(mat3)

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

def connect_trajectories(first_traj, second_traj, duration=None, only_last=False):
    '''
    trajectory_msgs/JointTrajectory
    '''
    merged_traj = copy.deepcopy(first_traj)

    # BUG: if duration >0, motion reverse has occured
    # get intarval
    if not duration:
        duration = merged_traj.points[-2:][1].time_from_start - merged_traj.points[-2:][0].time_from_start
    time_from_start = merged_traj.points[-1].time_from_start + duration

    # merge next trajectory
    if only_last:
        pt = second_traj.points[-1]
        pt.time_from_start += time_from_start
        merged_traj.points.append(pt)
    else:
        for pt in second_traj.points:
            pt.time_from_start += time_from_start
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

def reverse_motion_plan(plan, duration=None):
    reversed_plan = copy.deepcopy(plan)

    # reverse motion trajectory
    reversed_plan.jointTrajectory.points.reverse()

    # reverse time_from_start again
    time_intervals = []
    for pt in reversed_plan.jointTrajectory.points:
        time_intervals.append(pt.time_from_start)
    time_intervals.reverse()
    for i,interval in enumerate(time_intervals):
        reversed_plan.jointTrajectory.points[i].time_from_start = interval

        # reverse velocity and accel
        velocities = reversed_plan.jointTrajectory.points[i].velocities
        accelerations = reversed_plan.jointTrajectory.points[i].accelerations
        reversed_plan.jointTrajectory.points[i].velocities = [vel*-1 for vel in velocities]
        reversed_plan.jointTrajectory.points[i].accelerations = [acc*-1 for acc in accelerations]

    return reversed_plan


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

def create_workspace(target_bb3d):
    '''
    create the bottom surface which an target object is placed on 
    '''
    table_height = target_bb3d.center.position.z - target_bb3d.size.z/2.0

    table_bb3d = BoundingBox3D()
    table_bb3d.center.position.x = 0.5
    table_bb3d.center.position.y = 0.0
    table_bb3d.center.position.z = table_height/2.0
    table_bb3d.center.orientation.x = 0.0
    table_bb3d.center.orientation.y = 0.0
    table_bb3d.center.orientation.z = 1.0
    table_bb3d.center.orientation.w = 0.0
    table_bb3d.size.x = 0.7
    table_bb3d.size.y = 1.5
    table_bb3d.size.z = table_height

    table = Object()
    table.header.frame_id = 'base_footprint'
    table.id = 'obj_table'
    table.type = 'static'
    table.bb3d = table_bb3d
    
    return table

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

def interpolate_point32(A, B, interval=0.1):
    dist_x = B[0] - A[0]
    dist_y = B[1] - A[1]
    num = int(np.sqrt(dist_x**2 + dist_y**2) / interval) + 1
    dx = dist_x / num
    dy = dist_y / num
    return [Point32(A[0] + dx * i, A[1] + dy * i, 0.0) for i in range(num)]

def bb3d_to_polygon(bb3d):
    """
    A-----------B
    |     ^ X   |
    | Y <-+     |(length)
    |           |
    D--(width)--C
    """
    # Center
    x = bb3d.center.position.x
    y = bb3d.center.position.y
    length = bb3d.size.x
    width = bb3d.size.y
    quat = bb3d.center.orientation
    # X-axis heading from Quaternion
    # This equation is the X-axis direction (xx, xy, xz) from the rotation matrix R
    half_xx = 0.5 - (quat.y * quat.y + quat.z * quat.z)
    half_xy = quat.x * quat.y + quat.z * quat.w
    heading = np.arctan2(half_xy, half_xx)
    # Unit vectors
    x_unit = (np.cos(heading), np.sin(heading))
    y_unit = (-x_unit[1], x_unit[0])
    x_vec = (length * x_unit[0], length * x_unit[1])
    y_vec = (width * y_unit[0], width * y_unit[1])
    # Polygon vertices
    A = (x + 0.5 * x_vec[0] + 0.5 * y_vec[0], y + 0.5 * x_vec[1] + 0.5 * y_vec[1])
    B = (A[0] - y_vec[0], A[1] - y_vec[1])
    C = (B[0] - x_vec[0], B[1] - x_vec[1])
    D = (A[0] - x_vec[0], A[1] - x_vec[1])
    # Line segments
    line_segments = [(A, B), (B, C), (C, D), (D, A)]
    # Polygon.points (geometry_msgs/Point32)
    return list(itertools.chain.from_iterable(
        [interpolate_point32(a, b) for a, b in line_segments]))

if __name__ =='__main__':
    # Initialize ROS node
    rospy.init_node('behavior_utils', anonymous=True)
    print(get_object_info('obj_fridge'))