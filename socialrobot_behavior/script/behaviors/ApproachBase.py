import abc
from abc import ABCMeta
from numpy.core.numeric import require
from six import with_metaclass
import math
import copy
import rospy
import rosservice
import rosparam

from socialrobot_motion.srv import *
from socialrobot_behavior.msg import *
from socialrobot_behavior.srv import *
from socialrobot_hardware.srv import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import Pose, PoseArray, Pose2D, PoseStamped, Quaternion
from vision_msgs.msg import BoundingBox2D
from nav_msgs.msg import Path

import tf
import tf.transformations as tfm
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion
import numpy as np

from ir_repositioning.srv import Repositioning, RepositioningRequest
#non-holonomic
# from mobile_motion_planner.msg import Obstacle2D, MobileTrajectory
# from mobile_motion_planner.srv import *
#holonomic
from kimm_path_planner_ros_interface.msg import MobileTrajectory, Obstacle2D
from kimm_path_planner_ros_interface.srv import *

from behavior import BehaviorBase
import utils


class ApproachBaseBehavior(BehaviorBase):
    ''' 
    Get mobile path trajectory using
    reachability map and mobile path planner 
    '''
    def __init__(self, name, **params):
        super(ApproachBaseBehavior, self).__init__(name, **params)
        self.listener = tf.TransformListener()
        if self._hardware_if == 'vrep':
            self.service_name = '/sim_interface/set_motion'
            self.service_type = VrepSetJointTrajectory
        elif self._hardware_if == 'hw':
            self.service_name = '/hw_interface/set_motion'
            self.service_type = SetJointTrajectory
        
        # service client
        self.path_srv = rospy.ServiceProxy("/ns0/kimm_path_planner_ros_interface_server/plan_mobile_path", plan_mobile_path)
        self.pose_srv = rospy.ServiceProxy("/ir_server/find_positions", Repositioning)

        # pubisher
        self.path_pub = rospy.Publisher('/mobile_path', Path, queue_size=10)
        self.pose_pub = rospy.Publisher('/mobile_pose', PoseArray, queue_size=10)

        self.path = Path() 
        self.pose_array = PoseArray() 

        # init pose
        self.start_pose = Pose2D(x=0.0, y=0.0, theta=0.0)

        self.input_args = ['robot_group',
                            'target_object',
                            'static_object',
                            'dynamic_object']
        self.hardware_group = ['arm', 'mobile']

    def prepare_behavior(self):
        rospy.loginfo('preparing...%s' % self._name)
        return True

    def run_behavior(self):
        rospy.loginfo('running...%s' % self._name)
        return 1

    def get_motion(self, inputs):
        '''
        return the trajectory 
        '''
        rospy.loginfo("Calculating approaching mobile path..")
        res = self._call_ros_service(inputs)
        return res

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True


    def _call_ros_service(self, requirements):
        try:
            plan_req = MotionPlanRequest()
            plan_res = MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

            # target body
            robot_group = requirements.robot_group[0] # mobile and manipulator
                      
            # set obstacles
            obstacles = requirements.static_object + requirements.dynamic_object
            for obs in obstacles:
                plan_req.obstacle_ids.append(obs.id)
                plan_req.obstacles.append(obs.bb3d)

            # get bounding box of target object
            if len(requirements.target_object) == 0:
                rospy.logerr("Approaching target is not decided.")
                return plan_res
            else:
                target_object = requirements.target_object[0]

   
            # get mobile placement
            pose_req = RepositioningRequest()

            pose_req.max_dist = 1024.0
            pose_req.collision_offset = 0.3
            pose_req.strict_dual = False

            if MotionPlanRequest.BOTH_ARM == robot_group:
                pose_req.hand_type = RepositioningRequest.DUAL_HAND
                pose_req.collision_offset = 0.0
                pose_req.strict_dual = True
            elif MotionPlanRequest.LEFT_ARM == robot_group or MotionPlanRequest.LEFT_GRIPPER == robot_group:
                pose_req.hand_type = RepositioningRequest.LEFT_HAND
            elif MotionPlanRequest.RIGHT_ARM == robot_group or MotionPlanRequest.RIGHT_GRIPPER == robot_group:
                pose_req.hand_type = RepositioningRequest.RIGHT_HAND

            pose_req.Pt.x = target_object.bb3d.center.position.x
            pose_req.Pt.y = target_object.bb3d.center.position.y

            for i, obs in enumerate(obstacles):
                if obs.id != target_object.id:
                    bb2d = self.bbox_3d_to_2d(obs.bb3d)
                    pose_req.Obs.append(bb2d)

            pose_req.Cr.x = np.radians(0.0)
            pose_req.Cr.y = np.radians(30.1)

            pose_req.Ct.x = np.radians(-15)
            pose_req.Ct.y = np.radians(15)

            pose_req.section_definition.x = 0.05
            pose_req.section_definition.y = 1.0
            pose_req.section_definition.z = 0.02

            rospy.loginfo("[ApproachBaseBehavior] calculating base pose...")
            print(pose_req)

            # get mobile path
            start_pose = Pose2D(x=0.0, y=0.0, theta=0.0)
            path_req = plan_mobile_pathRequest()
            path_req.current_mobile_state = start_pose
            print(target_object.id)
            if target_object.id not in ['obj_courier_box', 'obj_tray', 'obj_human', 'obj_fridge']:
                pose_res = self.pose_srv(pose_req)
                print('approaching candidates:')
                print(pose_res)            

                if len(pose_res.candidates)>0:
                    # choose best pose
                    goal_pose = pose_res.candidates[0]
                    path_req.target_mobile_pose = goal_pose
                else:
                    return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_NO_SOLUTION)   
            
            #TODO: remove hard-coding for demo
            elif target_object.id in ['obj_courier_box', 'obj_tray', 'obj_human', 'obj_fridge']:
                target_goal = target_object.bb3d.center
                goal_pose = Pose()
                if target_object.id == 'obj_courier_box':
                    trans_pose = utils.create_pose([0, 3.2900e-01, 0],
                                                [0, 0, -0.707, 0.707]) 
                    goal_pose = utils.transform_pose(trans_pose, target_goal)

                elif target_object.id == 'obj_tray':
                    trans_pose = utils.create_pose([-4.6300e-01, 0, 0],
                                                [0, 0, 0, 1]) 
                    goal_pose = utils.transform_pose(trans_pose, target_goal)

                elif target_object.id == 'obj_human':
                    trans_pose = utils.create_pose([0.8, 0, 0],
                                                [0, 0, 1, 0]) 
                    goal_pose = utils.transform_pose(trans_pose, target_goal)

                elif target_object.id == 'obj_fridge':                        
                    trans_pose = utils.create_pose([0.61802059412003, -0.12404763698578, 0],
                                                            [0, 0, 1.0, 0]) 
                    temp_obstacles = []
                    for obs in obstacles:
                        if obs.id in ['obs_table', 'obs_fridge', 'obj_fridge_bottom_door', 'obj_wall_1', 'obj_wall_2']:
                            temp_obstacles.append(obs)
                    obstacles2d = self.get_obstacles(temp_obstacles)
                    path_req.Obstacles2D = obstacles2d

                    #if 'init' in requirements.constraints:  # init pose to open the fridge door
                    if rospy.has_param('fridge_isopen'):
                        is_open = rospy.get_param('fridge_isopen')
                        if is_open:
                            pass
                        elif (is_open == False or is_open == 'False') or 'init' in requirements.constraints:
                            trans_pose = utils.create_pose([+8.7531e-01, -2.9040e-01, 0],
                                                            [0, 0, -1.0, 0]) 
                            path_req.Obstacles2D = []

                    goal_pose = utils.transform_pose(trans_pose, target_goal)
                path_req.target_mobile_pose.x = goal_pose.position.x
                path_req.target_mobile_pose.y = goal_pose.position.y
                angles = tf.transformations.euler_from_quaternion([goal_pose.orientation.x, 
                                                                goal_pose.orientation.y, 
                                                                goal_pose.orientation.z, 
                                                                goal_pose.orientation.w])
                path_req.target_mobile_pose.theta = angles[2]
            else:
                return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_NO_SOLUTION)              
            

            rospy.loginfo("[ApproachBaseBehavior] calculating base motion path...")
            print(path_req)

            iter=0
            isProblem = True
            while(not rospy.is_shutdown() and iter<5 and isProblem): # try 5 times
                path_plan = self.path_srv(path_req)
                last_pt = path_plan.mobile_path.points[-1]
                dist = math.sqrt(math.pow(last_pt.x-path_req.target_mobile_pose.x, 2) + math.pow(last_pt.y-path_req.target_mobile_pose.y, 2))
                if dist < 0.05: 
                    isProblem = False
            
            if isProblem:
                MotionPlanResponse(planResult=MotionPlanResponse.ERROR_NO_SOLUTION)
            
            path, pose_array = self.res_to_msg(path_plan.mobile_path)

            # publish results
            iter=0
            while(not rospy.is_shutdown() and iter<100):
                self.path_pub.publish(path)
                self.pose_pub.publish(pose_array)
                iter+=1

            plan_res.planResult = MotionPlanResponse.SUCCESS
            plan_res.pathTrajectory = path

            return plan_res

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

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

    def res_to_msg(self, plan_result):
        path = Path()
        pose_array = PoseArray()

        path.header.stamp = pose_array.header.stamp = rospy.Time().now()
        path.header.frame_id = pose_array.header.frame_id = 'base_footprint'        

        for pt in plan_result.points:
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
            path.poses.append(pose)
            pose_array.poses.append(pose.pose)

        return path, pose_array

    def get_obstacles(self, objects):
        '''
        convert msg format from vision_msgs/BoundingBox3D into mobile_path_planner/Obstacle2D
        '''
        obstacles = []
        for obj in objects:            
            # get vertex points
            vertex = utils.get_vertex_from_bb3d(obj.bb3d)
            # calculate convex hull
            hull = utils.get_hull_from_points([(x,y) for x, y, z in vertex])
            robot_diameter = 0.8
            bb2d = utils.get_bb2d_from_hull(hull[0], 
                                    hull[1] + robot_diameter,
                                    hull[2] + robot_diameter, 
                                    hull[3])
            vertex = utils.get_vertex_from_bb2d(bb2d, minmax=True)
            obstacles.append(Obstacle2D(x1=std_msgs.msg.Float64(vertex[0][0]),
                                        y1=std_msgs.msg.Float64(vertex[0][1]),
                                        x2=std_msgs.msg.Float64(vertex[1][0]),
                                        y2=std_msgs.msg.Float64(vertex[1][1])))

        return obstacles
