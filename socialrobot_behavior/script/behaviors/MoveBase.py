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
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import Pose, PoseArray, Pose2D, PoseStamped
from vision_msgs.msg import BoundingBox2D, BoundingBox3D
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path

import tf
import tf.transformations as tfm
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion
import numpy as np

from kimm_path_planner_ros_interface.msg import MobileTrajectory, Obstacle2D
from kimm_path_planner_ros_interface.srv import *

from behavior import BehaviorBase
import utils

class MoveBaseBehavior(BehaviorBase):
    ''' 
    Get mobile path trajectory
    '''
    def __init__(self, name, **params):
        super(MoveBaseBehavior, self).__init__(name, **params)
        self.listener = tf.TransformListener()        
        
        # service client
        self.path_srv = rospy.ServiceProxy("/ns0/kimm_path_planner_ros_interface_server/plan_mobile_path", plan_mobile_path)

        # pubisher
        self.path_pub = rospy.Publisher('/mobile_path', Path, queue_size=10)
        self.pose_pub = rospy.Publisher('/mobile_pose', PoseArray, queue_size=10)

        self.path = Path() 
        self.pose_array = PoseArray() 

        # init pose
        self.start_pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        self.robot_radius = 0.25

        self.input_args = ['robot_group', 'goal_position',
                            'static_object',
                            'dynamic_object']
        self.hardware_group = ['mobile']

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
        rospy.loginfo("Calculating mobile path..")
        res = self._call_ros_service(inputs)
        return res

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True


    def _call_ros_service(self, requirements):
        print(requirements)
        try:
            plan_res = MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

            # target body
            robot_group = requirements.robot_group[0]
            if robot_group != MotionPlanRequest.MOBILE_BASE:
                rospy.logerr("[MoveBaseBehavior] robot group is not MOBILE_BASE.")
                return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_INPUT)
                      
            # set obstacles
            obstacles = requirements.static_object + requirements.dynamic_object
            obstacles2d = self.get_obstacles(obstacles)

            # goal pose
            goal_positions = requirements.goal_position
            if len(goal_positions) == 0:
                rospy.logerr("[MoveBaseBehavior] no goal pose.")
                return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_INPUT)
                
            else:
                # set path
                start_pose = Pose2D(x=0.0, y=0.0, theta=0.0)
                goal_pose = Pose2D()

                for goal in goal_positions:
                    goal_pose = utils.pose_2_pose2d(goal.pose)
                
                    #
                    path_req = plan_mobile_pathRequest()
                    path_req.current_mobile_state = start_pose
                    path_req.target_mobile_pose = goal_pose

                    # in local plan ignore obstacles
                    dist = math.sqrt(math.pow(goal_pose.x,2) + math.pow(goal_pose.y,2))
                    if dist > 0.5: 
                        path_req.Obstacles2D = obstacles2d

                    rospy.loginfo("[MoveBaseBehavior] calculating base motion path...")
                    #print(path_req)
                    path_plan = self.path_srv(path_req)
                
                    if len(path_plan.mobile_path.points)>0:
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
                    else:
                        rospy.logerr("[MoveBaseBehavior] No solution.")
                        return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_NO_SOLUTION)

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

    def res_to_msg(self, plan_result):
        '''
        pose2d array to markers
        '''
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
            robot_diameter = 0.7
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