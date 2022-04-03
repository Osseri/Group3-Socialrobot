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
from geometry_msgs.msg import Pose, Point, Quaternion

import tf
import tf.transformations as tfm
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion
import numpy as np
import random
import utils
from behavior import BehaviorBase

class PutupObjectBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(PutupObjectBehavior, self).__init__(name, **params)
        self.listener = tf.TransformListener()
        if self._hardware_if == 'vrep':
            self.service_name = '/sim_interface/set_motion'
            self.service_type = VrepSetJointTrajectory
        elif self._hardware_if == 'hw':
            self.service_name = '/hw_interface/set_motion'
            self.service_type = SetJointTrajectory
        armplan_srv = '/motion_plan/move_arm'

        self.talker = tf.TransformBroadcaster(queue_size=10)
        self.srv_plan = rospy.ServiceProxy(armplan_srv, MotionPlan)

        # initial pose
        self.gripper_pose = {}
        self.approach_option = None
        self.rot_x = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (1, 0, 0))
        self.rot_y = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 1, 0))
        self.rot_z = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 0, 1))

        # get hardware
        robot_name = rosparam.get_param("/robot_name")

        if robot_name == 'skkurobot':
            self.gripper_pose = {
                'left': '7dof_RISE_wrist_link',
                'right': '6dof_connection_link',
                'left_offset': [-0.02, 0.08],
                'right_offset': [-0.025, 0.08]
            }
        elif robot_name == 'social_robot':
            self.gripper_pose = {
                'left': 'LHand_base',
                'right': 'RHand_base',
                'left_offset': [-0.005, 0.05],
                'right_offset': [-0.005, 0.05]  # grasp, pre-grasp offset
            }

        self.input_args = ['robot_group',
                            'target_object',
                            'static_object',
                            'dynamic_object']
        self.hardware_group = ['arm']

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
        rospy.loginfo("Calculating put on motion..")
        res = self._call_ros_service(inputs)
        return res

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True

    def get_sample_degree(self, step, max_degree):
        sample_list = [0.0]
        for i in range(1, max_degree + 1, step):
            sample_list.append(i)
            sample_list.append(-i)

        return sample_list

    def _call_ros_service(self, requirements):
        '''
        requirements.robot_group[0] : group of robot
        requirements.target_object[0] : above object
        requirements.target_object[1] : below object        
        '''
        try:
            plan_req = MotionPlanRequest()
            plan_res = MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

            # target body
            robot_group = requirements.robot_group[0]    
            if robot_group == MotionPlanRequest.RIGHT_GRIPPER:
                robot_group = MotionPlanRequest.RIGHT_ARM
            elif robot_group == robot_group == MotionPlanRequest.RIGHT_ARM or robot_group == robot_group == MotionPlanRequest.RIGHT_ARM_WITHOUT_WAIST:
                robot_group = requirements.robot_group[0]
            elif robot_group == MotionPlanRequest.LEFT_GRIPPER:
                robot_group = MotionPlanRequest.LEFT_ARM  
            elif robot_group == robot_group == MotionPlanRequest.LEFT_ARM or robot_group == robot_group == MotionPlanRequest.LEFT_ARM_WITHOUT_WAIST:
                robot_group = requirements.robot_group[0]
            plan_req.targetBody = robot_group

            # get constraints
            constraints = requirements.constraints
            aff_allow = []
            aff_except = []
            for const in constraints:
                if 'not' in const:
                    aff_except.append(const)
                else:
                    aff_allow.append(const)
                    
                if 'slow' in const:
                    plan_req.acceleration_scaling_factor = 0.05
                    plan_req.velocity_scaling_factor = 0.05

            # get bounding box of target object
            if len(requirements.target_object) == 0:
                rospy.logerr("Approaching target is not decided.")
                return plan_res
            else:
                grasped_object = requirements.target_object[0]
                base_object = requirements.target_object[1]
                plan_req.targetObject = [grasped_object.id]            

            # set obstacles from perception
            obstacles = requirements.static_object + requirements.dynamic_object
            for obs in obstacles:
                plan_req.obstacle_ids.append(obs.id)
                plan_req.obstacles.append(obs.bb3d)

            # if object has affordances and grasp direction, transpose them based on robot frame            
            base_object = utils.transform_object(base_object)
            
            for aff in base_object.affordance:
                if self.check_affordance(aff, exception_constraints=aff_except):
                    # add affordance as obstacle
                    plan_req.obstacle_ids.append(aff.id)
                    plan_req.obstacles.append(aff.bb3d)

            # brute-force search approching pose
            pre_eef = []
            way_eef = []
            goal_eef = []

            sample_poses = self.create_sample_poses(robot_group, base_object)
            self.get_eef_poses(robot_group, sample_poses, grasped_object, pre_eef, way_eef, goal_eef)
            pre_wrist, way_wrist, goal_wrist = self.eef_to_wrist(pre_eef, way_eef, goal_eef, robot_group)

            for idx in range(len(goal_eef)):
                rospy.loginfo('[PutOn Behavior] %d/%d calculating approach pose to target...', idx+1, len(goal_eef))

                # publish goal eef pose
                self.publish_goal_eef_pose(goal_eef[idx], "approach_sample_pose", "base_footprint")

                # pre-grasp motion
                plan_req.targetPose = [goal_wrist[idx]]
                plan_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL
                
                # keep eef orientation
                plan_req.orientation_constraint.position.x = 3.14 #ignore this axis
                plan_req.orientation_constraint.position.y = 0.4 #30degree
                plan_req.orientation_constraint.position.z = 0.4
                
                grasp_plan = self.srv_plan(plan_req)
                if grasp_plan.planResult == MotionPlanResponse.SUCCESS:    
                    rospy.loginfo('final approach pose to target is found!')
                    return grasp_plan
                        
            rospy.loginfo('no solution of final approach pose to target')
            return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_NO_SOLUTION)

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

    def find_approach_pose(self, robot_group, target_object, pre_eef, way_eef, goal_eef):
        target_size = target_object.bb3d.size
        target_pose = target_object.bb3d.center

        if robot_group is PlannerInputs.LEFT_ARM or robot_group is PlannerInputs.LEFT_ARM_WITHOUT_WAIST:
            offset = self.gripper_pose['left_offset']
            i=1
        elif robot_group is PlannerInputs.RIGHT_ARM or robot_group is PlannerInputs.RIGHT_ARM_WITHOUT_WAIST:
            offset = self.gripper_pose['right_offset']
            i=-1

        # create approaching poses
        candidate_poses = []
        
        #TODO: re-orientation bounding box axis
        euler = euler_from_quaternion([target_pose.orientation.x, target_pose.orientation.y, 
                                        target_pose.orientation.z, target_pose.orientation.w])
        quat = quaternion_from_euler(0,0,euler[2])
        sample_pose = Pose()
        sample_pose.orientation = Quaternion(x=quat[0],y=quat[1],z=quat[2],w=quat[3])
        sample_pose.position = target_pose.position

        # get 4 approaching pose
        # y
        trans = utils.create_pose([0, target_size.y/2.0, 0],
                                        self.rot_z(-90))
        candidate_poses.append(utils.transform_pose(trans, sample_pose))
        # -y
        trans = utils.create_pose([0, -target_size.y/2.0, 0],
                                        self.rot_z(90))
        candidate_poses.append(utils.transform_pose(trans, sample_pose))
        # x
        trans = utils.create_pose([target_size.x/2.0, 0, 0],
                                        self.rot_z(180))
        candidate_poses.append(utils.transform_pose(trans, sample_pose))
        # -x
        trans = utils.create_pose([-target_size.x/2.0, 0, 0],
                                        self.rot_z(0))
        candidate_poses.append(utils.transform_pose(trans, sample_pose))
        

        # create eef poses
        for desired_pose in candidate_poses:

            # pre-grasp
            trans_pose = utils.create_pose([-(offset[1]), 0, 0],
                                            [0, 0, 0, 1])                                            
            pre_grasp_pose = utils.get_grasp_pose(utils.transform_pose(trans_pose, desired_pose), robot_group)
            
            # waypoint
            trans_pose = utils.create_pose([-((offset[0]+offset[1])/2.0), 0, 0],
                                            [0, 0, 0, 1])   
            waypoint_pose = utils.get_grasp_pose(utils.transform_pose(trans_pose, desired_pose), robot_group)
            
            # grasp
            trans_pose = utils.create_pose([-(offset[0]), 0, 0],
                                            [0, 0, 0, 1])   
            grasp_pose = utils.get_grasp_pose(utils.transform_pose(trans_pose, desired_pose), robot_group)

            pre_eef.append(pre_grasp_pose)
            way_eef.append(waypoint_pose)
            goal_eef.append(grasp_pose)

    def check_affordance(self, affordance, allow_constraints=[], exception_constraints=[]):
        for exception in exception_constraints:
            if affordance.id in exception:
                return False
        return True

    def get_eef_poses(self, robot_group, candidate_poses, grasped_object, pre_eef_poses, waypoint_eef_poses, goal_eef_poses):
        '''
        calculate eef poses from approaching angles and target object
        '''
        target_size = grasped_object.bb3d.size
        target_pos = grasped_object.bb3d.center.position
        target_rot = grasped_object.bb3d.center.orientation

        if robot_group is PlannerInputs.LEFT_ARM or robot_group is PlannerInputs.LEFT_ARM_WITHOUT_WAIST:
            offset = self.gripper_pose['left_offset']
        elif robot_group is PlannerInputs.RIGHT_ARM or robot_group is PlannerInputs.RIGHT_ARM_WITHOUT_WAIST:
            offset = self.gripper_pose['right_offset']

        # gripper end-effector position from target object
        for desired_pose in candidate_poses:

            # pre
            trans_pose = utils.create_pose([0, 0, target_size.z/2.0 + 0.05],
                                            [0, 0, 0, 1])                                            
            pre_grasp_pose = utils.get_grasp_pose(utils.transform_pose(trans_pose, desired_pose), robot_group)
            
            # waypoint
            trans_pose = utils.create_pose([0, 0, target_size.z/2.0 + 0.02],
                                            [0, 0, 0, 1])   
            waypoint_pose = utils.get_grasp_pose(utils.transform_pose(trans_pose, desired_pose), robot_group)
            
            # goal
            trans_pose = utils.create_pose([0, 0, target_size.z/2.0],
                                            [0, 0, 0, 1])   
            grasp_pose = utils.get_grasp_pose(utils.transform_pose(trans_pose, desired_pose), robot_group)

            pre_eef_poses.append(pre_grasp_pose)
            waypoint_eef_poses.append(waypoint_pose)
            goal_eef_poses.append(grasp_pose)

        #return pre_eef_poses, waypoint_eef_poses, goal_eef_poses
    
    def eef_to_wrist(self, pre_eef_poses, waypoint_eef_poses, goal_eef_poses, robot_group):

        # get transform between eef and wrist
        eef_to_wrist_trans = None
        eef_to_wrist_rot = None
        is_trans = False
        while is_trans == False:
            try:
                if robot_group is PlannerInputs.LEFT_ARM or robot_group is PlannerInputs.LEFT_ARM_WITHOUT_WAIST:
                    eef_to_wrist_trans, eef_to_wrist_rot = self.listener.lookupTransform(
                        '/left_end_effect_point', self.gripper_pose['left'], rospy.Time(0))

                elif robot_group is PlannerInputs.RIGHT_ARM or robot_group is PlannerInputs.RIGHT_ARM_WITHOUT_WAIST:
                    eef_to_wrist_trans, eef_to_wrist_rot = self.listener.lookupTransform(
                        '/right_end_effect_point', self.gripper_pose['right'], rospy.Time(0))
                is_trans = True
            except:
                pass
            
        # vector to pose
        eef_to_wrist_pose = utils.create_pose(eef_to_wrist_trans, eef_to_wrist_rot)

        # transpose
        pre_wrist_poses = []
        waypoint_wrist_poses = []
        goal_wrist_poses = []
        for i in range(len(pre_eef_poses)):
            pre_wrist_poses.append(utils.transform_pose(eef_to_wrist_pose, pre_eef_poses[i]))
            waypoint_wrist_poses.append(utils.transform_pose(eef_to_wrist_pose, waypoint_eef_poses[i]))
            goal_wrist_poses.append(utils.transform_pose(eef_to_wrist_pose, goal_eef_poses[i]))

        return pre_wrist_poses, waypoint_wrist_poses, goal_wrist_poses

    def create_sample_poses(self, robot_group, base_object):
        '''
        create sample puton pose around base object
        '''
        candidate_poses = []
        target_pose = Pose()
        target_pose.orientation = Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)
        target_pose.position = base_object.bb3d.center.position
        target_pose.position.z += base_object.bb3d.size.z/2.0 + 0.01

        if robot_group is PlannerInputs.LEFT_ARM or robot_group is PlannerInputs.LEFT_ARM_WITHOUT_WAIST:
            rot_deg = [0, -15, -30, -45, -60, -75, -90]
        elif robot_group is PlannerInputs.RIGHT_ARM or robot_group is PlannerInputs.RIGHT_ARM_WITHOUT_WAIST:
            rot_deg = [0, 15, 30, 45, 60, 75, 90]
        rot_deg.reverse()
        for deg in rot_deg:
            rot_pose = utils.create_pose([0, 0, 0],
                                         self.rot_z(deg))
            candidate_poses.append(utils.transform_pose(rot_pose, target_pose))

        return candidate_poses

    def publish_goal_eef_pose(self, eef_pose, eef_frame, base_frame):
        self.talker.sendTransform((eef_pose.position.x, eef_pose.position.y, eef_pose.position.z),
            (eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z,eef_pose.orientation.w), rospy.Time.now(), eef_frame, base_frame)
