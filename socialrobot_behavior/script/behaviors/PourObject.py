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
import numpy as np
import random
from behavior import BehaviorBase

class PourObjectBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(PourObjectBehavior, self).__init__(name, **params)
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
                'left_offset': [-0.0055, 0.03],
                'right_offset': [-0.0055, 0.03]
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
        rospy.loginfo("Calculating %s motion.." % self._name)
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
        target_object[0] : grasped object
        target_object[1] : container object
        '''
        try:
            plan_req = MotionPlanRequest()
            plan_res = MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

            # target body
            robot_group = requirements.robot_group[0]    
            if robot_group == MotionPlanRequest.RIGHT_ARM or robot_group == MotionPlanRequest.RIGHT_GRIPPER:
                robot_group = MotionPlanRequest.RIGHT_ARM
            elif robot_group == MotionPlanRequest.LEFT_ARM or robot_group == MotionPlanRequest.LEFT_GRIPPER:
                robot_group = MotionPlanRequest.LEFT_ARM      
            plan_req.targetBody = robot_group

            # set obstacles
            obstacles = requirements.static_object + requirements.dynamic_object
            for obs in obstacles:
                plan_req.obstacle_ids.append(obs.id)
                plan_req.obstacles.append(obs.bb3d)

            # get bounding box of target object
            if len(requirements.target_object) != 2:
                rospy.logerr("Grasped & Container objects are not decided.")
                return plan_res
            else:
                grasped_object = requirements.target_object[0]
                container_object = requirements.target_object[1]
                plan_req.targetObject = [grasped_object.id]

            # if object has affordances and grasp direction, trasnpose them based on robot frame            
            grasped_object = self._utils.transform_object(grasped_object)         
            container_object = self._utils.transform_object(container_object)

            # grasped object and container have OPEN affordance
            #TODO
            desired_eef_poses = []

            # build sample poses
            sample_poses = self.create_sample_poses(robot_group, grasped_object, container_object)

            # calculate wrist goal pose from eef pose
            pre_eef = []
            way_eef = []
            goal_eef = []
            
            self.get_eef_poses(robot_group, sample_poses, grasped_object, container_object, pre_eef, way_eef, goal_eef)
            pre_wrist, way_wrist, goal_wrist = self.eef_to_wrist(pre_eef, way_eef, goal_eef, robot_group)

            for idx in range(len(goal_eef)):
                rospy.loginfo('[PourObject Behavior] %d/%d calculating approach pose to target nearby...', idx+1, len(goal_eef))

                # publish goal eef pose
                self.publish_goal_eef_pose(goal_eef[idx], "approach_sample_pose", "base_footprint")

                # approach motion
                plan_req.targetPose = [pre_wrist[idx]]
                plan_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL

                # keep eef orientation
                #TODO: describe eef frame based 
                plan_req.orientation_constraint.position.x = 3.14 #ignore this axis
                plan_req.orientation_constraint.position.y = 0.4 #30degree
                plan_req.orientation_constraint.position.z = 0.4

                approach_plan = self.srv_plan(plan_req)
                if approach_plan.planResult == MotionPlanResponse.SUCCESS:
                    
                    rospy.loginfo('[PourObject Behavior] %d/%d pouring pose to target is found.', idx+1, len(goal_eef))
                    # get last joint state for waypoint             
                    joint_state = sensor_msgs.msg.JointState()
                    joint_state.header = approach_plan.jointTrajectory.header
                    joint_state.name = approach_plan.jointTrajectory.joint_names
                    joint_state.position = approach_plan.jointTrajectory.points[-1].positions

                    # waypoint motion
                    second_req = MotionPlanRequest()
                    second_req.obstacle_ids = plan_req.obstacle_ids
                    second_req.obstacles = plan_req.obstacles
                    second_req.currentJointState = joint_state
                    second_req.targetPose = [way_wrist[idx]]
                    second_req.orientation_constraint.position.x = 0.1
                    second_req.orientation_constraint.position.y = 0.1
                    second_req.orientation_constraint.position.z = 3.14
                    second_plan = self.srv_plan(second_req)

                    if second_plan.planResult == MotionPlanResponse.SUCCESS:
                        # get last joint state for waypoint             
                        joint_state = sensor_msgs.msg.JointState()
                        joint_state.header = second_plan.jointTrajectory.header
                        joint_state.name = second_plan.jointTrajectory.joint_names
                        joint_state.position = second_plan.jointTrajectory.points[-1].positions

                        # pouring motion
                        thrid_req = MotionPlanRequest()
                        thrid_req.obstacle_ids = plan_req.obstacle_ids
                        thrid_req.obstacles = plan_req.obstacles
                        thrid_req.currentJointState = joint_state
                        thrid_req.targetPose = [goal_wrist[idx]]
                        thrid_req.orientation_constraint.position.x = 0.1
                        thrid_req.orientation_constraint.position.y = 0.1
                        thrid_req.orientation_constraint.position.z = 3.14
                        third_plan = self.srv_plan(thrid_req)

                        if third_plan.planResult == MotionPlanResponse.SUCCESS:
                            rospy.loginfo('final pouring pose to target is found!')
                            merged_plan = self._utils.connect_motion_plan(approach_plan, second_plan, rospy.Duration(0.0))   
                            merged_plan = self._utils.connect_motion_plan(merged_plan, third_plan, rospy.Duration(0.0))
                            
                            # reverse pouring motion for put the object onto the table
                            reversed_plan = self._utils.reverse_motion_plan(merged_plan)
                            final_plan = self._utils.connect_motion_plan(merged_plan, reversed_plan, rospy.Duration(2.0))
         
                            final_plan.planResult = MotionPlanResponse.SUCCESS
                            return final_plan

            return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_NO_SOLUTION)

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

    def get_eef_poses(self, robot_group, candidate_poses, grasped_object, container_object, pre_eef_poses, waypoint_eef_poses, goal_eef_poses):
        '''
        calculate eef poses from approaching angles and target object
        '''
        gr_size = container_object.bb3d.size
        gr_pos = container_object.bb3d.center.position
        gr_rot = container_object.bb3d.center.orientation

        con_size = container_object.bb3d.size
        con_pos = container_object.bb3d.center.position
        con_rot = container_object.bb3d.center.orientation

        if robot_group is PlannerInputs.LEFT_ARM or robot_group is PlannerInputs.LEFT_ARM_WITHOUT_WAIST:
            i = 1
        elif robot_group is PlannerInputs.RIGHT_ARM or robot_group is PlannerInputs.RIGHT_ARM_WITHOUT_WAIST:
            i = -1

        # gripper end-effector position from target object
        for desired_pose in candidate_poses:

            # approach
            trans_pose = self._utils.create_pose([-min(gr_size.x, gr_size.y)/2.0, i*(gr_size.x + con_size.x/2.0), gr_size.z],
                                            [0, 0, 0, 1])                                            
            pre_pose = self._utils.get_grasp_pose(self._utils.transform_pose(trans_pose, desired_pose), robot_group)
            
            # waypoint
            quat = self.rot_x(45*i)
            trans_pose = self._utils.create_pose([-min(gr_size.x, gr_size.y)/2.0, i*(gr_size.x + con_size.x/2.0), gr_size.z],
                                            quat)   
            waypoint_pose = self._utils.get_grasp_pose(self._utils.transform_pose(trans_pose, desired_pose), robot_group)
            
            # pouring
            quat = self.rot_x(90*i)
            trans_pose = self._utils.create_pose([-min(gr_size.x, gr_size.y)/2.0, i*(gr_size.x + con_size.x/2.0), gr_size.z],
                                            quat)   
            grasp_pose = self._utils.get_grasp_pose(self._utils.transform_pose(trans_pose, desired_pose), robot_group)

            pre_eef_poses.append(pre_pose)
            waypoint_eef_poses.append(waypoint_pose)
            goal_eef_poses.append(grasp_pose)
    
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
        eef_to_wrist_pose = self._utils.create_pose(eef_to_wrist_trans, eef_to_wrist_rot)

        # transpose
        pre_wrist_poses = []
        waypoint_wrist_poses = []
        goal_wrist_poses = []
        for i in range(len(pre_eef_poses)):
            pre_wrist_poses.append(self._utils.transform_pose(eef_to_wrist_pose, pre_eef_poses[i]))
            waypoint_wrist_poses.append(self._utils.transform_pose(eef_to_wrist_pose, waypoint_eef_poses[i]))
            goal_wrist_poses.append(self._utils.transform_pose(eef_to_wrist_pose, goal_eef_poses[i]))

        return pre_wrist_poses, waypoint_wrist_poses, goal_wrist_poses

    def create_sample_poses(self, robot_group, grasped_object, container_object):
        '''
        create sample grasp pose around target object
        '''
        candidate_poses = []
        target_pose = Pose()
        target_pose.orientation = Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)
        target_pose.position = container_object.bb3d.center.position

        if robot_group is PlannerInputs.LEFT_ARM or robot_group is PlannerInputs.LEFT_ARM_WITHOUT_WAIST:
            rot_deg = [0, -15, -30, -45, -60, -75, -90]
        elif robot_group is PlannerInputs.RIGHT_ARM or robot_group is PlannerInputs.RIGHT_ARM_WITHOUT_WAIST:
            rot_deg = [0, 15, 30, 45, 60, 75, 90]
        rot_deg.reverse()
        for deg in rot_deg:
            rot_pose = self._utils.create_pose([0, 0, 0],
                                         self.rot_z(deg))
            candidate_poses.append(self._utils.transform_pose(rot_pose, target_pose))

        return candidate_poses

    def publish_goal_eef_pose(self, eef_pose, eef_frame, base_frame):
        self.talker.sendTransform((eef_pose.position.x, eef_pose.position.y, eef_pose.position.z),
            (eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z,eef_pose.orientation.w), rospy.Time.now(), eef_frame, base_frame)