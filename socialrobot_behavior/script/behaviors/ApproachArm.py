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
from socialrobot_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

import tf
import tf.transformations as tfm
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion
import numpy as np
import random
from behavior import BehaviorBase

class ApproachArmBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(ApproachArmBehavior, self).__init__(name, **params)
        self.listener = tf.TransformListener()
            
        armplan_srv = '/motion_plan/move_arm'
        self.talker = tf.TransformBroadcaster(queue_size=10)
        self.srv_plan = rospy.ServiceProxy(armplan_srv, MotionPlan)

        # initial pose
        self.gripper_pose = {}
        self.approach_option = None
        self.rot_x = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (1, 0, 0))
        self.rot_y = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 1, 0))
        self.rot_z = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 0, 1))

        # grasp offset
        self.z_offset = 0.01

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
                'left_offset': [-0.015, 0.06],
                'right_offset': [-0.015, 0.06]  # grasp, pre-grasp offset
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
        rospy.loginfo("Calculating approaching motion..")
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

            # set obstacles
            obstacles = requirements.static_object + requirements.dynamic_object
            has_workspace = False
            for obs in obstacles:
                if obs.id in ['obj_table','obj_fridge']:
                    has_workspace = True
                plan_req.obstacle_ids.append(obs.id)
                plan_req.obstacles.append(obs.bb3d)

            # get bounding box of target object
            if len(requirements.target_object) == 0:
                rospy.logerr("Approaching target is not decided.")
                return plan_res
            else:
                target_object = requirements.target_object[0]
                target_object.bb3d.center.position.z += self.z_offset
                # if target in known object, get data from database
                self._utils.update_object_info(target_object)
                # # TODO: hard-coding for demo
                # if target_object.id == 'obj_fridge':
                #     for i,aff in enumerate(target_object.affordance):
                #         if aff.id == 'obj_fridge_bottom_door':
                #             if rospy.has_param('fridge_isopen'):
                #                 if rospy.get_param('fridge_isopen'):
                #                     target_object.affordance[i].bb3d.center.position.x = +4.5400e-01
                #                     target_object.affordance[i].bb3d.center.position.y = +3.9664e-01
                #                     target_object.affordance[i].bb3d.center.position.z = -2.1505e-01
                #                     target_object.affordance[i].bb3d.center.orientation.x = 0.0
                #                     target_object.affordance[i].bb3d.center.orientation.y = 0.0
                #                     target_object.affordance[i].bb3d.center.orientation.z = 0.866
                #                     target_object.affordance[i].bb3d.center.orientation.w = 0.5

            # if object has affordances and grasp direction, transpose them based on robot frame            
            target_object = self._utils.transform_object(target_object)

            # if no workspace in obstacles, create virtual workspace
            if not has_workspace:
                table = self._utils.create_workspace(target_object.bb3d)
                plan_req.obstacle_ids.append(table.id)
                plan_req.obstacles.append(table.bb3d)

            ## 1.get grasp point from known object database
            pre_eef = []
            way_eef = []
            goal_eef = []
            desired_eef_poses = []
            has_grasp_point = False

            # object grasp point
            for gr_pt in target_object.grasp_point:
                desired_eef_poses.append(gr_pt)
                has_grasp_point = True
            self.get_eef_poses(robot_group, desired_eef_poses, target_object, pre_eef, way_eef, goal_eef)

            # affordance grasp point
            for aff in target_object.affordance:
                if self.check_affordance(aff, exception_constraints=aff_except):                    
                    desired_eef_poses = []
                    for gr_pt in aff.grasp_point:
                        desired_eef_poses.append(gr_pt)
                        has_grasp_point = True
                    self.get_eef_poses(robot_group, desired_eef_poses, aff, pre_eef, way_eef, goal_eef)

                # sub-affordance grasp point
                for sub_aff in aff.affordance:
                    if self.check_affordance(aff, exception_constraints=aff_except): 
                        desired_eef_poses = []
                        for gr_pt in sub_aff.grasp_point:
                            desired_eef_poses.append(gr_pt)
                            has_grasp_point = True
                        self.get_eef_poses(robot_group, desired_eef_poses, sub_aff, pre_eef, way_eef, goal_eef)

            if not has_grasp_point:
                # 2. get grasp approaching pose from object shape         
                self.find_approach_pose(robot_group, target_object, pre_eef, way_eef, goal_eef)

                # 3. brute-force search approching pose
                sample_poses = self.create_sample_poses(robot_group, target_object)
                self.get_eef_poses(robot_group, sample_poses, target_object, pre_eef, way_eef, goal_eef)
            
            # convert eef coordinate to wrist coordinate for moveit planning
            pre_wrist, way_wrist, goal_wrist = self.eef_to_wrist(pre_eef, way_eef, goal_eef, robot_group)

            for idx in range(len(goal_eef)):
                rospy.loginfo('[ApproachArm Behavior] %d/%d calculating approach pose to target nearby...', idx+1, len(goal_eef))

                # publish goal eef pose
                self.publish_goal_eef_pose(goal_eef[idx], "approach_sample_pose", "base_footprint")

                # pre-grasp motion
                plan_req.targetPose = [pre_wrist[idx]]
                plan_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL

                pre_grasp_plan = self.srv_plan(plan_req)
                if pre_grasp_plan.planResult == MotionPlanResponse.SUCCESS:
                    rospy.loginfo('[ApproachArm Behavior] %d/%d approach pose to target nearby is found.', idx+1, len(goal_eef))
                    # get last joint state for waypoint
                    waypoint_pos = copy.copy(pre_grasp_plan.jointTrajectory.points[-1].positions)
                    time_from_start = copy.copy(pre_grasp_plan.jointTrajectory.points[-1].time_from_start)

                    joint_state = sensor_msgs.msg.JointState()
                    joint_state.header = pre_grasp_plan.jointTrajectory.header
                    joint_state.name = pre_grasp_plan.jointTrajectory.joint_names
                    joint_state.position = pre_grasp_plan.jointTrajectory.points[-1].positions

                    # grasp motion
                    second_req = MotionPlanRequest()
                    second_req.obstacle_ids = plan_req.obstacle_ids
                    second_req.obstacles = plan_req.obstacles
                    second_req.currentJointState = joint_state
                    second_req.targetPose = [way_wrist[idx], goal_wrist[idx]]
                    second_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL_WITH_IGNORE_OBSTACLES
                    grasp_plan = self.srv_plan(second_req)

                    if grasp_plan.planResult == MotionPlanResponse.SUCCESS and len(grasp_plan.jointTrajectory.points)>1:
                        final_plan = self._utils.connect_motion_plan(pre_grasp_plan, grasp_plan)
                        final_plan.planResult = MotionPlanResponse.SUCCESS
                        return final_plan
                        
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
        trans = self._utils.create_pose([0, target_size.y/2.0, 0],
                                        self.rot_z(-90))
        candidate_poses.append(self._utils.transform_pose(trans, sample_pose))
        # -y
        trans = self._utils.create_pose([0, -target_size.y/2.0, 0],
                                        self.rot_z(90))
        candidate_poses.append(self._utils.transform_pose(trans, sample_pose))
        # x
        trans = self._utils.create_pose([target_size.x/2.0, 0, 0],
                                        self.rot_z(180))
        candidate_poses.append(self._utils.transform_pose(trans, sample_pose))
        # -x
        trans = self._utils.create_pose([-target_size.x/2.0, 0, 0],
                                        self.rot_z(0))
        candidate_poses.append(self._utils.transform_pose(trans, sample_pose))
        

        # create eef poses
        for desired_pose in candidate_poses:

            # pre-grasp
            trans_pose = self._utils.create_pose([-(offset[1]), 0, 0],
                                            [0, 0, 0, 1])                                            
            pre_grasp_pose = self._utils.get_grasp_pose(self._utils.transform_pose(trans_pose, desired_pose), robot_group)
            
            # waypoint
            trans_pose = self._utils.create_pose([-((offset[0]+offset[1])/2.0), 0, 0],
                                            [0, 0, 0, 1])   
            waypoint_pose = self._utils.get_grasp_pose(self._utils.transform_pose(trans_pose, desired_pose), robot_group)
            
            # grasp
            trans_pose = self._utils.create_pose([-(offset[0]), 0, 0],
                                            [0, 0, 0, 1])   
            grasp_pose = self._utils.get_grasp_pose(self._utils.transform_pose(trans_pose, desired_pose), robot_group)

            pre_eef.append(pre_grasp_pose)
            way_eef.append(waypoint_pose)
            goal_eef.append(grasp_pose)

    def check_affordance(self, affordance, allow_constraints=[], exception_constraints=[]):
        for exception in exception_constraints:
            if affordance.id in exception:
                return False
        return True

    def get_eef_poses(self, robot_group, candidate_poses, target_object, pre_eef_poses, waypoint_eef_poses, goal_eef_poses):
        '''
        calculate eef poses from approaching angles and target object
        '''
        target_size = target_object.bb3d.size
        target_pos = target_object.bb3d.center.position
        target_rot = target_object.bb3d.center.orientation

        # pre_eef_poses = []
        # waypoint_eef_poses = []
        # goal_eef_poses = []

        if robot_group is PlannerInputs.LEFT_ARM or robot_group is PlannerInputs.LEFT_ARM_WITHOUT_WAIST:
            offset = self.gripper_pose['left_offset']
        elif robot_group is PlannerInputs.RIGHT_ARM or robot_group is PlannerInputs.RIGHT_ARM_WITHOUT_WAIST:
            offset = self.gripper_pose['right_offset']

        # gripper end-effector position from target object
        for desired_pose in candidate_poses:

            # pre-grasp
            trans_pose = self._utils.create_pose([-(offset[1] + target_size.x/2.0), 0, 0],
                                            [0, 0, 0, 1])                                            
            pre_grasp_pose = self._utils.get_grasp_pose(self._utils.transform_pose(trans_pose, desired_pose), robot_group)
            
            # waypoint
            trans_pose = self._utils.create_pose([-((offset[0]+offset[1])/2.0 + target_size.x/2.0), 0, 0],
                                            [0, 0, 0, 1])   
            waypoint_pose = self._utils.get_grasp_pose(self._utils.transform_pose(trans_pose, desired_pose), robot_group)
            
            # grasp
            if self._hardware_if == 'vrep':
                trans_pose = self._utils.create_pose([-(offset[0] + target_size.x/2.0), 0, 0],
                                                 [0, 0, 0, 1])   
            elif self._hardware_if == 'hw':
                trans_pose = self._utils.create_pose([0, 0, 0],
                                                 [0, 0, 0, 1])   
            grasp_pose = self._utils.get_grasp_pose(self._utils.transform_pose(trans_pose, desired_pose), robot_group)

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

    def create_sample_poses(self, robot_group, target_object):
        '''
        create sample grasp pose around target object
        '''
        candidate_poses = []
        target_pose = Pose()
        target_pose.orientation = Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)
        target_pose.position = target_object.bb3d.center.position

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
