import abc
from abc import ABCMeta
from numpy.core.numeric import require
from six import with_metaclass

import rospy
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import Pose
from socialrobot_motion.srv import *
from socialrobot_behavior.msg import *
from socialrobot_behavior.srv import *
from socialrobot_hardware.srv import *
from social_robot_dual_arm_planner.srv import *

import copy
import tf
import tf.transformations as tfm
from tf.transformations import quaternion_from_euler, quaternion_matrix
import numpy as np
import math
from behavior import BehaviorBase

def degToRad(deg):
    rad = deg / 180.0 * math.pi
    return rad

def pose_2_mat(trans, rot):
    trans_vec = [trans['x'], trans['y'], trans['z']]
    rot_vec = [rot['x'], rot['y'], rot['z'], rot['w']]

    trans_mat = tf.transformations.translation_matrix(trans_vec)
    rot_mat = tf.transformations.quaternion_matrix(rot_vec)
    T = np.dot(trans_mat, rot_mat) 
    return T

def mat_2_pose(T):
    trans = tf.transformations.translation_from_matrix(T) 
    rot = tf.transformations.quaternion_from_matrix(T)
    
    pose_trans = {'x': trans[0], 'y': trans[1], 'z': trans[2]}
    pose_rot = {'x': rot[0], 'y': rot[1], 'z': rot[2], 'w': rot[3]}
    return pose_trans, pose_rot

class ApproachDualArmBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(ApproachDualArmBehavior, self).__init__(name, **params)
        self.listener = tf.TransformListener()

        self.talker = tf.TransformBroadcaster(queue_size=10)
        self.grasp_srv = None
        #self.grasp_srv = rospy.ServiceProxy("/dual_arm_planning/grasp", DualArmGrasp)   # box method
        self.grasp_srv = rospy.ServiceProxy("/dual_arm_planning/grasp_seperated_targets", DualArmGraspSeperatedTargets)   # tray method
        self.srv_plan = rospy.ServiceProxy("/motion_plan/move_arm", MotionPlan)     # SKKU method

        robot_name = 'social_robot'

        # for SKKU method
        #get hardware
        if rospy.has_param("/robot_name"):
            robot_name = rospy.get_param("/robot_name")

        if robot_name == 'skkurobot':
            self.gripper_pose = {
                'left': '7dof_RISE_wrist_link',
                'right': '6dof_connection_link',
                'left_offset': [-0.02, 0.08],
                'right_offset': [-0.025, 0.08]
            }            
        else:
            self.gripper_pose = {
                'left': 'LHand_base',
                'right': 'RHand_base',
                'left_offset': [0.002, 0.04],
                'right_offset': [0.002, 0.04]
            }
            self.waist_pose = [0, -5, -10, -15]

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
        res = self._call_ros_service_simultaneous(inputs)
        return res

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True

    # calculate dual arm motion simultaneously
    def _call_ros_service_simultaneous(self, requirements):
        try:
            plan_res = MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

            # target body
            robot_group = requirements.robot_group[0]    
            if robot_group != MotionPlanRequest.BOTH_ARM:
                return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_INPUT)

            # request add object into scene
            obstacles = requirements.static_object + requirements.dynamic_object

            # get bounding box of target object
            if len(requirements.target_object) == 0:
                rospy.logerr("Approaching target is not decided.")
                return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_INPUT)
            else:
                target_object = requirements.target_object[0]
                target_data = self._utils.get_object_info(target_object.id)
                if target_data != None:
                    target_object.grasp_point = target_data.grasp_point
                    # transpose them based on robot frame          
                    target_object = self._utils.transform_object(target_object)
                print('grasping target:')
                print(target_object)

            trans_pose = self._utils.create_pose([0, 0, 0],
                                                [0, 0, 0, 1]) 
            left_eef_pose = self._utils.get_grasp_pose(self._utils.transform_pose(trans_pose, target_object.grasp_point[0]), PlannerInputs.LEFT_ARM)
            right_eef_pose = self._utils.get_grasp_pose(self._utils.transform_pose(trans_pose, target_object.grasp_point[1]), PlannerInputs.RIGHT_ARM)

            # transform based on wrist coordinate from eef
            left_wrist_pose = self.eef_to_wrist(left_eef_pose, PlannerInputs.LEFT_ARM)
            right_wrist_pose = self.eef_to_wrist(right_eef_pose, PlannerInputs.RIGHT_ARM)

            plan_req = DualArmGraspSeperatedTargetsRequest()
            plan_req.left_target = left_wrist_pose
            plan_req.right_target = right_wrist_pose

            # add obstacles
            rospy.loginfo("adding obstacles into scene...")
            obs_req = MotionPlanRequest()
            obstacles = requirements.static_object + requirements.dynamic_object
            has_workspace = False
            for obs in obstacles:
                if obs.id in ['obj_table','obj_fridge']:
                    has_workspace = True
                obs_req.obstacle_ids.append(obs.id)
                obs_req.obstacles.append(obs.bb3d)

            # if no workspace in obstacles, create virtual workspace
            if not has_workspace:
                table = self._utils.create_workspace(target_object.bb3d)
                obs_req.obstacle_ids.append(table.id)
                obs_req.obstacles.append(table.bb3d)

            # add obstacles first
            obs_req.targetBody = MotionPlanRequest.BOTH_ARM
            obs_req.goalType = -1
            self.srv_plan(obs_req)

            # calculate pre-grasp motion
            plan = DualArmGraspSeperatedTargetsResponse()
            iter = 0
            while(iter < 5):
                rospy.logwarn("[ApproachDualArm Behavior] attempting to grasp motion....%d/%d" %(iter+1,5))
                plan = self.grasp_srv(plan_req)    
                if plan.pre_grasp_trajetory.points and plan.grasp_trajetory.points:
                    break
                iter+=1       

            if plan.pre_grasp_trajetory.points and plan.grasp_trajetory.points:
                rospy.loginfo("[ApproachDualArm Behavior] planning is done.")
                # fix zero velocities
                for i, pt in enumerate(plan.grasp_trajetory.points):
                    vel_list = []
                    for vel in pt.velocities:
                        if vel==0.0:
                            fixed_vel = 0.2
                            vel_list.append(fixed_vel)
                        else:
                            vel_list.append(vel)
                    plan.grasp_trajetory.points[i].velocities = vel_list

                #### Robocare hardware ERROR?? ###
                # merge pre-grasp and grasp trajectory
                merged_traj = self._utils.connect_trajectories(plan.pre_grasp_trajetory, plan.grasp_trajetory, rospy.Duration(0.0), only_last=True)

                # #### calculate again for final grasp
                # # get start joint
                # waypoint_pos = copy.copy(plan.pre_grasp_trajetory.points[-1].positions)
                # time_from_start = copy.copy(plan.pre_grasp_trajetory.points[-1].time_from_start)
                # start_state = sensor_msgs.msg.JointState()
                # start_state.header = plan.pre_grasp_trajetory.header
                # start_state.name = plan.pre_grasp_trajetory.joint_names
                # start_state.position = plan.pre_grasp_trajetory.points[-1].positions

                # goal_state = sensor_msgs.msg.JointState()
                # goal_state.header = plan.grasp_trajetory.header
                # goal_state.name = plan.grasp_trajetory.joint_names
                # goal_state.position = plan.grasp_trajetory.points[-1].positions

                # # get goal joint
                # second_req = MotionPlanRequest()
                # second_req.targetBody = robot_group
                # # second_req.obstacle_ids = obs_req.obstacle_ids
                # # second_req.obstacles = obs_req.obstacles
                # second_req.currentJointState = start_state
                # second_req.targetJointState = [goal_state]
                # second_req.goalType = MotionPlanRequest.JOINT_SPACE_GOAL
                # grasp_plan = self.srv_plan(second_req)

                # print('========================pre===========================', len(plan.pre_grasp_trajetory.points))
                # print(plan.pre_grasp_trajetory)
                # print('========================grasp===========================', len(plan.grasp_trajetory.points))
                # print(plan.grasp_trajetory)
                # print('========================merge===========================')
                # print(merged_traj)
                # print('========================second===========================')
                # print(grasp_plan)

                plan_res.planResult = MotionPlanResponse.SUCCESS
                plan_res.jointTrajectory = merged_traj
                return plan_res
            else:
                rospy.logerr('[ApproachDualArm Behavior] no solution.')
                plan_res.planResult = MotionPlanResponse.ERROR_NO_SOLUTION
                return plan_res
      
        # No solution
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            plan_res.planResult = MotionPlanResponse.ERROR_FAIL
            return plan_res

    def get_sample_degree(self, step, max_degree):
        sample_list = [0.0]
        for i in range(1, max_degree + 1, step):
            sample_list.append(i)
            sample_list.append(-i)

        return sample_list

    # calculate dual arm motion indivisually
    def _call_ros_service_indivisual(self, requirements):
        try:
            plan_req = MotionPlanRequest()
            plan_res = MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

            self.approach_option = PlannerInputs.APPROACH_SIDE

            # target body
            robot_group = requirements.robot_group[0]    
            if robot_group != MotionPlanRequest.BOTH_ARM:
                return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_INPUT)
            else:
                plan_req.targetBody = robot_group

            # set obstacles
            obstacles = requirements.static_object + requirements.dynamic_object
            for obs in obstacles:
                plan_req.obstacle_ids.append(obs.id)
                plan_req.obstacles.append(obs.bb3d)

            # get bounding box of target object
            if len(requirements.target_object) == 0:
                rospy.logerr("Approaching target is not decided.")
                return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_INPUT)
            else:
                target_object = requirements.target_object[0]

            # build sample poses for both arm
            left_approach_angles = [60, 75, 90, 105, 120]  # [180, 165, 150, 135, 120, 105, 90]
            right_approach_angles = [300, 285, 270, 255, 240]  #[180, 195, 210, 225, 240, 255, 270]
            waist_pitch_angles = self.waist_pose

            # if robot has torso, get torso pitch motion first
            if len(waist_pitch_angles)>0:
                waist_req = copy.copy(plan_req)
                for waist_deg in waist_pitch_angles:
                    print("[ApproachDualArm] Waist deg = %d"  %waist_deg)
                    waist_rad = degToRad(waist_deg)

                    # set waist angle
                    waist_req.goalType = MotionPlanRequest.JOINT_SPACE_GOAL
                    joint_state = JointState()
                    joint_state.name = [
                        'Waist_Roll', 'Waist_Pitch', 
                        'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll',
                        'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll'
                    ]
                    joint_state.position = [0, waist_rad, 
                                            0, -1.309, 0, 0, 0, 0,
                                            0, 1.309, 0, 0, 0, 0]  #init_pose
                    waist_req.targetJointState = [joint_state]

                    # request to motion planner
                    waist_plan = self.srv_plan(waist_req)

                    if waist_plan.planResult == MotionPlanResponse.SUCCESS:
                        # get last waist pose to set start pose
                        waist_state = sensor_msgs.msg.JointState()
                        waist_state.header = waist_plan.jointTrajectory.header
                        waist_state.name = waist_plan.jointTrajectory.joint_names
                        waist_state.position = list(waist_plan.jointTrajectory.points[-1].positions)

                        # right side
                        if target_object.bb3d.center.position.y < 0:
                            right_req = copy.copy(waist_req)
                            right_req.targetBody = MotionPlanRequest.RIGHT_ARM
                            right_req.currentJointState = waist_state

                            # get right arm approach pose
                            right_traj = self.getApproachMotion("right", right_req, right_approach_angles, target_object)
                            if right_traj == None:
                                continue                            
                            else:
                                # get left arm approach pose
                                left_req = copy.copy(waist_req)
                                left_req.targetBody = MotionPlanRequest.LEFT_ARM_WITHOUT_WAIST
                                left_req.currentJointState = waist_state
                                right_joint_position = right_traj.points[-1].positions

                                for i, joint_name in enumerate(right_traj.joint_names):
                                    idx = left_req.currentJointState.name.index(joint_name)                
                                    left_req.currentJointState.position[idx] = right_joint_position[i]

                                left_traj = self.getApproachMotion("left", left_req, left_approach_angles, target_object)
                                if left_traj == None:
                                    continue
                                else:
                                    # merge joint trajectory
                                    return self.mergePlan(waist_plan, right_traj, left_traj)

                        # left side
                        else:
                            left_req = copy.copy(waist_req)
                            left_req.targetBody = MotionPlanRequest.LEFT_ARM
                            left_req.currentJointState = waist_state

                            # get left arm approach pose
                            left_traj = self.getApproachMotion("left", left_req, left_approach_angles, target_object)
                            if left_traj == None:
                                continue                            
                            else:
                                # get right arm approach pose    
                                right_req = copy.copy(waist_req)
                                right_req.targetBody = MotionPlanRequest.RIGHT_ARM_WITHOUT_WAIST
                                right_req.currentJointState = waist_state     
                                left_joint_position = left_traj.points[-1].positions

                                for i, joint_name in enumerate(left_traj.joint_names):
                                    idx = right_req.currentJointState.name.index(joint_name)                
                                    right_req.currentJointState.position[idx] = left_joint_position[i]

                                right_traj = self.getApproachMotion("right", right_req, right_approach_angles, target_object)
                                if right_traj == None:
                                    continue
                                else:
                                    # merge joint trajectory
                                    return self.mergePlan(waist_plan, left_traj, right_traj)

                # no dual solution                
                return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_NO_SOLUTION)
            # if robot has no torso,
            else:
                pass        
        # No solution
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            plan_res.planResult = MotionPlanResponse.ERROR_FAIL
            return plan_res

    def mergePlan(self, waist_plan, first_traj, second_traj):
        final_plan = copy.copy(waist_plan)
        joint_names = final_plan.jointTrajectory.joint_names

        init_pos = final_plan.jointTrajectory.points[-1].positions
        init_vel = [0.0] * len(init_pos)
        init_acc = [0.0] * len(init_pos)
        init_time = final_plan.jointTrajectory.points[-1].time_from_start

        # merge left trajectory
        for pt in first_traj.points:
            traj_pt = JointTrajectoryPoint()
            traj_pt.positions = list(copy.copy(init_pos))
            traj_pt.velocities = list(copy.copy(init_vel))
            traj_pt.accelerations = list(copy.copy(init_acc))
            traj_pt.time_from_start = init_time

            for i, joint_name in enumerate(first_traj.joint_names):
                idx = joint_names.index(joint_name)                
                traj_pt.positions[idx] = pt.positions[i]
            traj_pt.time_from_start = rospy.Time().from_sec(traj_pt.time_from_start.to_sec() + pt.time_from_start.to_sec())

            final_plan.jointTrajectory.points.append(traj_pt)

        # merge right trajectory
        init_pos = final_plan.jointTrajectory.points[-1].positions
        init_time = final_plan.jointTrajectory.points[-1].time_from_start

        for pt in second_traj.points:
            traj_pt = JointTrajectoryPoint()
            traj_pt.positions = list(copy.copy(init_pos))
            traj_pt.velocities = list(copy.copy(init_vel))
            traj_pt.accelerations = list(copy.copy(init_acc))
            traj_pt.time_from_start = init_time

            for i, joint_name in enumerate(second_traj.joint_names):
                idx = joint_names.index(joint_name)
                traj_pt.positions[idx] = pt.positions[i]
            traj_pt.time_from_start = rospy.Time().from_sec(traj_pt.time_from_start.to_sec() + pt.time_from_start.to_sec())

            final_plan.jointTrajectory.points.append(traj_pt)

        final_plan.planResult = MotionPlanResponse.SUCCESS
        return final_plan

    def getApproachMotion(self, robot_group, request, sample_angle, target_object):
        for idx, i in enumerate(sample_angle):
            print("[ApproachDualArm] %s approach deg = %d" %(robot_group, idx))
            # first motion
            first_wrist_pose, waypoints, eef_pose = self.getApproachPose(robot_group, i, target_object)

            # publish sample pose
            self.talker.sendTransform(
                (eef_pose.position.x, eef_pose.position.y, eef_pose.position.z),
                (eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z,
                    eef_pose.orientation.w), rospy.Time.now(), "approach_sample_pose", "base_footprint")

            if first_wrist_pose:
                # target pose
                request.targetPose = [first_wrist_pose]
                request.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL

                first_plan = self.srv_plan(request)

                if first_plan.planResult == MotionPlanResponse.SUCCESS:
                    rospy.loginfo('%s (%d) %d deg approach pose to target nearby is found!', robot_group, idx, i)

                    # get last joint state for waypoint
                    waypoint_pos = copy.copy(first_plan.jointTrajectory.points[-1].positions)
                    time_from_start = copy.copy(first_plan.jointTrajectory.points[-1].time_from_start)
                    
                    joint_state = sensor_msgs.msg.JointState()
                    joint_state.header = first_plan.jointTrajectory.header
                    joint_state.name = first_plan.jointTrajectory.joint_names
                    joint_state.position = first_plan.jointTrajectory.points[-1].positions

                    # second motion
                    second_req = MotionPlanRequest()
                    second_req.obstacle_ids = request.obstacle_ids
                    second_req.obstacles = request.obstacles
                    second_req.currentJointState = joint_state
                    second_req.targetPose = waypoints
                    
                    second_plan = self.srv_plan(second_req)

                    if second_plan.planResult == MotionPlanResponse.SUCCESS:
                        rospy.loginfo('final %s approach pose to target is found!', robot_group)
                        final_plan = MotionPlanResponse()
                        for i in first_plan.jointTrajectory.points:
                            final_plan.jointTrajectory.points.append(i)
                        final_plan.jointTrajectory.joint_names = second_plan.jointTrajectory.joint_names
                        final_plan.jointTrajectory.header = second_plan.jointTrajectory.header
                        for i in second_plan.jointTrajectory.points:
                            i.time_from_start += (time_from_start + rospy.Time(0.2))
                            final_plan.jointTrajectory.points.append(i)
                            
                        return final_plan.jointTrajectory
                    else:
                        rospy.loginfo('no solution of final %s approach pose to target', robot_group)
        return None

    def getApproachPose(self, robot_group, azimuth_angle, target_object):
        """
        [get gripper's approaching pose]
        """

        target_size = target_object.bb3d.size
        target_pos = target_object.bb3d.center.position
        target_rot = target_object.bb3d.center.orientation

        # get transforms between wrist and end-effector
        eef_to_wrist_trans = eef_to_wrist_rot = []
        is_trans = False

        while is_trans == False:
            try:
                if robot_group == "left":
                    eef_to_wrist_trans, eef_to_wrist_rot = self.listener.lookupTransform(
                        '/left_end_effect_point', self.gripper_pose['left'], rospy.Time(0))
                    offset = self.gripper_pose['left_offset']

                elif robot_group == "right":
                    eef_to_wrist_trans, eef_to_wrist_rot = self.listener.lookupTransform(
                        '/right_end_effect_point', self.gripper_pose['right'], rospy.Time(0))
                    offset = self.gripper_pose['right_offset']
                is_trans = True
            except:
                pass
        is_trans = False

        # for i in azimuth_angle:
        first_eef_pose = Pose()
        second_eef_pose = Pose()
        final_eef_pose = Pose()

        # gripper end-effector position from target object
        i = azimuth_angle
        if self.approach_option == PlannerInputs.APPROACH_TOP:
            z_offset = target_size.z / 4
        elif self.approach_option == PlannerInputs.APPROACH_BOTTOM:
            z_offset = -(target_size.z / 4)
        else:
            z_offset = 0

        final_eef_pose.position.x = target_pos.x + (target_size.x / 2 + offset[0]) * math.cos(degToRad(i))
        final_eef_pose.position.y = target_pos.y + (target_size.x / 2 + offset[0]) * math.sin(degToRad(i))
        final_eef_pose.position.z = target_pos.z + z_offset

        second_eef_pose.position.x = target_pos.x + (target_size.x / 2 + (offset[0]+offset[1])/2) * math.cos(degToRad(i))
        second_eef_pose.position.y = target_pos.y + (target_size.x / 2 + (offset[0]+offset[1])/2) * math.sin(degToRad(i))
        second_eef_pose.position.z = target_pos.z + z_offset

        first_eef_pose.position.x = target_pos.x + (target_size.x / 2 + offset[1]) * math.cos(degToRad(i))
        first_eef_pose.position.y = target_pos.y + (target_size.x / 2 + offset[1]) * math.sin(degToRad(i))
        first_eef_pose.position.z = target_pos.z + z_offset

        #
        init_quat = [0, -0.707, 0, 0.707]
        quat = tfm.quaternion_multiply(init_quat, tfm.quaternion_about_axis(degToRad(i), (1, 0, 0)))

        first_eef_pose.orientation.x = quat[0]
        first_eef_pose.orientation.y = quat[1]
        first_eef_pose.orientation.z = quat[2]
        first_eef_pose.orientation.w = quat[3]

        if robot_group == MotionPlanRequest.RIGHT_ARM:
            quat = tfm.quaternion_multiply(np.array([quat[0], quat[1], quat[2], quat[3]]), self.rot_z(180))

            first_eef_pose.orientation.x = quat[0]
            first_eef_pose.orientation.y = quat[1]
            first_eef_pose.orientation.z = quat[2]
            first_eef_pose.orientation.w = quat[3]

        second_eef_pose.orientation = copy.deepcopy(first_eef_pose.orientation)
        final_eef_pose.orientation = copy.deepcopy(first_eef_pose.orientation)

        # transform to wrist from eef
        trans1_mat = tf.transformations.translation_matrix(
            [first_eef_pose.position.x, first_eef_pose.position.y, first_eef_pose.position.z])
        rot1_mat = tf.transformations.quaternion_matrix([
            first_eef_pose.orientation.x, first_eef_pose.orientation.y, first_eef_pose.orientation.z,
            first_eef_pose.orientation.w
        ])
        mat1 = np.dot(trans1_mat, rot1_mat)

        trans2_mat = tf.transformations.translation_matrix(eef_to_wrist_trans)
        rot2_mat = tf.transformations.quaternion_matrix(eef_to_wrist_rot)
        mat2 = np.dot(trans2_mat, rot2_mat)

        mat3 = np.dot(mat1, mat2)
        trans3 = tf.transformations.translation_from_matrix(mat3)
        rot3 = tf.transformations.quaternion_from_matrix(mat3)

        first_wrist_pose = Pose()
        first_wrist_pose.position.x = trans3[0]
        first_wrist_pose.position.y = trans3[1]
        first_wrist_pose.position.z = trans3[2]

        first_wrist_pose.orientation.x = rot3[0]
        first_wrist_pose.orientation.y = rot3[1]
        first_wrist_pose.orientation.z = rot3[2]
        first_wrist_pose.orientation.w = rot3[3]

        waypoints = []
        #
        trans1_mat = tf.transformations.translation_matrix(
            [second_eef_pose.position.x, second_eef_pose.position.y, second_eef_pose.position.z])
        rot1_mat = tf.transformations.quaternion_matrix([
            second_eef_pose.orientation.x, second_eef_pose.orientation.y, second_eef_pose.orientation.z,
            second_eef_pose.orientation.w
        ])
        mat1 = np.dot(trans1_mat, rot1_mat)

        trans2_mat = tf.transformations.translation_matrix(eef_to_wrist_trans)
        rot2_mat = tf.transformations.quaternion_matrix(eef_to_wrist_rot)
        mat2 = np.dot(trans2_mat, rot2_mat)

        mat3 = np.dot(mat1, mat2)
        trans3 = tf.transformations.translation_from_matrix(mat3)
        rot3 = tf.transformations.quaternion_from_matrix(mat3)

        second_wrist_pose = Pose()
        second_wrist_pose.position.x = trans3[0]
        second_wrist_pose.position.y = trans3[1]
        second_wrist_pose.position.z = trans3[2]

        second_wrist_pose.orientation.x = rot3[0]
        second_wrist_pose.orientation.y = rot3[1]
        second_wrist_pose.orientation.z = rot3[2]
        second_wrist_pose.orientation.w = rot3[3]
        waypoints.append(second_wrist_pose)

        #
        trans1_mat = tf.transformations.translation_matrix(
            [final_eef_pose.position.x, final_eef_pose.position.y, final_eef_pose.position.z])
        rot1_mat = tf.transformations.quaternion_matrix([
            final_eef_pose.orientation.x, final_eef_pose.orientation.y, final_eef_pose.orientation.z,
            final_eef_pose.orientation.w
        ])
        mat1 = np.dot(trans1_mat, rot1_mat)

        trans2_mat = tf.transformations.translation_matrix(eef_to_wrist_trans)
        rot2_mat = tf.transformations.quaternion_matrix(eef_to_wrist_rot)
        mat2 = np.dot(trans2_mat, rot2_mat)

        mat3 = np.dot(mat1, mat2)
        trans3 = tf.transformations.translation_from_matrix(mat3)
        rot3 = tf.transformations.quaternion_from_matrix(mat3)

        final_wrist_pose = Pose()
        final_wrist_pose.position.x = trans3[0]
        final_wrist_pose.position.y = trans3[1]
        final_wrist_pose.position.z = trans3[2]

        final_wrist_pose.orientation.x = rot3[0]
        final_wrist_pose.orientation.y = rot3[1]
        final_wrist_pose.orientation.z = rot3[2]
        final_wrist_pose.orientation.w = rot3[3]
        waypoints.append(final_wrist_pose)

        return first_wrist_pose, waypoints, final_eef_pose

    def eef_to_wrist(self, eef_pose, robot_group):
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
        return self._utils.transform_pose(eef_to_wrist_pose, eef_pose)