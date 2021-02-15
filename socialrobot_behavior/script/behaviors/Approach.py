import abc
from abc import ABCMeta
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
from geometry_msgs.msg import Pose

import tf
import tf.transformations as tfm
import numpy as np
import random
from behavior import BehaviorBase


def degToRad(deg):
    rad = deg / 180.0 * math.pi
    return rad


class ApproachBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(ApproachBehavior, self).__init__(name, **params)
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

    def check_requirement(self):
        rospy.loginfo('checking...%s' % self._name)
        self.service_list = rosservice.get_service_list()

        if self.service_name in self.service_list:
            return True

        return False

    def prepare_behavior(self):
        rospy.loginfo('preparing...%s' % self._name)
        return True

    def run_behavior(self):
        rospy.loginfo('running...%s' % self._name)
        if self._hardware_if == 'vrep':
            move_srv = rospy.ServiceProxy(self.service_name, self.service_type)
            move_req = VrepSetJointTrajectoryRequest()
            move_req.trajectory = self.behavior_data.get('trajectory')
            move_req.duration = self.behavior_data.get('duration')

            res = move_srv(move_req)

            if res.result == VrepSetJointTrajectoryResponse.OK:
                return 1

        elif self._hardware_if == 'hw':
            move_srv = rospy.ServiceProxy(self.service_name, self.service_type)
            move_req = SetJointTrajectoryRequest()
            move_req.trajectory = self.behavior_data.get('trajectory')
            move_req.duration = self.behavior_data.get('duration')

            res = move_srv(move_req)

            if res.result == SetJointTrajectoryResponse.OK:
                return 1

        return -1

    def get_motion(self, inputs):
        '''
        return the trajectory 
        '''
        rospy.loginfo("Calculating approaching motion..")
        res = MotionPlanResponse()
        ret, srv_response = self._call_ros_service(inputs)
        if ret == MotionPlanResponse.SUCCESS:
            rospy.loginfo("Approach planning is done.")
            self.motion_trajectory = srv_response.jointTrajectory
            res.planResult = MotionPlanResponse.SUCCESS
            res.jointTrajectory = srv_response.jointTrajectory
            return res
        else:
            rospy.loginfo("Approach planning is failed.")
            res.planResult = MotionPlanResponse.ERROR_NO_SOLUTION
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

    def _call_ros_service(self, inputs):
        try:
            plan_req = MotionPlanRequest()

            # target body
            body_type = inputs.targetBody
            # if body_type != MotionPlanRequest.LEFT_ARM and body_type != MotionPlanRequest.RIGHT_ARM:
            #     return (MotionPlanResponse.ERROR_INPUT, None)
            plan_req.targetBody = body_type

            # get obstacles
            plan_req.obstacle_ids = inputs.obstacle_ids
            plan_req.obstacles = inputs.obstacles

            # get bounding box of target object
            if len(inputs.targetObject) == 0:
                rospy.logerr("Approaching target is not decided.")
                return (MotionPlanResponse.ERROR_FAIL, None)
            target_object = inputs.targetObject

            try:
                idx = inputs.obstacle_ids.index(target_object[0])
            except:
                rospy.logerr("Cannot find target object in the obstacle list for approaching.")
                return (MotionPlanResponse.ERROR_FAIL, None)
            boundingbox = inputs.obstacles[idx]

            # # start state
            # start_state = inputs.currentJointState
            # if start_state is None:
            #     return (MotionPlanResponse.ERROR_INPUT, None)
            # plan_req.currentJointState = start_state

            # approach direction
            approach_direction = inputs.approachDirection
            self.approach_option = inputs.approachPoint

            # target object
            obj_center = boundingbox.center
            obj_size = boundingbox.size

            # build sample poses
            sample_top = sample_side = sample_front = sample_back = []
            """sample_angle
            180    135
                ^     /
                |  /
                |------> 90
            (left hand)
            """
            sample_angle = []
            if inputs.approachSamples:
                sample_angle = [sample for sample in inputs.approachSamples]
            else:
                if approach_direction == PlannerInputs.APPROACH_ANY or approach_direction == PlannerInputs.APPROACH_SIDE:  # both or side
                    sample_angle = [90, 105, 120, 135, 150, 165, 180]  # [180, 165, 150, 135, 120, 105, 90]
                if body_type == MotionPlanRequest.RIGHT_ARM:
                    sample_angle = [270, 255, 240, 225, 210, 195, 180]  #[180, 195, 210, 225, 240, 255, 270]

            for idx, i in enumerate(sample_angle):
                # first motion
                first_wrist_pose, second_wrist_pose, first_eef_pose, second_eef_pose = self.getApproachPose(
                    body_type, i, boundingbox)

                # publish sample pose
                self.talker.sendTransform(
                    (second_eef_pose.position.x, second_eef_pose.position.y, second_eef_pose.position.z),
                    (second_eef_pose.orientation.x, second_eef_pose.orientation.y, second_eef_pose.orientation.z,
                     second_eef_pose.orientation.w), rospy.Time.now(), "approach_sample_pose", "base_footprint")

                if first_wrist_pose:
                    # target pose
                    plan_req.targetPose = first_wrist_pose
                    plan_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL

                    first_plan = self.srv_plan(plan_req)

                    if first_plan.planResult == MotionPlanResponse.SUCCESS:
                        rospy.loginfo('(%d) %d deg approach pose to target nearby is found!', idx, i)

                        # get last joint state for waypoint
                        waypoint_pos = copy.copy(first_plan.jointTrajectory.points[-1].positions)
                        time_from_start = copy.copy(first_plan.jointTrajectory.points[-1].time_from_start)

                        joint_state = sensor_msgs.msg.JointState()
                        joint_state.header = first_plan.jointTrajectory.header
                        joint_state.name = first_plan.jointTrajectory.joint_names
                        joint_state.position = first_plan.jointTrajectory.points[-1].positions

                        # second motion
                        second_req = MotionPlanRequest()
                        second_req.obstacle_ids = inputs.obstacle_ids
                        second_req.obstacles = inputs.obstacles
                        second_req.currentJointState = joint_state
                        second_req.targetPose = second_wrist_pose
                        #second_req.goalType = MotionPlanRequest.CARTESIAN_WITH_ORIENTATION_CONSTRAINTS
                        second_plan = self.srv_plan(second_req)

                        if second_plan.planResult == MotionPlanResponse.SUCCESS:
                            rospy.loginfo('final approach pose to target is found!')
                            final_plan = MotionPlanResponse()
                            for i in first_plan.jointTrajectory.points:
                                final_plan.jointTrajectory.points.append(i)
                            final_plan.jointTrajectory.joint_names = second_plan.jointTrajectory.joint_names
                            final_plan.jointTrajectory.header = second_plan.jointTrajectory.header
                            for i in second_plan.jointTrajectory.points:
                                i.time_from_start += (time_from_start + rospy.Time(0.2))
                                final_plan.jointTrajectory.points.append(i)

                            return (MotionPlanResponse.SUCCESS, final_plan)
                        else:
                            rospy.loginfo('no solution of final approach pose to target')

            return (MotionPlanResponse.ERROR_NO_SOLUTION, None)

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            return (MotionPlanResponse.ERROR_FAIL, None)

    def getApproachPose(self, body_type, azimuth_angle, target_bb):
        """
        [get gripper's approaching pose]
        """
        #samples_wrist = []
        #samples_eef = []

        target_size = target_bb.size
        target_pos = target_bb.center.position
        target_rot = target_bb.center.orientation

        # get transforms between wrist and end-effector
        base_to_eef_trans = base_to_eef_rot = []
        eef_to_wrist_trans = eef_to_wrist_rot = []
        is_trans = False

        while is_trans == False:
            try:
                if body_type is PlannerInputs.LEFT_ARM or body_type is PlannerInputs.LEFT_ARM_WITHOUT_WAIST:
                    base_to_eef_trans, base_to_eef_rot = self.listener.lookupTransform(
                        '/base_footprint', '/left_end_effect_point', rospy.Time(0))
                    eef_to_wrist_trans, eef_to_wrist_rot = self.listener.lookupTransform(
                        '/left_end_effect_point', self.gripper_pose['left'], rospy.Time(0))
                    offset = self.gripper_pose['left_offset']

                elif body_type is PlannerInputs.RIGHT_ARM or body_type is PlannerInputs.RIGHT_ARM_WITHOUT_WAIST:
                    base_to_eef_trans, base_to_eef_rot = self.listener.lookupTransform(
                        '/base_footprint', '/right_end_effect_point', rospy.Time(0))
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
        first_wrist_pose = Pose()
        second_wrist_pose = Pose()

        i = azimuth_angle
        # gripper end-effector position from target object
        if self.approach_option == PlannerInputs.APPROACH_TOP:
            z_offset = target_size.z / 4
        elif self.approach_option == PlannerInputs.APPROACH_BOTTOM:
            z_offset = -(target_size.z / 4)
        else:
            z_offset = 0

        second_eef_pose.position.x = target_pos.x + (target_size.x / 2 + offset[0]) * math.cos(degToRad(i))
        second_eef_pose.position.y = target_pos.y + (target_size.x / 2 + offset[0]) * math.sin(degToRad(i))
        second_eef_pose.position.z = target_pos.z + z_offset

        first_eef_pose.position.x = target_pos.x + (target_size.x / 2 + offset[1]) * math.cos(degToRad(i))
        first_eef_pose.position.y = target_pos.y + (target_size.x / 2 + offset[1]) * math.sin(degToRad(i))
        first_eef_pose.position.z = target_pos.z + z_offset

        #
        init_quat = [0, -0.707, 0, 0.707]
        quat = tfm.quaternion_multiply(init_quat, tfm.quaternion_about_axis(degToRad(i), (1, 0, 0)))

        first_eef_pose.orientation.x = second_eef_pose.orientation.x = quat[0]
        first_eef_pose.orientation.y = second_eef_pose.orientation.y = quat[1]
        first_eef_pose.orientation.z = second_eef_pose.orientation.z = quat[2]
        first_eef_pose.orientation.w = second_eef_pose.orientation.w = quat[3]

        if body_type == MotionPlanRequest.RIGHT_ARM:
            quat = tfm.quaternion_multiply(np.array([quat[0], quat[1], quat[2], quat[3]]), self.rot_z(180))

            first_eef_pose.orientation.x = second_eef_pose.orientation.x = quat[0]
            first_eef_pose.orientation.y = second_eef_pose.orientation.y = quat[1]
            first_eef_pose.orientation.z = second_eef_pose.orientation.z = quat[2]
            first_eef_pose.orientation.w = second_eef_pose.orientation.w = quat[3]

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

        first_wrist_pose.position.x = trans3[0]
        first_wrist_pose.position.y = trans3[1]
        first_wrist_pose.position.z = trans3[2]

        first_wrist_pose.orientation.x = rot3[0]
        first_wrist_pose.orientation.y = rot3[1]
        first_wrist_pose.orientation.z = rot3[2]
        first_wrist_pose.orientation.w = rot3[3]

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

        second_wrist_pose.position.x = trans3[0]
        second_wrist_pose.position.y = trans3[1]
        second_wrist_pose.position.z = trans3[2]

        second_wrist_pose.orientation.x = rot3[0]
        second_wrist_pose.orientation.y = rot3[1]
        second_wrist_pose.orientation.z = rot3[2]
        second_wrist_pose.orientation.w = rot3[3]

        return first_wrist_pose, second_wrist_pose, first_eef_pose, second_eef_pose
