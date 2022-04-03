import copy
import math
import numpy as np
import rospy
import rosservice
import rosparam
import tf
import tf.transformations as tfm
from behavior import BehaviorBase
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Quaternion
from socialrobot_hardware import srv as hardware_srv
from socialrobot_motion import srv as motion_srv
from socialrobot_behavior import srv as behavior_srv
from socialrobot_behavior import msg as behavior_msg


class PushBehavior(BehaviorBase):
    left_group = [
        'Waist_Roll', 'Waist_Pitch', 'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch',
        'LWrist_Roll'
    ]
    right_group = [
        'Waist_Roll', 'Waist_Pitch', 'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch',
        'RWrist_Roll'
    ]

    def __init__(self, name, **params):
        super(PushBehavior, self).__init__(name, **params)

        self.listener = tf.TransformListener()
        self.talker = tf.TransformBroadcaster(queue_size=10)
        self.motion_proxy = rospy.ServiceProxy('/motion_plan/move_arm', motion_srv.MotionPlan)

        if self._hardware_if == 'vrep':
            self.service_name = '/sim_interface/set_motion'
            self.service_type = hardware_srv.VrepSetJointTrajectory
        elif self._hardware_if == 'hw':
            self.service_name = '/hw_interface/set_motion'
            self.service_type = hardware_srv.SetJointTrajectory
        self.service_request = {
            'vrep': hardware_srv.VrepSetJointTrajectoryRequest,
            'hw': hardware_srv.SetJointTrajectoryRequest,
        }
        self.service_response = {
            'vrep': hardware_srv.VrepSetJointTrajectoryResponse,
            'hw': hardware_srv.SetJointTrajectoryResponse,
        }
        self.gripper_pose = {}
        self.rot_x = lambda x: tfm.quaternion_about_axis(math.radians(x), (1, 0, 0))
        self.rot_y = lambda x: tfm.quaternion_about_axis(math.radians(x), (0, 1, 0))
        self.rot_z = lambda x: tfm.quaternion_about_axis(math.radians(x), (0, 0, 1))

        robot_name = rosparam.get_param("/robot_name")
        if robot_name == 'skkurobot':
            pass
            #raise NotImplementedError
        elif robot_name == 'social_robot':
            self.gripper_pose = {
                'left': 'LHand_base',
                'right': 'RHand_base',
                'left_offset': [-0.0055, 0.03],
                'right_offset': [-0.0055, 0.03]
            }

    def check_requirement(self):
        '''
        Returns:
            return (bool): { True: self.READY_STATE, False: self.ERROR_STATE }
        '''
        rospy.loginfo('checking...%s' % self._name)
        return self.service_name in rosservice.get_service_list()

    def prepare_behavior(self):
        '''
        Returns:
            return (bool): { True: self.RUNNING_STATE, False: self.ERROR_STATE }
        '''
        rospy.loginfo('preparing...%s' % self._name)
        return True

    def run_behavior(self):
        '''Send a result trajectory to the simulation or the robot.

        Returns:
            state (int): {1: self.DONE_STATE,
                          0: self.RUNNING_STATE,
                         -1: self.ERROR_STATE}
        '''
        rospy.loginfo('running...%s' % self._name)
        move_srv = rospy.ServiceProxy(self.service_name, self.service_type)
        move_req = self.service_request.get(self._hardware_if)()
        move_req.trajectory = self.behavior_data.get('trajectory')
        move_req.duration = self.behavior_data.get('duration')
        res = move_srv(move_req)
        return 1 if res.result == self.service_response.get(self._hardware_if).OK else -1

    def get_motion(self, inputs):
        '''Calculate a motion plan.

        Args:
            inputs (socialrobot_behavior.msg.PlannerInputs)

        Returns:
            res (socialrobot_motion.srv.MotionPlanResponse): Return a trajectory.
        '''
        rospy.loginfo("Calculating push motion..")
        # motion_srv.MotionPlanResponse()
        res = self._call_ros_service(inputs)
        if res.planResult == motion_srv.MotionPlanResponse.SUCCESS:
            rospy.loginfo("Push planning is done..")
        else:
            rospy.logerr("Push planning is failed..")
        return res

    def finish_behavior(self):
        '''
        Returns:
            return (bool): { True: self.DONE_STATE, False: self.ERROR_STATE }
        '''
        rospy.loginfo('finishing...%s' % self._name)
        return True

    def _call_ros_service(self, inputs):
        '''
        Args:
            inputs (socialrobot_behavior.msg.PlannerInputs)
                - targetBody
                - obstacle_ids
                - obstacles
                - targetObject

        Returns:
            res (socialrobot_motion.srv.MotionPlanResponse): Return a trajectory.
        '''
        motion_resp = motion_srv.MotionPlanResponse()

        body_type = inputs.targetBody

        errors = []
        if body_type not in [motion_srv.MotionPlanRequest.LEFT_ARM, motion_srv.MotionPlanRequest.RIGHT_ARM]:
            errors.append("inputs.targetBody not in expected types")
        if len(inputs.targetObject) == 0:
            errors.append("len(inputs.targetObject) == 0. Approaching target is not decided.")
        if errors:
            for msg in errors:
                rospy.logerr("[%s] %s" % (self._name, msg))
            motion_resp.planResult = motion_srv.MotionPlanResponse.ERROR_INPUT
            return motion_resp

        try:
            target_name = inputs.targetObject[0]  # string
            idx = inputs.obstacle_ids.index(target_name)
        except ValueError:
            rospy.logerr("Cannot find target object in the obstacle list for approaching.")
            motion_resp.planResult = motion_srv.MotionPlanResponse.ERROR_FAIL
            return motion_resp

        target_box = inputs.obstacles[idx]

        # build sample poses
        # sample_angles = [90, 105, 120, 135, 150, 165, 180]
        # in_hand = True, False
        """sample_angles
           180    135
            ^     /
            |  /
            |------> 90
        (left hand)
        """
        sample_angles = [180, 165, 150, 135, 120, 105, 90]
        for idx, azimuth in enumerate(sample_angles):
            try:
                motion_resp = self._try_approach(
                    idx,
                    body_type,
                    azimuth,
                    target_box,
                    inputs.obstacle_ids,
                    inputs.obstacles,
                )
                if motion_resp.planResult == motion_srv.MotionPlanResponse.SUCCESS:
                    break
            except rospy.ServiceException as e:
                rospy.logerr('Service call failed: %s' % e)
                motion_resp.planResult = motion_srv.MotionPlanResponse.ERROR_FAIL
                break
        return motion_resp

    def _try_approach(self, iteration, body_type, azimuth, bounding_box, obstacle_ids, obstacles):
        """
        Return: motion_srv.MotionPlanResponse()
        """
        final_plan = motion_srv.MotionPlanResponse()

        candidates = self._get_approach_pose(body_type, azimuth, bounding_box)
        if None in candidates:
            raise NotImplementedError
        first_wrist_pose = candidates[0]
        second_wrist_pose = candidates[1]
        # first_ee_pose = candidates[2]
        second_ee_pose = candidates[3]

        # publish sample pose
        _ep = second_ee_pose.position
        _eo = second_ee_pose.orientation
        self.talker.sendTransform((_ep.x, _ep.y, _ep.z), (_eo.x, _eo.y, _eo.z, _eo.w), rospy.Time.now(),
                                  "approach_sample_pose", "base_footprint")

        # first motion
        first_req = motion_srv.MotionPlanRequest()
        first_req.targetBody = body_type
        first_req.obstacle_ids = obstacle_ids  # a list of string
        first_req.obstacles = obstacles  # a list of BoundingBox3D

        first_req.targetPose = first_wrist_pose
        first_req.goalType = motion_srv.MotionPlanRequest.CARTESIAN_SPACE_GOAL
        first_plan = self.motion_proxy(first_req)

        if first_plan.planResult == motion_srv.MotionPlanResponse.SUCCESS:
            rospy.loginfo('%d: approach pose to target nearby is found!', iteration)
        else:
            rospy.logwarn('%d: no solution of approach pose to target nearby', iteration)
            final_plan.planResult = motion_srv.MotionPlanResponse.ERROR_NO_SOLUTION
            return final_plan

        # get last joint state for waypoint
        waypoint_pos = copy.copy(first_plan.jointTrajectory.points[-1].positions)
        time_from_start = copy.copy(first_plan.jointTrajectory.points[-1].time_from_start)

        joint_state = JointState()
        joint_state.header = first_plan.jointTrajectory.header
        joint_state.name = first_plan.jointTrajectory.joint_names
        joint_state.position = waypoint_pos

        # second motion
        second_req = motion_srv.MotionPlanRequest()
        second_req.obstacle_ids = obstacle_ids
        second_req.obstacles = obstacles
        second_req.targetPose = second_wrist_pose
        second_req.currentJointState = joint_state
        # second_req.goalType = MotionPlanRequest.CARTESIAN_WITH_ORIENTATION_CONSTRAINTS
        second_plan = self.motion_proxy(second_req)

        if second_plan.planResult == motion_srv.MotionPlanResponse.SUCCESS:
            rospy.loginfo('%d: final approach pose to target is found!', iteration)
        else:
            rospy.logwarn('%d: no solution of final approach pose to target', iteration)
            final_plan.planResult = motion_srv.MotionPlanResponse.ERROR_NO_SOLUTION
            return final_plan

        # first + second
        first_points = first_plan.jointTrajectory.points
        second_points = second_plan.jointTrajectory.points
        for point in second_points:
            point.time_from_start += (time_from_start + rospy.Time(0.2))

        final_plan.jointTrajectory.header = second_plan.jointTrajectory.header
        final_plan.jointTrajectory.points = first_points + second_points
        final_plan.jointTrajectory.joint_names = second_plan.jointTrajectory.joint_names
        final_plan.planResult = motion_srv.MotionPlanResponse.SUCCESS
        return final_plan

    def _get_approach_pose(self, body_type, azimuth, target_bounding_box):
        """Get gripper's approaching pose"""
        # === STEP 1: ee_pose ===
        target_size = target_bounding_box.size
        target_xyz = target_bounding_box.center.position
        # target_quat = target_bounding_box.center.orientation

        # get transforms between wrist and end-effector
        base_to_ee_trans = base_to_ee_rot = []
        ee_to_wrist_trans = ee_to_wrist_rot = []

        side = "left" if body_type is motion_srv.MotionPlanRequest.LEFT_ARM else "right"
        tf_base = '/base_footprint'
        tf_ee = '/%s_end_effect_point' % side
        tf_gripper = self.gripper_pose[side]
        offset = self.gripper_pose['%s_offset' % side]
        while True:
            try:
                base_to_ee_trans, base_to_ee_rot = self.listener.lookupTransform(tf_base, tf_ee, rospy.Time(0))
                ee_to_wrist_trans, ee_to_wrist_rot = self.listener.lookupTransform(tf_ee, tf_gripper, rospy.Time(0))
                break
            except Exception:
                pass

        rad_azimuth = math.radians(azimuth)
        cos_azimuth = math.cos(rad_azimuth)
        sin_azimuth = math.sin(rad_azimuth)

        first_ee_pose = Pose()
        first_ee_pose.position.x = target_xyz.x + (target_size.x / 2 + offset[1]) * cos_azimuth
        first_ee_pose.position.y = target_xyz.y + (target_size.x / 2 + offset[1]) * sin_azimuth
        first_ee_pose.position.z = target_xyz.z

        second_ee_pose = Pose()
        second_ee_pose.position.x = target_xyz.x + (target_size.x / 2 + offset[0]) * cos_azimuth
        second_ee_pose.position.y = target_xyz.y + (target_size.x / 2 + offset[0]) * sin_azimuth
        second_ee_pose.position.z = target_xyz.z

        # === STEP 2: wrist_pose ===
        # self.rot_x(azimuth) = tfm.quaternion_about_axis(math.radians(azimuth), (1, 0, 0)),
        init_quat = [0, -0.707, 0, 0.707]
        quat = tfm.quaternion_multiply(init_quat, self.rot_x(azimuth))
        if body_type is motion_srv.MotionPlanRequest.RIGHT_ARM:
            quat = tfm.quaternion_multiply(quat, self.rot_z(180))
        quat = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        first_ee_pose.orientation = second_ee_pose.orientation = quat

        # Wrist
        trans2_mat = tf.transformations.translation_matrix(ee_to_wrist_trans)
        rot2_mat = tf.transformations.quaternion_matrix(ee_to_wrist_rot)
        mat2 = np.dot(trans2_mat, rot2_mat)

        first_wrist_pose = self._transform_from_ee_to_wrist(first_ee_pose, mat2)
        second_wrist_pose = self._transform_from_ee_to_wrist(second_ee_pose, mat2)
        return first_wrist_pose, second_wrist_pose, first_ee_pose, second_ee_pose

    def _transform_from_ee_to_wrist(self, ee_pose, mat2):
        _p = ee_pose.position
        _o = ee_pose.orientation
        trans1_mat = tf.transformations.translation_matrix([_p.x, _p.y, _p.z])
        rot1_mat = tf.transformations.quaternion_matrix([_o.x, _o.y, _o.z, _o.w])
        mat1 = np.dot(trans1_mat, rot1_mat)

        mat3 = np.dot(mat1, mat2)
        trans3 = tf.transformations.translation_from_matrix(mat3)
        rot3 = tf.transformations.quaternion_from_matrix(mat3)

        wrist_pose = Pose()
        wrist_pose.position.x = trans3[0]
        wrist_pose.position.y = trans3[1]
        wrist_pose.position.z = trans3[2]
        wrist_pose.orientation.x = rot3[0]
        wrist_pose.orientation.y = rot3[1]
        wrist_pose.orientation.z = rot3[2]
        wrist_pose.orientation.w = rot3[3]
        return wrist_pose
