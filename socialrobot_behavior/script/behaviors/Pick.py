import tf
import rospy
import rosservice

from behavior import BehaviorBase

from socialrobot_motion.srv import *
from socialrobot_hardware.srv import *
from sensor_msgs.msg import *
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_matrix

#################
## Input value ##
#################

# necessary : targetBody, obstacle_ids, obstacles, targetObject
#             grasp_point

##################
## Output value ##
##################

# planResult, jointTrajectory


class PickBehavior(BehaviorBase):
    def __init__(self, name, **params):
        super(PickBehavior, self).__init__(name, **params)
        self.listener = tf.TransformListener()
        if self._hardware_if == 'vrep':
            self.service_name = '/sim_interface/set_motion'
            self.service_type = VrepSetJointTrajectory
        elif self._hardware_if == 'hw':
            self.service_name = '/hw_interface/set_motion'
            self.service_type = SetJointTrajectory
        self.srv_arm_plan = rospy.ServiceProxy(
            '/motion_plan/move_arm', MotionPlan)
        self.srv_grasp_plan = rospy.ServiceProxy(
            '/motion_plan/grasp_plan', MotionPlan)

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

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True

    def get_motion(self, inputs):
        '''
        return the trajectory
        '''
        res = MotionPlanResponse()
        rospy.loginfo("Calculating pick motion..")
        srv_res, srv_ret = self._call_ros_service(inputs)

        res.planResult = srv_res
        if srv_res == MotionPlanResponse.SUCCESS:
            rospy.loginfo("Pick planning is done..")
            res.jointTrajectory = srv_ret.jointTrajectory
        else:
            rospy.logerr("Pick planning is failed..")

        return res

    def _call_ros_service(self, inputs):
        manipulability_req = MotionPlanRequest()
        grasp_req = MotionPlanRequest()  # for grasp_planner service
        arm_req = MotionPlanRequest()   # for arm_planner service
        total_plan = MotionPlanResponse()

        ### set parameter from inputs ###
        # set targetBody
        body_type = inputs.targetBody
        arm_req.targetBody = body_type
        manipulability_req.targetBody = body_type
        if body_type == MotionPlanRequest.LEFT_ARM:
            grasp_req.targetBody = MotionPlanRequest.LEFT_GRIPPER
        elif body_type == MotionPlanRequest.RIGHT_ARM:
            grasp_req.targetBody = MotionPlanRequest.RIGHT_GRIPPER
        else:
            return (MotionPlanResponse.ERROR_INPUT, None)

        # set obstacles environment(obstacle_ids, obstacles)
        grasp_req.obstacle_ids = inputs.obstacle_ids
        grasp_req.obstacles = inputs.obstacles
        arm_req.obstacle_ids = inputs.obstacle_ids
        arm_req.obstacles = inputs.obstacles

        # set targetobject
        grasp_req.targetObject = inputs.targetObject

        ### calculate manipulability of grasp points ###
        _manipulability_list = []
        manipulability_req.find_manipulability = True
        manipulability_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL
        for i, grasp_pose in enumerate(inputs.grasp_point):
            manipulability_req.targetPose = grasp_pose
            _moveit = self.srv_arm_plan(manipulability_req)
            if _moveit.planResult == MotionPlanResponse.SUCCESS:
                _manipulability_list.append(
                    (_moveit.manipulability, grasp_pose))
        _manipulability_list.sort(reverse=True)

        ### calculate approach pos and total path ###
        for manipulability, grasp_pose in _manipulability_list:
            # planning graspit!
            grasp_req.gripper_pose = grasp_pose
            _graspit = self.srv_grasp_plan(grasp_req)
            # planning moveit!
            arm_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL
            for gripper_pose in _graspit.endEffectorPose:
                arm_req.targetPose = gripper_pose
                _moveit = self.srv_arm_plan(arm_req)

                if _moveit.planResult == MotionPlanResponse.SUCCESS:
                    # calculate the approach pose from gripper pose
                    # change distance...
                    approach_pos = self.approach_pos(gripper_pose, 0.05)
                    arm_req.targetPose = approach_pos

                    # first_plan(msg_type) : MotionPlanResponse
                    first_plan = self.srv_arm_plan(arm_req)
                    total_plan = first_plan

                    if first_plan.planResult == MotionPlanResponse.SUCCESS:
                        joint_state = sensor_msgs.msg.JointState()
                        joint_state.header = first_plan.jointTrajectory.header
                        joint_state.name = first_plan.jointTrajectory.joint_names
                        joint_state.position = first_plan.jointTrajectory.points[-1].positions
                        arm_req.currentJointState = joint_state
                        arm_req.targetPose = gripper_pose

                        # second_plan(msg_type) : MotionPlanResponse
                        second_plan = self.srv_arm_plan(arm_req)
                        if second_plan.planResult == MotionPlanResponse.SUCCESS:
                            for joint_trajectory in second_plan.jointTrajectory.points:
                                total_plan.jointTrajectory.points.append(
                                    joint_trajectory)
                            return (MotionPlanResponse.SUCCESS, total_plan)
        return (MotionPlanResponse.ERROR_NO_SOLUTION, None)

    def approach_pos(self, pose, dist):
        # find tf matrix by pose's orientation
        quaternion = [pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w]
        transf_m = quaternion_matrix(quaternion)

        # find z axis
        z_axis = [-transf_m[0][2], -transf_m[1][2], -transf_m[2][2]]
        center = [pose.position.x, pose.position.y, pose.position.z]

        approach_pos = Pose()
        approach_pos.orientation = pose.orientation
        approach_pos.position.x = center[0] + dist/(3**0.5)*z_axis[0]
        approach_pos.position.y = center[1] + dist/(3**0.5)*z_axis[1]
        approach_pos.position.z = center[2] + dist/(3**0.5)*z_axis[2]
        return approach_pos

    ### get current pose by tf ###
    # def get_current_pose(self, inputs):
    #     is_listen = True
    #     while is_listen:
    #         try:
    #             if inputs.targetBody is MotionPlanRequest.LEFT_ARM:
    #                 (base_to_eef_trans, base_to_eef_rot) = self.listener.lookupTransform(
    #                     '/base_footprint', '/LHand_base', rospy.Time(0))
    #             elif inputs.targetBody is MotionPlanRequest.RIGHT_ARM:
    #                 (base_to_eef_trans, base_to_eef_rot) = self.listener.lookupTransform(
    #                     '/base_footprint', '/RHand_base', rospy.Time(0))
    #             is_listen = False
    #         except:
    #             pass
    #     return (base_to_eef_trans, base_to_eef_rot)
