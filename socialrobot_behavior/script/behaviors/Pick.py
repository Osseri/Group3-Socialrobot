import numpy as np
import tf
import rospy
import rosservice
import copy
from behavior import BehaviorBase

from socialrobot_motion.srv import *
from socialrobot_hardware.srv import *
from sensor_msgs.msg import *
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler, quaternion_matrix

#################
## Input value ##
#################

# necessary : targetBody, obstacle_ids, obstacles, targetObject
#             grasp_point

##################
## Output value ##
##################

# planResult, jointTrajectory


def calc_pose_offset(pose, quaternion, offset):
    # find tf matrix by pose's orientation
    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    transf_m = quaternion_matrix(quaternion)

    # find hand z axis
    # When gripper is forward, z-axis is [0, 0, -1].
    # hand z_vector is [0, 0, -1].
    # transf_m(3x3) * [0, 0, -1] = -transf_m[*][2]
    # So, the length of z_vector is 1.
    z_vector = np.array([-transf_m[0][2], -transf_m[1][2], -transf_m[2][2]])
    offset_vector = offset * z_vector

    offset_pose = Pose()
    offset_pose.position.x = pose.position.x + offset_vector[0]
    offset_pose.position.y = pose.position.y + offset_vector[1]
    offset_pose.position.z = pose.position.z + offset_vector[2]
    offset_pose.orientation = pose.orientation
    return offset_pose


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
        self.srv_arm_plan = rospy.ServiceProxy('/motion_plan/move_arm', MotionPlan)
        self.srv_grasp_plan = rospy.ServiceProxy('/motion_plan/grasp_plan', MotionPlan)

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
        arm_req = MotionPlanRequest()  # for arm_planner service
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
            # int32 SUCCESS = 0
            # int32 ERROR_INPUT = 1
            # int32 ERROR_NO_SOLUTION = 2
            # int32 ERROR_FAIL = 3
            rospy.loginfo("\n_moveit.manipulability")
            rospy.logwarn(grasp_pose)
            if _moveit.planResult == MotionPlanResponse.SUCCESS:
                _manipulability_list.append((_moveit.manipulability, grasp_pose))
        _manipulability_list.sort(reverse=True)
        rospy.loginfo('Manipulability is calculated.')

        ### calculate approach pos and total path ###
        for manipulability, grasp_pose in _manipulability_list:
            # planning graspit!
            rospy.loginfo("[behavior Pick] try grasp_pose:\n%s" % grasp_pose)
            grasp_req.gripper_pose = grasp_pose
            _graspit = self.srv_grasp_plan(grasp_req)

            # planning moveit!
            arm_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL
            for gripper_pose in _graspit.endEffectorPose:
                arm_req.targetPose = gripper_pose
                _moveit = self.srv_arm_plan(arm_req)

                if _moveit.planResult == MotionPlanResponse.SUCCESS:
                    rospy.logwarn("SUCCESS 0: availability")
                    # calculate the approach pose from gripper pose
                    # change distance...
                    approach_pos = calc_pose_offset(
                        gripper_pose,
                        gripper_pose.orientation,
                        0.0289,
                    )
                    arm_req.targetPose = approach_pos

                    # first_plan(msg_type) : MotionPlanResponse
                    first_plan = self.srv_arm_plan(arm_req)

                    if first_plan.planResult == MotionPlanResponse.SUCCESS:
                        rospy.logwarn("SUCCESS 1: first plan")
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
                        second_req.targetPose = gripper_pose

                        # second_plan(msg_type) : MotionPlanResponse
                        second_plan = self.srv_arm_plan(second_req)
                        if second_plan.planResult == MotionPlanResponse.SUCCESS:
                            rospy.logwarn("SUCCESS 2: second plan")
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


if __name__ == "__main__":
    from std_msgs.msg import *
    from geometry_msgs.msg import *
    from vision_msgs.msg import *
    from sensor_msgs.msg import *
    from socialrobot_motion.srv import *
    from socialrobot_behavior.srv import *
    rospy.init_node('pick_behavior')
    pb = PickBehavior('pick')

    plan_req = GetMotionRequest()

    # arm type
    plan_req.planner_name = "pick"
    plan_req.inputs.targetBody = plan_req.inputs.LEFT_ARM

    # red_gotica
    obs1 = BoundingBox3D()
    c1 = Pose()
    c1.position.x = +3.0000e-01
    c1.position.y = +9.9997e-02
    c1.position.z = +8.2886e-01
    c1.orientation.x = 1.31936e-05
    c1.orientation.y = 2.20794e-10
    c1.orientation.z = 6.07222e-07
    c1.orientation.w = 1
    obs1.center = c1
    v1 = Vector3()
    v1.x = 0.0618015
    v1.y = 0.059508
    v1.z = 0.23814
    obs1.size = v1

    # gotica
    obs2 = BoundingBox3D()
    c2 = Pose()
    c2.position.x = +4.0000e-01
    c2.position.y = -1.5003e-02
    c2.position.z = +8.2886e-01
    c2.orientation.x = 1.31627e-05
    c2.orientation.y = 2.26816e-10
    c2.orientation.z = -1.15535e-18
    c2.orientation.w = 1.0
    obs2.center = c2
    v2 = Vector3()
    v2.x = 0.065
    v2.y = 0.065
    v2.z = 0.23544
    obs2.size = v2

    # table
    obs3 = BoundingBox3D()
    c = Pose()
    c.position.x = 0.550006
    c.position.y = 8.80659e-06
    c.position.z = 0.365011
    c.orientation.x = 0
    c.orientation.y = 0
    c.orientation.z = 0.707
    c.orientation.w = 0.707
    obs3.center = c
    v = Vector3()
    v.x = 1.1342161893844604
    v.y = 0.7088739275932312
    v.z = 0.6899999976158142
    obs3.size = v

    # grasp point (juice)
    from math import *
    for i in range(2):
        for j in range(2):
            euler = [pi * (i) / 6, pi * j, pi / 2]
            # rule: Axes 4-string (e.g. 'sxyz' or 'ryxy')
            #     - first character : rotations are applied to 's'tatic or 'r'otating frame
            #     - remaining characters : successive rotation axis 'x', 'y', or 'z'
            # quaternion = quaternion_from_euler(x, y, z, axes="sxyz")
            quat = quaternion_from_euler(euler[0], euler[1], euler[2], axes="rzxy"),
            grasp_p = calc_pose_offset(c1, quat, 0.17)
            grasp_p.orientation.x = quat[0]
            grasp_p.orientation.y = quat[1]
            grasp_p.orientation.z = quat[2]
            grasp_p.orientation.w = quat[3]
            plan_req.inputs.grasp_point.append(grasp_p)

    # add obstacles
    plan_req.inputs.obstacle_ids = ['obj_red_gotica', 'obj_gotica', 'obj_table']
    plan_req.inputs.obstacles.append(obs1)
    plan_req.inputs.obstacles.append(obs2)
    plan_req.inputs.obstacles.append(obs3)

    # add target obstacle
    plan_req.inputs.targetObject = ['obj_red_gotica']

    res = pb.get_motion(plan_req.inputs)

    # set behavior
    behavior_srv = rospy.ServiceProxy('/behavior/set_behavior', SetBehavior)
    behavior_req = SetBehaviorRequest()
    behavior_req.header.frame_id = 'pick'
    behavior_req.trajectory = res.jointTrajectory
    behavior_res = behavior_srv(behavior_req)