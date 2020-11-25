import os
import math
import rospy
import eigenpy
import numpy as np
import moveit_commander
import numpy.linalg as lin

from sensor_msgs.msg import *
from six import with_metaclass
from socialrobot_motion.srv import *
from moveit_msgs.msg import DisplayTrajectory, RobotState
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_matrix

####################
### Object model ###
####################

#  path : ../socialrobot_motion/mesh/moveit/

###################
### Input value ###
###################

## Option 1 : calculate manipulability ##
# necessary : targetBody, find_manipulability, goalType, targetPose

## Option 2 : update detach & attach ##
# necessary : targetBody, obstacle_ids, obstacles, targetObject

## Option 3 : compute path ##
# necessary : targetBody, obstacle_ids, obstacles, goalType, targetPose or targetJointState
# choose(default) : currentJointState(current joint state in moveit!)

####################
### Output value ###
####################

# Option 1 : planResult, manipulability
# Option 2 : x
# Option 3 : planResult, jointTrajectory


class Singleton(type):
    '''
    for singleton pattern
    '''
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(
                Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class ArmPlanner(with_metaclass(Singleton)):
    def __init__(self, **params):
        self.vis_pub = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=100)

        self.robot = params['robot']
        self.scene = params['scene']
        self.group = ''
        self.attached_objects = {'left': [], 'right': []}
        self.detected_objects = []
        self.reset_bool = False

        # object model file path
        self.dir_path = os.path.abspath(os.path.dirname(
            os.path.dirname(__file__))) + "/mesh/moveit/"

        # set group list
        group_list = self.robot.get_group_names()
        self.leftarm_group = None
        if 'left_arm' in group_list:
            self.leftarm_group = self.robot.get_group('left_arm')
            self.lefteef_group = self.robot.get_group('left_eef')
        self.rightarm_group = None
        if 'right_arm' in group_list:
            self.rightarm_group = self.robot.get_group('right_arm')
            self.righteef_group = self.robot.get_group('right_eef')
        self.dualarm_group = None
        if 'dual_arm' in group_list:
            self.dualarm_group = self.robot.get_group('dual_arm')

    def reset(self):
        self.reset_bool = True
        arm_group = [self.leftarm_group, self.rightarm_group]
        for i, group in enumerate(arm_group):
            eef_link = group.get_end_effector_link()
            self.scene.remove_attached_object(eef_link)
        self.scene.remove_world_object()

    def calculate_manipulability(self, req):
        eigenpy.switchToNumpyMatrix()
        # clear scene
        if self.reset_bool == False:
            self.reset()

        _plan = self.compute_path(req)
        if _plan.joint_trajectory.points:
            joint_values = []
            z_axis = np.array([0, 0, 1])
            # get joint state
            for i, value in enumerate(_plan.joint_trajectory.points[-1].positions):
                joint_values.append(value)

            jacobian = self.group.get_jacobian_matrix(joint_values)[0:3, :]
            quaternion = [req.targetPose.orientation.x, req.targetPose.orientation.y,
                          req.targetPose.orientation.z, req.targetPose.orientation.w]
            vector = np.dot(np.array(quaternion_matrix(quaternion)[
                0:3, 0:3]), z_axis.T)
            unit_vector = vector/lin.norm(vector)

            # ellipsoid equation : V^T(J*J^T)^(-1)V = 1
            # calculate : v^T(J*J^T)^(-1)v
            A = np.dot(np.dot(unit_vector, lin.inv(
                np.dot(jacobian, jacobian.T))), unit_vector.T)

            # manipulability = sqrt(1/v^T(J*J^T)^(-1)v)
            manipulability = math.sqrt(1/A)

            return MotionPlanResponse().SUCCESS, manipulability
        return MotionPlanResponse().ERROR_FAIL, 0

    def callback_arm_plan(self, req):
        res = MotionPlanResponse()

        ### set params ###
        if req.targetBody == req.LEFT_ARM:
            self.group = self.leftarm_group
        elif req.targetBody == req.RIGHT_ARM:
            self.group = self.rightarm_group
        elif req.targetBody == req.BOTH_ARM:
            self.group = self.dualarm_group

        ### set moveit options ###
        self.group.clear_path_constraints()
        self.group.clear_pose_targets()
        self.group.set_planning_time(0.5)
        self.group.set_max_acceleration_scaling_factor(1.0)
        self.group.set_max_velocity_scaling_factor(1.0)
        self.group.set_num_planning_attempts(3)
        # self.group.allow_replanning(True)

        ### set start pose ###
        if req.currentJointState != sensor_msgs.msg.JointState():
            start_state = RobotState()
            start_state.joint_state = req.currentJointState
            self.group.set_start_state(start_state)
        else:
            self.group.set_start_state_to_current_state()

        ### Option 1 : calculate manipulability ###
        if req.find_manipulability:
            self.group.set_planning_time(0.2)
            rospy.loginfo("[Arm planner] calculate manipulability..")
            res.planResult, res.manipulability = self.calculate_manipulability(
                req)
            return res

        ### update objects ###
        for i, obj in enumerate(req.obstacle_ids):
            self.add_obstacles(obj, req.obstacles[i])

        ### Option 2 : update detach & attach ###
        self.update_robot_scene(req)

        ### Option 3 : compute path ###
        if req.targetJointState != sensor_msgs.msg.JointState() or req.targetPose != Pose():
            # compute path
            plan = self.compute_path(req)

            # visualize path
            self.visualize_trajectory(plan)

            # return result
            if plan.joint_trajectory.points:
                rospy.loginfo('[Arm planner] solution is found..')
                res.planResult = MotionPlanResponse().SUCCESS
                res.jointTrajectory = plan.joint_trajectory
            else:
                rospy.logerr('[Arm planner] have no solution..')
                res.planResult = MotionPlanResponse().ERROR_NO_SOLUTION

        return res

    def update_robot_scene(self, req):
        target_object = []
        attached_objects = self.scene.get_attached_objects()

        if len(req.targetObject)>0:
            target_objects = req.targetObject

        # for obj in self.attached_objects['left']:
        #     if obj in self.scene.get_known_object_names():
        #         self.attach_object(obj, self.leftarm_group)
        # for obj in self.attached_objects['right']:
        #     if obj in self.scene.get_known_object_names():
        #         self.attach_object(obj, self.rightarm_group)
            
            # detach object 
            for obj in attached_objects:
                if not obj in target_objects:
                    self.detach_object(obj, self.group)

            # attach target object
            for target in target_objects:
                if target in self.scene.get_known_object_names():
                    self.attach_object(target, self.group)    

        else:
            # detach object 
            for obj in attached_objects:
                self.detach_object(obj, self.group)
		    

    def attach_object(self, object_id, target_group):
        touch_group = ''
        eef_link = target_group.get_end_effector_link()
        if target_group.get_name() == 'left_arm':
            touch_group = self.lefteef_group
            self.attached_objects['left'].append(object_id)
        elif target_group.get_name() == 'right_arm':
            touch_group = self.righteef_group
            self.attached_objects['right'].append(object_id)

        touch_links = self.robot.get_link_names(group=touch_group.get_name())
        self.scene.attach_box(
            eef_link, object_id, touch_links=touch_links)
        #rospy.loginfo(self.wait_for_state_update(object_id, self.scene))
        return True

    def detach_object(self, object_id, target_group):
        if target_group.get_name() == 'left_arm':
            self.attached_objects['left'].remove(object_id)
        elif target_group.get_name() == 'right_arm':
            self.attached_objects['right'].remove(object_id)
        self.scene.remove_attached_object(
            target_group.get_end_effector_link(), name=object_id)

    def compute_path(self, req):
        if(req.goalType == req.JOINT_SPACE_GOAL):
            joint_goal = self.group.get_current_joint_values()
            for i in range(len(joint_goal)):
                joint_goal[i] = req.targetJointState.position[i]
            self.group.set_joint_value_target(joint_goal)
        elif(req.goalType == req.CARTESIAN_SPACE_GOAL):
            self.group.set_pose_target(req.targetPose)

        plan = self.group.plan()

        self.group.stop()
        self.group.clear_pose_targets()
        return plan

    def visualize_trajectory(self, plan):
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.vis_pub.publish(display_trajectory)
        return

    def add_obstacles(self, obstacle_name, obstacle):
        self.reset_bool = False

        # set objects params
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = self.robot.get_planning_frame()
        object_pose.pose = obstacle.center
        mesh_size = [obstacle.size.x, obstacle.size.y, obstacle.size.z]
        obstacle_dir = self.dir_path + obstacle_name + ".stl"

        # set object scale
        if obstacle_name == 'obj_juice':
            scale = [1, 1, 1]
        else:
            scale = [0.001, 0.001, 0.001]

        try:
            try:
                # check version : pyassimp 4.1.3
                # 1. sudo dpkg --remove --force-depends python-pyassimp
                # 2. sudo -H pip install pyassimp==4.1.3
                self.scene.add_mesh(
                    obstacle_name, object_pose, obstacle_dir, size=scale)
            except:
                self.scene.add_box(obstacle_name, object_pose, mesh_size)

            # wait scene update
            while(not (obstacle_name in self.scene.get_known_object_names())):
                rospy.sleep(0.1)
        except:
            rospy.logerr("[Arm planner] can not import %s..." % obstacle_name)
            return -1
        return 0

    # def wait_for_state_update(box_name, scene, box_is_known=False, box_is_attached=False, timeout=4):
    #     start = rospy.get_time()
    #     seconds = rospy.get_time()
    #     while (seconds - start < timeout) and not rospy.is_shutdown():
    #         attached_objects = scene.get_attached_objects(box_name)
    #         is_attached = len(attached_objects.keys()) > 0
    #         is_known = box_name in scene.get_known_object_names()
    #         if (box_is_attached == is_attached) and (box_is_known == is_known):
    #             return True
    #         rospy.sleep(0.2)
    #         seconds = rospy.get_time()
    #     return False
