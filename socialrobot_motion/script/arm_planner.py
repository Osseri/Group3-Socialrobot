from codecs import backslashreplace_errors
import os
import math
import rospy
import eigenpy
import tf
import numpy as np
import numpy.linalg as lin
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from socialrobot_msgs.srv import *
import socialrobot_msgs.msg as social_msg
import std_srvs.srv as std_srv
from six import with_metaclass
from socialrobot_motion.srv import *
from moveit_msgs.msg import DisplayTrajectory, RobotState, Constraints, OrientationConstraint, PositionConstraint
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
        self.listener = tf.TransformListener()
        self.vis_pub = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=100)
        self.commander = params['commander']
        self.robot = params['robot']
        self.scene = params['scene']
        self.group = ''
        self.attached_objects = {'left': [], 'right': []}
        self.detected_objects = []
        self.planning_frame = self.robot.get_planning_frame()
        self.is_reset = False

        # object model file path
        self.dir_path = os.path.abspath(os.path.dirname(
            os.path.dirname(__file__))) + "/mesh/moveit/"

        # set group list
        group_list = self.robot.get_group_names()
        self.leftarm_group = None
        self.rightarm_group = None
        self.lefteef_group = None
        self.righteef_group = None
        
        if 'left_arm' in group_list:
            self.leftarm_group = self.robot.get_group('left_arm')
            self.lefteef_group = self.robot.get_group('left_eef')
            if 'left_arm_only' in group_list:
                self.leftarm_only_group = self.robot.get_group('left_arm_only')
            
        if 'right_arm' in group_list:
            self.rightarm_group = self.robot.get_group('right_arm')
            self.righteef_group = self.robot.get_group('right_eef')
            if 'right_arm_only' in group_list:
                self.rightarm_only_group = self.robot.get_group('right_arm_only')
            
        if 'dual_arm' in group_list:
            self.dualarm_group = self.robot.get_group('dual_arm')

    def reset(self):
        self.is_reset = True
        arm_group = [self.leftarm_group, self.rightarm_group]
        for i, group in enumerate(arm_group):
            eef_link = group.get_end_effector_link()
            self.scene.remove_attached_object(eef_link)
        self.scene.remove_world_object()

    def calculate_manipulability(self, req):
        eigenpy.switchToNumpyMatrix()
        # clear scene
        if self.is_reset == False:
            self.reset()

        _plan = self.compute_path(req)
        if _plan.joint_trajectory.points:
            joint_values = []
            z_axis = np.array([0, 0, 1])
            # get joint state
            for i, value in enumerate(_plan.joint_trajectory.points[-1].positions):
                joint_values.append(value)

            jacobian = self.group.get_jacobian_matrix(joint_values)[0:3, :]
            target_pose = req.targetPose[0]
            quaternion = [target_pose.orientation.x, target_pose.orientation.y,
                          target_pose.orientation.z, target_pose.orientation.w]
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

    def callback_reset(self, req):
        self.reset()
        return std_srv.EmptyResponse()

    def callback_get_group_info(self, req):
        res = GetGroupsResponse()
        for name in req.group_name:
            group = social_msg.Group()
            if name == 'left_arm':
                while(self.leftarm_group == None):
                    pass
                group.name = self.leftarm_group.get_name()
                group.active_joints = self.leftarm_group.get_active_joints()
            elif name == 'right_arm':
                while(self.rightarm_group == None):
                    pass
                group.name = self.rightarm_group.get_name()
                group.active_joints = self.rightarm_group.get_active_joints()
            elif name == 'left_eef':
                while(self.lefteef_group == None):
                    pass
                group.name = self.lefteef_group.get_name()
                group.active_joints = self.lefteef_group.get_active_joints()
            elif name == 'right_eef':
                while(self.righteef_group == None):
                    pass
                group.name = self.righteef_group.get_name()
                group.active_joints = self.righteef_group.get_active_joints()
            res.groups.append(group)

        return res

    def callback_update_scene_objects(self, req):
        res = UpdateObjectsResponse(result=UpdateObjectsResponse.FAIL)
        command = req.command
        obj_ids = req.object_ids
        obj_list = req.objects

        if command == UpdateObjectsRequest.ADD:
            for obj in obj_list:
                self.add_obstacles(obj.id, obj.bb3d)
                for aff in obj.affordance:
                    self.add_obstacles(aff.id, aff.bb3d)
        elif command == UpdateObjectsRequest.REMOVE:
            for obj in obj_ids:
                self.scene.remove_world_object(obj)

        return UpdateObjectsResponse(result=UpdateObjectsResponse.SUCCESS)

    def callback_get_scene_objects(self, req):
        res = GetObjectsResponse()

        # get object informations from moveit scene
        attached_objects = self.scene.get_attached_objects()
        scene_objects = self.scene.get_objects()

        for obj in scene_objects.keys():
            scene_object = social_msg.Object()
            scene_object.id = scene_objects[obj].id
            if scene_objects[obj].primitives: # if object shape is primitive
                scene_object.bb3d.size.x = scene_objects[obj].primitives[0].dimensions[0]
                scene_object.bb3d.size.y = scene_objects[obj].primitives[0].dimensions[1]
                scene_object.bb3d.size.z = scene_objects[obj].primitives[0].dimensions[2]
                scene_object.bb3d.center.position = scene_objects[obj].primitive_poses[0].position
                scene_object.bb3d.center.orientation = scene_objects[obj].primitive_poses[0].orientation
                res.objects.append(scene_object)
            elif scene_objects[obj].mesh_poses: # object mesh data is known
                scene_object.bb3d.center.position = scene_objects[obj].mesh_poses[0].position
                scene_object.bb3d.center.orientation = scene_objects[obj].mesh_poses[0].orientation
                res.objects.append(scene_object)

        for obj in attached_objects.keys():            
            rel_pos = rel_ori = None
            scene_object = social_msg.Object()
            scene_object.id = attached_objects[obj].object.id
            if attached_objects[obj].object.primitives>0: # if object shape is primitive
                scene_object.bb3d.size.x = attached_objects[obj].object.primitives[0].dimensions[0]
                scene_object.bb3d.size.y = attached_objects[obj].object.primitives[0].dimensions[1]
                scene_object.bb3d.size.z = attached_objects[obj].object.primitives[0].dimensions[2]
                rel_pos = attached_objects[obj].object.primitive_poses[0].position
                rel_ori = attached_objects[obj].object.primitive_poses[0].orientation

            elif attached_objects[obj].object.mesh_poses>0: # object mesh data is known
                rel_pos = attached_objects[obj].object.mesh_poses[0].position
                rel_ori = attached_objects[obj].object.mesh_poses[0].orientation                

            # transform attached object pose based on robot base            
            parent_link = attached_objects[obj].object.header.frame_id

            base_to_eef_trans, base_to_eef_rot = self.listener.lookupTransform(
                        '/base_footprint', parent_link, rospy.Time(0))

            trans_mat = tf.transformations.translation_matrix(base_to_eef_trans)
            rot_mat = tf.transformations.quaternion_matrix(base_to_eef_rot)
            base_to_eef_mat = np.dot(trans_mat, rot_mat)

            trans_mat = tf.transformations.translation_matrix([rel_pos.x, rel_pos.y, rel_pos.z])
            rot_mat = tf.transformations.quaternion_matrix([rel_ori.x, rel_ori.y, rel_ori.z,rel_ori.w])
            eef_to_obj_mat = np.dot(trans_mat, rot_mat)
            base_to_obj_mat = np.dot(base_to_eef_mat, eef_to_obj_mat)

            base_pos = tf.transformations.translation_from_matrix(base_to_obj_mat)
            base_quat = tf.transformations.quaternion_from_matrix(base_to_obj_mat)

            scene_object.bb3d.center.position.x = base_pos[0]
            scene_object.bb3d.center.position.y = base_pos[1]
            scene_object.bb3d.center.position.z = base_pos[2]

            scene_object.bb3d.center.orientation.x = base_quat[0]
            scene_object.bb3d.center.orientation.y = base_quat[1]
            scene_object.bb3d.center.orientation.z = base_quat[2]
            scene_object.bb3d.center.orientation.w = base_quat[3]

            res.objects.append(scene_object)

        return res

    def callback_arm_plan(self, req):
        res = MotionPlanResponse()

        # set plannig group
        if req.targetBody == req.LEFT_ARM or req.targetBody == req.LEFT_GRIPPER:
            self.group = self.leftarm_group
        elif req.targetBody == req.LEFT_ARM_WITHOUT_WAIST:
            self.group = self.leftarm_only_group
        elif req.targetBody == req.RIGHT_ARM or req.targetBody == req.RIGHT_GRIPPER:
            self.group = self.rightarm_group
        elif req.targetBody == req.RIGHT_ARM_WITHOUT_WAIST:
            self.group = self.rightarm_only_group
        elif req.targetBody == req.BOTH_ARM:
            self.group = self.dualarm_group

        # # detach object if gripper is opened 
        # elif req.targetBody == req.LEFT_GRIPPER:
        #     self.group = self.leftarm_group
        #     for obj in self.scene.get_attached_objects():
        #         self.detach_object(obj, self.group)
        #     res.planResult = MotionPlanResponse().SUCCESS
        #     return res
        # elif req.targetBody == req.RIGHT_GRIPPER:
        #     self.group = self.rightarm_group
        #     for obj in self.scene.get_attached_objects():
        #         self.detach_object(obj, self.group)
        #     res.planResult = MotionPlanResponse().SUCCESS
        #     return res

        ### set moveit default options ###
        self.group.clear_pose_targets()
        self.group.set_planning_time(1.5)
        self.group.set_max_acceleration_scaling_factor(1.0)
        self.group.set_max_velocity_scaling_factor(1.0)
        self.group.set_num_planning_attempts(20)
        # self.group.allow_replanning(True)

        # set robot velocity
        if req.velocity_scaling_factor > 0.0:
            self.group.set_max_velocity_scaling_factor(req.velocity_scaling_factor)
        if req.velocity_scaling_factor > 0.0:
            self.group.set_max_velocity_scaling_factor(req.velocity_scaling_factor)

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
        
        ### update moveit scene ###
        self.update_robot_scene(req)
        if req.goalType == -1:
            return res

        ### Option 3 : compute path ###
        if len(req.targetJointState)>0 or len(req.targetPose)>0:
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

        attached_objects = self.scene.get_attached_objects()
        rospy.logwarn("attached objects:")
        print(attached_objects)

        target_objects = req.targetObject
        rospy.logwarn("target_objects:")
        print(target_objects)                  

        # if there is target object in requirements, attach it
        if target_objects:
            rospy.logwarn("attach objects.")
            for target in target_objects:
                if target in self.scene.get_known_object_names():
                    rospy.logwarn('%s', target)
                    self.attach_object(target, self.group) 
            rospy.sleep(rospy.Duration(1.0))    # wait for update
            attached_objects = self.scene.get_attached_objects()
            rospy.logwarn("attached objects:")
            print(attached_objects)

        # if there are no target object and attached object, detach attached object
        else:
            rospy.logwarn("detach objects.")
            # detach object 
            for obj in attached_objects:
                self.detach_object(obj, self.group)
            pass	                   
        rospy.sleep(rospy.Duration(1.0))    # wait for update

        # update objects into scene
        rospy.logwarn("update objects.")
        for i, obj in enumerate(req.obstacle_ids):
            if obj not in attached_objects:
                rospy.logwarn('%s', obj)
                self.add_obstacles(obj, req.obstacles[i]) 
        rospy.sleep(rospy.Duration(1.0))    # wait for update

    def attach_object(self, object_id, target_group):
        touch_group = ''
        eef_link = target_group.get_end_effector_link()
        if 'left' in target_group.get_name():
            touch_group = self.lefteef_group
            if not object_id in self.attached_objects['left']:
                self.attached_objects['left'].append(object_id)
        elif 'right' in target_group.get_name():
            touch_group = self.righteef_group
            if not object_id in self.attached_objects['right']:
                self.attached_objects['right'].append(object_id)
        else:
            #TODO: attach the object
            touch_group = self.dualarm_group

        touch_links = self.robot.get_link_names(group=touch_group.get_name())
        self.scene.attach_box(eef_link, object_id, touch_links=touch_links)        

    def detach_object(self, object_id, target_group):
        if 'left' in target_group.get_name() and object_id in self.attached_objects['left']:
            self.attached_objects['left'].remove(object_id)
        elif 'right' in target_group.get_name() and object_id in self.attached_objects['right']:
            self.attached_objects['right'].remove(object_id)
            
        self.scene.remove_attached_object(target_group.get_end_effector_link(), name=object_id)

    def set_constraints(self, req):

        # set constraints
        constraints = Constraints()
        constraints.name = "constraints"

        # orientation constraints
        if (req.orientation_constraint != Pose()):
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = '/base_footprint'
            orientation_constraint.link_name = self.group.get_end_effector_link()
            orientation_constraint.orientation = req.targetPose[0].orientation  #req.orientation_constraint.orientation
            orientation_constraint.absolute_x_axis_tolerance =  req.orientation_constraint.position.x #3.14 ignore this axis 
            orientation_constraint.absolute_y_axis_tolerance = req.orientation_constraint.position.y
            orientation_constraint.absolute_z_axis_tolerance = req.orientation_constraint.position.z
            orientation_constraint.weight = 1                
            constraints.orientation_constraints.append(orientation_constraint)

        if (req.position_constraint != PoseWithCovariance()):
            # position constraint constraints
            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = '/base_footprint'
            position_constraint.link_name = self.group.get_end_effector_link()
            position_constraint.target_point_offset.x 
            position_constraint.target_point_offset.y 
            position_constraint.target_point_offset.z 
            position_constraint.constraint_region
            position_constraint.weight = 1
            constraints.position_constraints.append(position_constraint)

        self.group.set_path_constraints(constraints) 

    def compute_path(self, req):
        # set path constraints
        self.set_constraints(req)

        # Joint space plan
        if(req.goalType == req.JOINT_SPACE_GOAL):
            joint_goal = self.group.get_current_joint_values()
            if len(req.targetJointState)==1:    # single joint state goal
                target_joints = req.targetJointState[0]
                for i in range(len(joint_goal)):
                    joint_goal[i] = target_joints.position[i]
            else:   # multiple joint state goal
                pass
            self.group.set_joint_value_target(joint_goal)
            plan = self.group.plan()
        # Cartesian space plan
        elif(req.goalType == req.CARTESIAN_SPACE_GOAL):
            if len(req.targetPose)==1:    # single goal pose
                target_pose = req.targetPose[0]
                self.group.set_pose_target(target_pose)   
                plan = self.group.plan()            
            else:   # multiple goal pose
                (plan, fraction) = self.group.compute_cartesian_path(
                             req.targetPose,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold     

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

    def add_obstacles(self, obstacle_name, obstacle, is_tracking=False):
        self.is_reset = False

        # set objects params
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = self.planning_frame
        object_pose.pose = obstacle.center
        mesh_size = [obstacle.size.x, obstacle.size.y, obstacle.size.z]
        obstacle_dir = self.dir_path + obstacle_name + ".stl"

        # set object scale
        scale = [1,1,1]

        attached_objects = self.scene.get_attached_objects().keys()
        scene_objects = self.scene.get_known_object_names()

        try:
            try:
                # if object 3d model in database, add model from dir
                # check version : pyassimp 4.1.3
                # 1. sudo dpkg --remove --force-depends python-pyassimp
                # 2. sudo -H pip install pyassimp==4.1.3
                self.scene.add_mesh(
                    obstacle_name, object_pose, obstacle_dir, size=scale)
            except:
                # update tracking objects every time
                self.scene.add_box(obstacle_name, object_pose, mesh_size)
                # if is_tracking: 
                #     # update tracking objects every time
                #     self.scene.add_box(obstacle_name, object_pose, mesh_size)
                # else:
                #     # update only unknown objects
                #     if not obstacle_name in scene_objects and not obstacle_name in attached_objects:
                #             self.scene.add_box(obstacle_name, object_pose, mesh_size)

            # wait scene update
            while(not (obstacle_name in self.scene.get_known_object_names())):
               rospy.sleep(0.1)
        except:
            rospy.logerr("[Arm planner] can not add %s object..." % obstacle_name)
            return -1
        return 0

    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = box_name in self.scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.2)
            seconds = rospy.get_time()
        return False
