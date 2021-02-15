import os
import math
from six import with_metaclass
import rospy
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import arm_motion_planner.msg
import arm_motion_planner.srv
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryActionGoal
)
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from socialrobot_motion.srv import *
import tf
import numpy as np

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

class SingleArmPlanner(with_metaclass(Singleton)):
    left_group = ['Waist_Roll', 'Waist_Pitch', 'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll']
    right_group = ['Waist_Roll', 'Waist_Pitch', 'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll']
    dual_group = []

    def __init__(self, **params):
       
        self.current_group = None
        self.attached_objects = {'left': [], 'right': []}
        self.detected_objects = []
        self.reset_bool = False
        self.joint_states = None
        self.left_joint_states = None
        self.right_joint_states = None
        self.current_joint_states = None
        self.current_service = None

        # subscriber 
        self.sub_joints = rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self._callback_joints)
        self.listener = tf.TransformListener()

        # publisher
        self.pub_obstacle_marker = rospy.Publisher('/snu_test', MarkerArray,queue_size=10)
        self.pub_motion = rospy.Publisher('/cmd_joint', sensor_msgs.msg.JointState ,queue_size=10)

        # service for single arm planner
        rospy.wait_for_service("/plan_left_arm_motion")
        self.plan_left_arm_motion = rospy.ServiceProxy(
            "/plan_left_arm_motion", arm_motion_planner.srv.plan_arm_motion
        )
        rospy.wait_for_service("/plan_right_arm_motion")
        self.plan_right_arm_motion = rospy.ServiceProxy(
            "/plan_right_arm_motion", arm_motion_planner.srv.plan_arm_motion
        )

    def callback_arm_plan(self, req):
        res = MotionPlanResponse()
        planner_req = arm_motion_planner.srv.plan_arm_motionRequest()

        ### set params ###
        from_tf = None
        to_tf = None
        if req.targetBody == req.LEFT_ARM:
            self.current_group = self.left_group
            self.current_service = self.plan_left_arm_motion
            to_tf = 'left_end_effect_point'
            from_tf = 'LHand_base'
        elif req.targetBody == req.RIGHT_ARM:
            self.current_group = self.right_group
            self.current_service = self.plan_right_arm_motion
            to_tf = 'right_end_effect_point'
            from_tf = 'RHand_base'
        elif req.targetBody == req.BOTH_ARM:
            self.current_group = self.dual_group


        ### set current joint state ###
        if req.currentJointState != sensor_msgs.msg.JointState():
            self.current_joint_states = req.currentJointState
        else:
            if self.current_group == self.left_group:
                self.current_joint_states = self.left_joint_states
            elif self.current_group == self.right_group:
                self.current_joint_states = self.right_joint_states
        planner_req.current_joint_state = self.current_joint_states
        
        ### set target pose ###
        # TODO: current target pose is  R & LHAND_BASE link because of moveit
        # transform eef to base tf
        #target_pose = self.transform_pose(req.targetPose, from_tf, to_tf)
        planner_req.target_ee_pose = req.targetPose

        ### update objects ###
        for i, bb3d in enumerate(req.obstacle_ids):
            # convert vision_msgs/BoundingBox3D into arm_motion_planner/Obstacle3D[] Obstacles3D
            obstacle = arm_motion_planner.msg.Obstacle3D()
            obstacle.Box_pose = req.obstacles[i].center
            obstacle.Box_dimension.x = req.obstacles[i].size.x/2.0
            obstacle.Box_dimension.y = req.obstacles[i].size.y/2.0
            obstacle.Box_dimension.z = req.obstacles[i].size.z/2.0
            planner_req.Obstacles3D.append(obstacle)

        ### constraints ###
        is_constraint = False
        if req.goalType > 0:
            is_constraint = True
        planner_req.Pose_bound = arm_motion_planner.msg.PoseConstraint(
                constrain_pose=std_msgs.msg.Bool(is_constraint),
                position_bound_lower=geometry_msgs.msg.Point32(-0.05, -0.05, -0.05),
                position_bound_upper=geometry_msgs.msg.Point32(0.05, 0.05, 0.05),
                orientation_bound_lower=geometry_msgs.msg.Point32(-0.15, -0.15, -0.15),
                orientation_bound_upper=geometry_msgs.msg.Point32(0.15, 0.15, 0.15),
            )
        planner_req.interpolate_path = std_msgs.msg.Bool(True)
        # ### compute path ###
        try:
            planner_res = self.current_service(planner_req)
            if(not planner_res.joint_trajectory.points):
                rospy.loginfo("motion planning is failed.")
                res.planResult = MotionPlanResponse().ERROR_NO_SOLUTION
                return res
            else:
                planner_res.joint_trajectory.header.stamp = rospy.Time.now()
                planner_res.joint_trajectory.header.frame_id = 'base_footprint'
                res.jointTrajectory = planner_res.joint_trajectory
                for i,pt in enumerate(res.jointTrajectory.points):                    
                    pt.velocities = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
                    pt.accelerations = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
                    pt.time_from_start = rospy.Duration(0.025*i)

                res.planResult = MotionPlanResponse().SUCCESS
                return res
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        return planner_res

    def _callback_joints(self, data):
        self.joint_states = data
        self.left_joint_states = sensor_msgs.msg.JointState()
        self.right_joint_states = sensor_msgs.msg.JointState()

        for i in self.left_group:
            idx = data.name.index(i)
            self.left_joint_states.header = data.header
            self.left_joint_states.name.append(i)
            self.left_joint_states.position.append(data.position[idx])

        
        for i in self.right_group:
            idx = data.name.index(i)
            self.right_joint_states.header = data.header
            self.right_joint_states.name.append(i)
            self.right_joint_states.position.append(data.position[idx])

    def transform_pose(self, pose, from_tf, to_tf):
        trans = None
        rot = None
        rate = rospy.Rate(10.0)
        while(trans == None or rot == None):
            try:                                 
                trans, rot = self.listener.lookupTransform(from_tf, to_tf, rospy.Time(0))
            except:
                rate.sleep()
                pass

        # transform
        obj_trans_mat = tf.transformations.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
        obj_rot_mat   = tf.transformations.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        obj_mat = np.dot(obj_trans_mat, obj_rot_mat)

        trans_mat = tf.transformations.translation_matrix(trans)
        rot_mat = tf.transformations.quaternion_matrix(rot)
        transform_mat = np.dot(trans_mat, rot_mat)            
        
        transformed_mat = np.dot(obj_mat, transform_mat)
        pos = tf.transformations.translation_from_matrix(transformed_mat) 
        ori = tf.transformations.quaternion_from_matrix(transformed_mat)
        

        transformed_pose = geometry_msgs.msg.Pose()
        transformed_pose.position.x = pos[0]
        transformed_pose.position.y = pos[1]
        transformed_pose.position.z = pos[2]
        transformed_pose.orientation.x = ori[0]
        transformed_pose.orientation.y = ori[1]
        transformed_pose.orientation.z = ori[2]
        transformed_pose.orientation.w = ori[3]
        return transformed_pose

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
        
        return

if __name__ == "__main__":
    rospy.init_node("test_single_arm_planner")
    planner_interface = SingleArmPlanner() 
    rospy.spin()   
    # rospy.loginfo("wait for joint states")
    # while(not rospy.is_shutdown() and not planner_interface.left_joint_states):
    #     continue    
    # rospy.loginfo("got the joint states")