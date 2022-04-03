import abc
from abc import ABCMeta
from six import with_metaclass 
import rospy
import rosservice
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *

from socialrobot_msgs.msg import Behavior, Object, Position
from socialrobot_msgs.srv import *
from socialrobot_motion.srv import *
from socialrobot_behavior.msg import *
from socialrobot_behavior.srv import *
from socialrobot_hardware.srv import *
from sbpl_door_planner.msg import * #Door,MobilePoseTrajectory,MobileTwistTrajectory,Obstacle3D
from sbpl_door_planner.srv import * #plan_door_open_motion
import math
import numpy as np 
import tf 
import tf.transformations as tfm
import random
import utils
from behavior import BehaviorBase

class OpenDoorBehavior(BehaviorBase):
    def __init__(self, name, **params):
        # TODO: this behavior is based on SNU's sbpl_planner with robocare robot only
        super(OpenDoorBehavior, self).__init__(name, **params)        

        if self._hardware_if == 'vrep':
            self.service_name = '/sim_interface/set_motion'
            self.service_type = VrepSetJointTrajectory
        elif self._hardware_if == 'hw':
            self.service_name = '/hw_interface/set_motion'
            self.service_type = SetJointTrajectory

        # service
        self.planner_service = "/plan_door_open_motion"
        self.door_srv = rospy.ServiceProxy("/plan_door_open_motion", plan_door_open_motion)
        self.scene_srv = rospy.ServiceProxy("/motion_plan/update_scene_objects", UpdateObjects)

        # pubisher
        self.path_pub = rospy.Publisher('/mobile_path', Path, queue_size=10)
        self.pose_pub = rospy.Publisher('/mobile_pose', PoseArray, queue_size=10)
        
        self.input_args = ['robot_group',
                            'target_object',
                            'static_object',
                            'dynamic_object']
        self.hardware_group = ['arm','mobile']

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
        rospy.loginfo("Calculating door openning motion..")
        res = self._call_ros_service(inputs)
        return res

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True

    def _call_ros_service(self, requirements):
        req = plan_door_open_motionRequest()
        res = MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

        # joint states
        torso_states = sensor_msgs.msg.JointState()
        arm_states = sensor_msgs.msg.JointState()
        mobile_pose = geometry_msgs.msg.Pose2D()
        current_joint_states = requirements.current_position.joint_state
        joints = ["Waist_Roll", "Waist_Pitch", "RShoulder_Pitch",
                    "RShoulder_Roll", "RElbow_Pitch", "RElbow_Yaw",
                    "RWrist_Pitch", "RWrist_Roll"]
        torso_joint = joints[:2]
        right_arm_joint = joints[2:]

        #door information
        door = Door()    
        desired_door_angle = 120
        door_info = requirements.target_object[0]
        door_frame = [None, None]
        door_handle = None 
        door_center = None

        #transform doorinfo based on robot base
        door_info = utils.transform_object(door_info)        
        #self.add_objects_into_scene([door_info])

        for aff in door_info.affordance:
            if aff.id == 'center':
                door_center = aff
            elif aff.id == 'frame_p1':
                door_frame[0] = aff
            elif aff.id == 'frame_p2':
                door_frame[1] = aff
            elif aff.id == 'obj_fridge_handle':
                door_handle = aff

        # get current joint from topic
        try:
            torso_states.name = torso_joint
            torso_states.position = [current_joint_states.position[current_joint_states.name.index(j)] for j in torso_joint]
            torso_states.velocity = [current_joint_states.velocity[current_joint_states.name.index(j)] for j in torso_joint]
            torso_states.effort = [current_joint_states.effort[current_joint_states.name.index(j)] for j in torso_joint]

            arm_states.name = right_arm_joint
            arm_states.position = [current_joint_states.position[current_joint_states.name.index(j)] for j in right_arm_joint]
            arm_states.velocity = [current_joint_states.velocity[current_joint_states.name.index(j)] for j in right_arm_joint]
            arm_states.effort = [current_joint_states.effort[current_joint_states.name.index(j)] for j in right_arm_joint]
        except:
            # use temp values
            torso_init = [0,0]
            torso_states.position = [i*math.pi/180.0 for i in torso_init]
            torso_states.name = joints[:2]
            torso_states.velocity = [0.0, 0.0]
            torso_states.effort = [0.0, 0.0]
            arm_init = [0,0,0,0,0,0]
            arm_states.position = [i*math.pi/180.0 for i in arm_init]
            arm_states.name = joints[2:]
            arm_states.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            arm_states.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            rospy.logwarn("[OpenDoor Behavior] cannot get current robocare robot joint states.")

        # frame_id = base_footprint
        mobile_pose.x = 0.0
        mobile_pose.y = 0.0
        mobile_pose.theta = 0.0

        # door pose info based on base_footprint frame
        door.center.x = door_center.bb3d.center.position.x
        door.center.y = door_center.bb3d.center.position.y

        door.frame_p1.x = door_frame[0].bb3d.center.position.x
        door.frame_p1.y = door_frame[0].bb3d.center.position.y
        door.frame_p2.x = door_frame[1].bb3d.center.position.x
        door.frame_p2.y = door_frame[1].bb3d.center.position.y

        door.handle.x = door_handle.bb3d.center.position.x
        door.handle.y = door_handle.bb3d.center.position.y
        # try:        
        #     door.center.x = door_center.bb3d.center.position.x
        #     door.center.y = door_center.bb3d.center.position.y

        #     door.frame_p1.x = door_frame[0].bb3d.center.position.x
        #     door.frame_p1.y = door_frame[0].bb3d.center.position.y
        #     door.frame_p2.x = door_frame[1].bb3d.center.position.x
        #     door.frame_p2.y = door_frame[1].bb3d.center.position.y

        #     door.handle.x = door_handle.bb3d.center.position.x
        #     door.handle.y = door_handle.bb3d.center.position.y
            
        # except:      
        #     rospy.logwarn("[OpenDoor Behavior] cannot get door information.") 
        #     # use temp values  
        #     door.frame_p1.x = +6.1700e-01
        #     door.frame_p1.y = -5.3600e-01
        #     door.frame_p2.x = +6.1700e-01
        #     door.frame_p2.y = -3.6002e-02

        #     door.handle.x = +4.9700e-01
        #     door.handle.y = -8.6002e-02

        #     door.center.x = +5.1699e-01
        #     door.center.y = -2.8601e-01  

        door.door_p1.x = door.frame_p1.x
        door.door_p1.y = door.frame_p1.y
        door.door_p2.x = door.frame_p2.x
        door.door_p2.y = door.frame_p2.y
        door.handle_in.x = +1.6550e+00
        door.handle_in.y = +1.8050e+00
        door.travel_dir.x = 1.0
        door.travel_dir.y = 0.0
        door.travel_dir.z = 0.0
        door.rot_dir = Door().ROT_DIR_COUNTERCLOCKWISE
        door.hinge = Door().HINGE_P1
        door.header.frame_id = "base_footprint"
        if desired_door_angle:
            door.desired_door_angle = desired_door_angle/180.0*math.pi
        else:
            door.desired_door_angle = 120.0/180.0*math.pi

        req.current_mobile_state = mobile_pose
        req.current_right_joint_state = arm_states
        req.current_torso_joint_state = torso_states
        req.current_mobile_state = mobile_pose
        req.door_info = door
        req.interpolate_path.data = True
        #print(req)
        door_plan = self.door_srv(req)
        pt_num = len(door_plan.mobile_pose_trajectory[0].points)   

        if pt_num>0:
            res.planResult = MotionPlanResponse.SUCCESS
            path = Path()
            path.header.stamp = rospy.Time().now()
            path.header.frame_id = 'base_footprint'
            
            for i in range(pt_num):
                mobile_pose = door_plan.mobile_pose_trajectory[0].points[i]
                pose = PoseStamped()
                pose.pose.position.x = mobile_pose.x
                pose.pose.position.y = mobile_pose.y
                pose.pose.position.z = 0.0
                quaternion = tf.transformations.quaternion_from_euler(0, 0, mobile_pose.theta)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                path.poses.append(pose)
            res.planResult = MotionPlanResponse.SUCCESS
            res.pathTrajectory = path

            # set door status opened
            rospy.set_param("fridge_isopen", True)
        else:
            res.planResult = MotionPlanResponse.ERROR_NO_SOLUTION   
        return res

    def add_objects_into_scene(self, objects):
        req = UpdateObjectsRequest()
        req.command = UpdateObjectsRequest.ADD
        req.objects = objects
        res = self.scene_srv(req)

if __name__ == "__main__":
    rospy.init_node("sbpl_test")
    planner = OpenDoorBehavior('opendoor')
    req = Behavior()
    planner.get_motion(req)
    
