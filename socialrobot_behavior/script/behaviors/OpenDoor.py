import abc
from abc import ABCMeta
from six import with_metaclass 
import rospy
import rosservice

from socialrobot_motion.srv import *
from socialrobot_behavior.msg import *
from socialrobot_behavior.srv import *
from socialrobot_hardware.srv import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from sbpl_door_planner.msg import * #Door,MobilePoseTrajectory,MobileTwistTrajectory,Obstacle3D
from sbpl_door_planner.srv import * #plan_door_open_motion
import math
import numpy as np 
import tf 
import tf.transformations as tfm
import random
from behavior import BehaviorBase

class OpenDoorBehavior(BehaviorBase):
    def __init__(self, name, **params):
        # TODO: this behavior is based on SNU's sbpl_planner with robocare robot only
        super(OpenDoorBehavior, self).__init__(name, **params)
        
        # service
        self.planner_service = "/plan_door_open_motion"
        self.door_srv = rospy.ServiceProxy("/plan_door_open_motion", plan_door_open_motion)

        # pubisher
        self.path_pub = rospy.Publisher('/mobile_path', Path, queue_size=10)
        self.pose_pub = rospy.Publisher('/mobile_pose', PoseArray, queue_size=10)
        

    def check_requirement(self):
        rospy.loginfo('checking...%s' % self._name)
        self.service_list = rosservice.get_service_list()
        if self.planner_service in self.service_list:
            return True
        rospy.llogerr('cannot find %s service in the list.' % self.planner_service)
        return False

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
        if res.planResult == MotionPlanResponse.SUCCESS:
            rospy.loginfo("Door planning is done.")
        else:
            rospy.loginfo("Door planning is failed.")
        return res

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True

    def _call_ros_service(self, inputs):
        req = plan_door_open_motionRequest()        
        res = MotionPlanResponse()

        desired_door_angle = inputs.desired_door_angle
        door_info = inputs.door_info
        current_joint_states = inputs.currentJointState
        joints = ["Waist_Roll", "Waist_Pitch", "RShoulder_Pitch",
            "RShoulder_Roll", "RElbow_Pitch", "RElbow_Yaw",
            "RWrist_Pitch", "RWrist_Roll"]
        torso_joint = joints[:2]
        right_arm_joint = joints[2:]

        # set variables
        torso_service = sensor_msgs.msg.JointState()
        arm_service = sensor_msgs.msg.JointState()
        mobile_service = geometry_msgs.msg.Pose2D()
        door = Door()    

        # get current joint from topic
        try:
            torso_service.name = torso_joint
            torso_service.position = [current_joint_states.position[current_joint_states.name.index(j)] for j in torso_joint]
            torso_service.velocity = [current_joint_states.velocity[current_joint_states.name.index(j)] for j in torso_joint]
            torso_service.effort = [current_joint_states.effort[current_joint_states.name.index(j)] for j in torso_joint]

            arm_service.name = right_arm_joint
            arm_service.position = [current_joint_states.position[current_joint_states.name.index(j)] for j in right_arm_joint]
            arm_service.velocity = [current_joint_states.velocity[current_joint_states.name.index(j)] for j in right_arm_joint]
            arm_service.effort = [current_joint_states.effort[current_joint_states.name.index(j)] for j in right_arm_joint]
        except:
            # use temp values
            torso_init = [0,0]
            torso_service.position = [i*math.pi/180.0 for i in torso_init]
            torso_service.name = joints[:2]
            torso_service.velocity = [0.0, 0.0]
            torso_service.effort = [0.0, 0.0]
            arm_init = [0,0,0,0,0,0]
            arm_service.position = [i*math.pi/180.0 for i in arm_init]
            arm_service.name = joints[2:]
            arm_service.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            arm_service.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            rospy.logwarn("[OpenDoor Behavior] cannot get current robocare robot joint states.")

        # frame_id = base_footprint
        mobile_service.x = 0.0
        mobile_service.y = 0.0
        mobile_service.theta = 0.0

        # door pose info based on base_footprint frame
        try:
            # transform from base_footprint to QR marker
            base_to_qr = np.dot(tf.transformations.translation_matrix(door_info[:3]), 
                            tf.transformations.quaternion_matrix(door_info[3:]))

            qr_to_center = np.dot(tf.transformations.translation_matrix([0.2, -1.1498e-01, +1.2977e-02]), 
                                    tf.transformations.quaternion_matrix([-0.5, -0.5, -0.5, 0.5]))
            base_to_door = np.dot(base_to_qr, qr_to_center)

            door_to_right = np.dot(tf.transformations.translation_matrix([-1.0001e-01, +2.4999e-01, 0]), 
                                    tf.transformations.quaternion_matrix([0, 0, 0, 1]))

            door_to_left = np.dot(tf.transformations.translation_matrix([-1.0001e-01, -2.5001e-01, 0]), 
                                    tf.transformations.quaternion_matrix([0, 0, 0, 1]))

            door_to_handle = np.dot(tf.transformations.translation_matrix([+1.9991e-02, -2.0001e-01, 0]), 
                                    tf.transformations.quaternion_matrix([0, 0, 0, 1]))
            
            door.center.x = tf.transformations.translation_from_matrix(base_to_door)[0]
            door.center.y = tf.transformations.translation_from_matrix(base_to_door)[1]

            door.frame_p1.x = tf.transformations.translation_from_matrix(np.dot(base_to_door, door_to_right))[0]
            door.frame_p1.y = tf.transformations.translation_from_matrix(np.dot(base_to_door, door_to_right))[1]
            door.frame_p2.x = tf.transformations.translation_from_matrix(np.dot(base_to_door, door_to_left))[0]
            door.frame_p2.y = tf.transformations.translation_from_matrix(np.dot(base_to_door, door_to_left))[1]

            door.handle.x = tf.transformations.translation_from_matrix(np.dot(base_to_door, door_to_handle))[0]
            door.handle.y = tf.transformations.translation_from_matrix(np.dot(base_to_door, door_to_handle))[1]
            
        except:      
            # use temp values  
            door.frame_p1.x = +6.1500e-01
            door.frame_p1.y = -5.1400e-01
            door.frame_p2.x = +6.1500e-01
            door.frame_p2.y = -1.4001e-02

            door.handle.x = +4.9500e-01
            door.handle.y = -6.4001e-02

            door.handle_in.x = +1.6550e+00
            door.handle_in.y = +1.8050e+00

            door.center.x = +5.7000e-01
            door.center.y = -2.6400e-01
            rospy.logwarn("[OpenDoor Behavior] cannot get door information.")   

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

        req.current_mobile_state = mobile_service
        req.current_right_joint_state = arm_service
        req.current_torso_joint_state = torso_service
        req.current_mobile_state = mobile_service
        req.door_info = door
        req.interpolate_path.data = True
        
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
            res.pathTrajectory = path
        else:
            res.planResult = MotionPlanResponse.ERROR_NO_SOLUTION   
        return res

if __name__ == "__main__":
    rospy.init_node("sbpl_test")
    planner = OpenDoorBehavior('opendoor')
    req = PlannerInputs()
    planner.get_motion(req)
    
