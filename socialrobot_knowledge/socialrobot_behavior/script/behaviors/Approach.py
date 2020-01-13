import abc
from abc import ABCMeta
from six import with_metaclass 
import math
import rospy
import rosservice
import rosparam

from socialrobot_motion.srv import *
from socialrobot_behavior.srv import *
from socialrobot_hardware.srv import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import Pose

import tf.transformations as tfm
import numpy as np 
import random
from behavior import BehaviorBase

class ApproachBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(ApproachBehavior, self).__init__(name, **params)
        
        if self._hardware_if == 'vrep':
            self.service_name = '/sim_interface/set_motion'
            self.service_type = VrepSetJointTrajectory
        elif self._hardware_if == 'hw':
            self.service_name = '/hw_interface/set_motion'
            self.service_type = None

        # initial pose 
        self.gripper_pose = {} 

        self.rot_x = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (1,0,0))
        self.rot_y = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0,1,0))
        self.rot_z = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0,0,1))

        # get hardware 
        robot_name = rosparam.get_param("/robot_name")

        # set gripper's end effector pose
        # skkurobot gripper
        self.barrett_left_pose = {'init': np.array([0.5, 0.5, 0.5, 0.5]),
		    'top': tfm.quaternion_multiply(self.rot_x(90), np.array([0.5, 0.5, 0.5, 0.5])),
		    'side': np.array([0.707, 0.0, 0.0, 0.707]),
		    'front': np.array([0.5, 0.5, 0.5, 0.5])
            }       
        self.barrett_right_pose = {'init': np.array([0.0, 0.707, -0.707, 0]),
            'top': tfm.quaternion_multiply(self.rot_x(-90), tfm.quaternion_multiply(self.rot_y(-90), np.array([0.0, 0.707, -0.707, 0]))),	
		    'side': np.array([-0.707, 0.0, 0.0, 0.707]),
		    'front': np.array([0.5, -0.5, 0.5, -0.5])
            }

        # robocare gripper
        self.robocare_left_pose = {'init': np.array([0, 0, 0, 1]),
		    'top': tfm.quaternion_multiply(self.rot_z(-90), tfm.quaternion_multiply(self.rot_y(-90), tfm.quaternion_multiply(self.rot_z(-90), np.array([0, 0, 0, 1])))),
		    'side': tfm.quaternion_multiply(self.rot_x(90), tfm.quaternion_multiply(self.rot_z(-90), np.array([0, 0, 0, 1]))),
		    'front': tfm.quaternion_multiply(self.rot_x(90), tfm.quaternion_multiply(self.rot_z(-90), np.array([0, 0, 0, 1])))
            }       
        self.robocare_right_pose = {'init': np.array([0, 0, 0, 1]),
		    'top': tfm.quaternion_multiply(self.rot_z(90), tfm.quaternion_multiply(self.rot_y(90), tfm.quaternion_multiply(self.rot_z(90), np.array([0, 0, 0, 1])))),
		    'side': tfm.quaternion_multiply(self.rot_x(90), tfm.quaternion_multiply(self.rot_z(90), np.array([0, 0, 0, 1]))),
		    'front': tfm.quaternion_multiply(self.rot_x(90), tfm.quaternion_multiply(self.rot_z(90), np.array([0, 0, 0, 1])))
            }  

        if robot_name == 'skkurobot':
            self.gripper_pose = {'left': self.barrett_left_pose,
                                'right': self.barrett_right_pose,
                                'offset': 0.085, # between the wrist and object center
                                'rot': self.rot_z # sample rotation axis
                                }  
        elif robot_name == 'social_robot':
            self.gripper_pose =  {'left': self.robocare_left_pose,
                                'right': self.robocare_right_pose,
                                'offset': 0.13, # between the wrist and object center
                                'rot': self.rot_y # sample rotation axis
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
            
        return -1

    def get_motion(self, inputs):
        '''
        return the trajectory 
        '''
        rospy.loginfo("Calculating approaching motion..")

        ret , srv_response = self._call_ros_service(inputs)
        if ret == MotionPlanResponse.SUCCESS:
            rospy.loginfo("Approach planning is done.")
            self.motion_trajectory = srv_response.jointTrajectory
        else:
            rospy.loginfo("Approach planning if failed.")
        return ret        

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

        # service_name: at this time, it is hard coded.
        armplan_srv = '/motion_plan/move_arm'
  
        try:
            # call the ros service
            plan_arm = rospy.ServiceProxy(armplan_srv, MotionPlan)
            plan_req = MotionPlanRequest()
            
            # target body
            body_type = inputs.targetBody
            if body_type != MotionPlanRequest.LEFT_ARM and body_type != MotionPlanRequest.RIGHT_ARM:
                return (MotionPlanResponse.ERROR_INPUT, None)
            plan_req.targetBody = body_type
            
            # obstacles
            plan_req.obstacle_ids = inputs.obstacle_ids
            plan_req.obstacles = inputs.obstacles

            # target object
            if not inputs.targetObject:
                return (MotionPlanResponse.ERROR_FAIL, None)
            target_object = inputs.targetObject            
            boundingbox = inputs.obstacles[0]
            
            # arm planning
            result = None
            
            # start state
            start_state = inputs.currentJointState
            if start_state is None:
                return (MotionPlanResponse.ERROR_INPUT, None)
            plan_req.currentJointState = start_state

            # approach direction
            approach_direction = inputs.approachDirection

            # target object
            obj_center = boundingbox.center
            obj_size = boundingbox.size

            # build sample poses
            sample_top = sample_side = sample_front = sample_back = []
            gripper_pose = {}

            if body_type is MotionPlanRequest.LEFT_ARM:
                gripper_pose = self.gripper_pose['left']
            elif body_type is MotionPlanRequest.RIGHT_ARM:
                gripper_pose = self.gripper_pose['right']

            if approach_direction == 0 or approach_direction == 1: # both or top
                sample_top = [(q, (0,0,obj_size.z/2 + self.gripper_pose['offset']), tfm.quaternion_multiply(self.gripper_pose['rot'](q), gripper_pose['top'])) for q in self.get_sample_degree(1,2)]
            
            if approach_direction == 0 or approach_direction == 2: # both or side
                
                sample_side = [(q, (-1*obj_size.x/2 - self.gripper_pose['offset'],0,0), tfm.quaternion_multiply(self.gripper_pose['rot'](q), gripper_pose['front'])) for q in self.get_sample_degree(1,2)]
                        
            sample = sample_top + sample_side

            for i,sample_pose in enumerate(sample):
                print "Calculating approach sample angle ", sample_pose[0],i,"/",len(sample)

                # pose
                q = sample_pose[0] / 180 * math.pi
                trans = sample_pose[1]
                rot = sample_pose[2]                

                # sample pose to pose msg
                pose = Pose()
                pose.orientation.x = rot[0]
                pose.orientation.y = rot[1]
                pose.orientation.z = rot[2]
                pose.orientation.w = rot[3]
                pose.position.x = obj_center.position.x + trans[0]*math.cos(q) - trans[1]*math.sin(q)
                pose.position.y = obj_center.position.y + trans[0]*math.sin(q) + trans[1]*math.cos(q)
                pose.position.z = obj_center.position.z + trans[2]

                # target pose
                plan_req.targetPose = pose
                plan_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL

                plan_res = plan_arm(plan_req)
                if plan_res.planResult == MotionPlanResponse.SUCCESS:
                    result = plan_res
                    rospy.loginfo('approach pose and arm motion found!')
                    return (MotionPlanResponse.SUCCESS, result)

            return (MotionPlanResponse.ERROR_NO_SOLUTION, None)

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            return (MotionPlanResponse.ERROR_FAIL, None)