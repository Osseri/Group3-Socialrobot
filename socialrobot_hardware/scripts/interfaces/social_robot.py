import abc
from abc import ABCMeta
from six import with_metaclass 

import math
import rospy
import rosservice
import rosparam
import actionlib

from social_robot_arm_msgs.msg import *
from social_robot_arm_msgs.srv import *
from socialrobot_hardware.srv import *
import socialrobot_interface.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from moveit_msgs.msg import *
from interface import InterfaceBase
'''
manipulator states:
         "\"IS_MOVING\""
          "\"STOPPED\""        
          "\"ACTUATOR_ENABLED\""        
          "\"ACTUATOR_DISABLED\""
'''
ACTION_READY = 0
ACTION_BUSY = 1

RIGHT_ARM = 0
LEFT_ARM = 1
RIGHT_HAND = 2
LEFT_HAND = 3

class social_robotInterface(InterfaceBase):
    
    def __init__(self, robot_name, **params):
        super(social_robotInterface, self).__init__(robot_name, **params)
        
        # subscriber for social_robot
        rospy.Subscriber("/social_robot/left_arm/states", ManipulatorState, self.callback_left_arm_state, queue_size=10)
        rospy.Subscriber("/social_robot/right_arm/states", ManipulatorState, self.callback_right_arm_state, queue_size=10)
        rospy.Subscriber("/social_robot/states", ManipulatorState, self.callback_robot_state, queue_size=10)
        rospy.Subscriber("/social_robot/joint_states", JointState, self.callback_joint_states)
        
        # publisher
        self.pub_joints = rospy.Publisher("/hw_interface/joint_states", JointState, queue_size=10)
        self.pub_state = rospy.Publisher("/hw_interface/state", Int32, queue_size=10)

        #
        self.srv_get_state = rospy.ServiceProxy('/social_robot/get_robot_state', GetRobotState)

        # action
        self.action_client  = actionlib.SimpleActionClient('/execute_trajectory', ExecuteTrajectoryAction)

        self.left_arm_state = ACTION_READY
        self.right_arm_state = ACTION_READY
        self.hand_state = ACTION_READY
        self.robot_state = ACTION_READY
        self.hand_stop_time = 0

        self.trajectory = None
        self.goal_recieved = False
        return

    def get_state(self):
        # check hand moving state
        now = rospy.get_time()
        if now < self.hand_stop_time:
            self.hand_state = ACTION_BUSY
        else:
            self.hand_state = ACTION_READY

        if self.left_arm_state == ACTION_BUSY or self.right_arm_state == ACTION_BUSY or self.hand_state == ACTION_BUSY:
            self.action_state = socialrobot_interface.msg.RobotState().ACTION_RUNNING
        else:
            self.action_state = socialrobot_interface.msg.RobotState().READY_FOR_ACTION        

        self.pub_state.publish(self.action_state)
        return self.action_state

    def set_motion(self, req):     
        res = SetJointTrajectoryResponse()
        robot_part = 0
        trajectory = req.trajectory
        self.joint_names = trajectory.joint_names
  
        # set robot part
        if self.joint_names == ['RFinger_1', 'RFinger_2', 'RFinger_3']:
            robot_part = RIGHT_HAND
            self.planning_group = 'right_arm'
        elif self.joint_names == ['LFinger_1', 'LFinger_2', 'LFinger_3']:
            robot_part = LEFT_HAND
            self.planning_group = 'left_arm'
        elif self.joint_names == ['RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll']:
            robot_part = RIGHT_ARM
            self.planning_group = 'right_arm'
        elif self.joint_names == ['LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll']:
            robot_part = LEFT_ARM
            self.planning_group = 'left_arm'
        else:
            rospy.logerr('[hw_interface] undefined robot part')
            res.result = SetJointTrajectoryResponse.ERROR

        # set ROS service
        if robot_part == RIGHT_HAND or robot_part == LEFT_HAND:
            self.srv_name = "/social_robot/goal_tool_control"
        elif robot_part == RIGHT_ARM or robot_part == LEFT_ARM:
           self.srv_name = "/social_robot/goal_joint_space_path"
        self.joint_srv = rospy.ServiceProxy(self.srv_name, SetJointPosition)

        # set joint trajectory
        if self.action_state == self.ACTION_READY:
            res.result = SetJointTrajectoryResponse.OK
            self.desired_trajectory['duration'] = req.duration
            self.desired_trajectory['trajectory'] = req.trajectory
            self.action_state = ACTION_BUSY
            self.action_start_time = 0
            self.current_time = 0
        else:
            res.result = SetJointTrajectoryResponse.ERROR

        length = len(req.trajectory.points)
        duration = req.duration
        
        if robot_part == RIGHT_HAND or robot_part == LEFT_HAND:
            for points in req.trajectory.points:
                srv = SetJointPositionRequest()
                srv.planning_group = self.planning_group
                srv.joint_position.position = points.positions
                srv.joint_position.joint_name = self.joint_names
                srv.path_time = 1.0

                # set moving stop time
                now = rospy.get_time()
                self.hand_stop_time = now + srv.path_time

                # send joint position
                self.joint_srv(srv)
                rospy.sleep(1)

        elif robot_part == RIGHT_ARM or robot_part == LEFT_ARM:
            goal = ExecuteTrajectoryGoal()
            goal.trajectory.joint_trajectory = req.trajectory

            # send joint trajectory
            self.action_client.wait_for_server()
            self.action_client.send_goal(goal)

        else:
            res.result = SetJointTrajectoryResponse.ERROR    

        res.result = SetJointTrajectoryResponse.OK

        return res

    def wait_robot_moving(self):
        i=0
        while(self.robot_state == 'READY'):
            i+=1
            if i>5:
                break
            rospy.sleep(1)
            rospy.logwarn('waiting MOVING')
        i=0
        while(self.robot_state == 'MOVING'):
            i+=1
            if i>30:
                break
            rospy.sleep(1)
            rospy.logerr('waiting STOPPED')
        return 

    def set_target_joint_positions(self, **joint_goal):
        '''
        set joint target position
        '''        
        if joint_goal:
            req = SetJointPositionRequest()        
            req.planning_group = self.planning_group
            req.joint_position.joint_name = self.joint_names
            req.path_time = 1.0
            for joint in self.joint_names:
                req.joint_position.position.append(joint_goal[joint]) 
            res = self.joint_srv(req)

        return

    def callback_left_arm_state(self, state):
        if state.manipulator_moving_state == "\"IS_MOVING\"":
            self.left_arm_state = ACTION_BUSY
        elif state.manipulator_moving_state == "\"STOPPED\"":
            self.left_arm_state = ACTION_READY
            
    def callback_right_arm_state(self, state):
        if state.manipulator_moving_state == "\"IS_MOVING\"":
            self.right_arm_state = ACTION_BUSY
        elif state.manipulator_moving_state == "\"STOPPED\"":
            self.right_arm_state = ACTION_READY

    def callback_robot_state(self, state):
        if state.manipulator_moving_state == "READY":
            self.robot_state = ACTION_READY
        elif state.manipulator_moving_state == "STOPPED":
            self.robot_state = ACTION_READY
        elif state.manipulator_moving_state == "MOVING":
            self.robot_state = ACTION_BUSY
            
        self.pub_state.publish(self.robot_state)       

    def callback_joint_states(self, joints):
        # re-publish joint states
        self.pub_joints.publish(joints)

    def update(self):

        return 
