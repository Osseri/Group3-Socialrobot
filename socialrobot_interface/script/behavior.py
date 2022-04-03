#!/usr/bin/env python
import rospy
import rosparam
import tf
from std_msgs import msg as std_msg
from std_srvs import srv as std_srv
from sensor_msgs import msg as sensor_msg
from socialrobot_actionlib import msg as actionlib_msg
from socialrobot_msgs import msg as social_msg
from socialrobot_msgs import srv as social_srv
from socialrobot_hardware import msg as hardware_msg
from socialrobot_behavior import srv as behavior_srv
import interface


class BehaviorInterface(interface.InterfaceBase):
    def __init__(self):
        super(BehaviorInterface, self).__init__()
        """
        Module for Behavior Manager
        """
        rospy.loginfo("Initializing BehaviorInterface...")

        # get robot status
        self.base_group = None
        self.joint_states = None
        self.gripper_group = [None, None]
        self.gripper_status = [True, True]
        self.group = {}
        self.is_sim = False
        self.has_group_info = False

        # set the data label name for DB
        self.joint_state_name = "/behavior/jointState"

        # Subscribe data
        topic_robot_state = "/sim_interface/vrep_state"
        topic_joint_state = "/sim_interface/joint_states"
        if rospy.has_param("robot_hw") and rospy.get_param("robot_hw") != "vrep":
            self.is_sim = True
            topic_robot_state = "/hw_interface/state"
            topic_joint_state = "/hw_interface/joint_states"
        else:
            rospy.logwarn("Cannot find robot hardware param")

        # Subscriber
        self.listener = tf.TransformListener()
        
        rospy.Subscriber(
            topic_joint_state, sensor_msg.JointState, self._callback_joint_state
        )
        rospy.Subscriber(topic_robot_state, std_msg.Int32, self._callback_robot_state)

        # Publisher
        self.pub_joint_state = rospy.Publisher(
            "/joint_state", sensor_msg.JointState, queue_size=10
        )
        self.pub_robot_state = rospy.Publisher(
            "~behavior/robot_state", hardware_msg.RobotState, queue_size=10
        )
        self.pub_eef_tf = rospy.Publisher(
            "/visual_robot_perception", std_msg.Float32MultiArray, queue_size=10
        )

        # Service
        self.srv_group_info = rospy.ServiceProxy("/motion_plan/get_group_info", social_srv.GetGroups)
        self.request_group_info()


    def clear_objects(self):
        rospy.loginfo("Clearing scene objects..." )
        reset = rospy.ServiceProxy("/motion_plan/reset", std_srv.Empty)
        reset()

    def get_objects(self):
        rospy.loginfo("Getting scene objects..." )     

    def request_group_info(self):
        req = social_srv.GetGroupsRequest()
        group_list = ['left_arm', 'right_arm', 'left_eef', 'right_eef']
        req.group_name = group_list
        rospy.wait_for_service("/motion_plan/get_group_info", timeout=10.0)
        res = self.srv_group_info(req)
        
        for group in res.groups:
            self.group[group.name] = group.active_joints
        self.has_group_info = True

    def execute_action(self, action, motion):
        rospy.loginfo("Executing %s..." % action.name)
        # check gripper status
        status = True
        if action.name == 'open_hand' or action.name == 'release_object':
            status = True
        elif action.name == 'close_hand' or action.name == 'grasp_object':
            status = False
        if motion.jointTrajectory.joint_names == ['LFinger_1', 'LFinger_2', 'LFinger_3']:
            self.gripper_status[1] = status
        elif motion.jointTrajectory.joint_names == ['RFinger_1', 'RFinger_2', 'RFinger_3']:
            self.gripper_status[0] = status
        elif motion.jointTrajectory.joint_names == ['RFinger_1', 'RFinger_2', 'RFinger_3', 'LFinger_1', 'LFinger_2', 'LFinger_3']:
            self.gripper_status[0] = status
            self.gripper_status[1] = status

        proxy = rospy.ServiceProxy("/behavior/set_behavior", behavior_srv.SetBehavior)
        behavior_req = behavior_srv.SetBehaviorRequest()
        behavior_req.behavior_name = action.planner[0]
        behavior_req.trajectory = motion.jointTrajectory
        behavior_req.path = motion.pathTrajectory

        res = proxy(behavior_req)
        return res.result == behavior_srv.SetBehaviorResponse.OK

    def get_motion(self, requirements):
        """
        Inpouts:    string planner_name
                    socialrobot_behavior/PlannerInputs inputs
                    socialrobot_msgs/Behavior requirements
        ---
        Outputs:    bool result
                    socialrobot_behavior/PlannerOutputs motion
        """
        print('requesting motion trajectory...')
        req = behavior_srv.GetMotionRequest()
        req.requirements = requirements
        proxy = rospy.ServiceProxy("/behavior/get_motion", behavior_srv.GetMotion)
        return proxy(req)

    def get_requirements(self, behavior_name):
        """
        Inpouts:    string planner_name
        ---
        Outputs:    bool result
                    string requirements
        """
        req = behavior_srv.GetRequirementsRequest()
        req.behavior_name = behavior_name
        proxy = rospy.ServiceProxy("/behavior/get_requirements", behavior_srv.GetRequirements)
        return proxy(req)

    def set_detect_pose(self, objects):
        '''
        for robocare robot
        set robot's pose for detecting
        '''
        plan_req = behavior_srv.GetMotionRequest()
        plan_req.requirements.name = "movearm"

        # arm type
        plan_req.requirements.robot_group = [plan_req.requirements.BOTH_ARM]

        # goal position
        goal = social_msg.Position()
        goal.joint_state.name = [
            'Waist_Roll', 'Waist_Pitch', 
            'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll',
            'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll'
        ]

        goal.joint_state.position = [0, 0.48, 
                                    0.5, -1.5, 0.0, 0.0, -1.5224, 0.0, 
                                    0.5, 1.5, 0.0, 0.0, -1.5224, 0.0]
        plan_req.requirements.goal_position.append(goal)
        
        # add obstacles from topic
        plan_req.requirements.dynamic_object = objects
        print(plan_req.requirements)
        # execute standby pose
        motion_plan = self.get_motion(plan_req.requirements)
        if motion_plan.result == True:
            action = actionlib_msg.Action(name='move_arm',planner=['movearm'])
            self.execute_action(action, motion_plan.motion)
            return True
        else:
            return False

    def check_gripper_opened(self, gripper_group):
        #TODO: in VREP simulation, joint state publishing is too slow
        if gripper_group == 'left':
            if not self.is_sim:
                gripper_pos = self.gripper_group[1]
                if gripper_pos[0]<-0.34 and gripper_pos[1]<-0.34 and gripper_pos[2]<-0.34:  #Left is opened
                    return True
            return self.gripper_status[1]
        elif gripper_group == 'right':
            if not self.is_sim:
                gripper_pos = self.gripper_group[0]
                if gripper_pos[0]<-0.34 and gripper_pos[1]<-0.34 and gripper_pos[2]>0.34:  #Right is opened
                    return True
            return self.gripper_status[0]
        else:
            return False # invalid input error


    def _update_joints_info(self, joint_names, joint_state, joints):
        for joint in joint_names:
            if joint in joint_state.name:
                joints.name.append(joint)
                idx = joint_state.name.index(joint)
                joints.position.append(joint_state.position[idx])
                joints.velocity.append(joint_state.position[idx])
                joints.effort.append(joint_state.position[idx])

    def _callback_joint_state(self, joint_state):
        """
        Transfer robot poses for knowledge
        """
        self.joint_states = joint_state
        if self.has_group_info:
            # publish enf-effector joints and position for context_manager
            if joint_state.name:
                for group in self.group.keys():
                    perception = std_msg.Float32MultiArray()
                    perception.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0]
                    if group:
                        joints = sensor_msg.JointState()
                        joints.header = joint_state.header
                        try:
                            if group == "left_arm":
                                joint_names = self.group['left_eef']
                                self._update_joints_info(joint_names, joint_state, joints)
                                (pos, ori) = self.listener.lookupTransform(
                                    "/base_footprint",
                                    "/left_end_effect_point",
                                    rospy.Time(0),
                                )
                                self.gripper_group[1] = joints.position
                                perception.data = [pos[0], pos[1], pos[2], 0, 0, 0, 12.0]
                            elif group == "right_arm":
                                joint_names = self.group['right_eef']
                                self._update_joints_info(joint_names, joint_state, joints)
                                (pos, ori) = self.listener.lookupTransform(
                                    "/base_footprint",
                                    "/right_end_effect_point",
                                    rospy.Time(0),
                                )
                                self.gripper_group[0] = joints.position
                                perception.data = [pos[0], pos[1], pos[2], 0, 0, 0, 11.0]
                        except (
                            tf.LookupException,
                            tf.ConnectivityException,
                            tf.ExtrapolationException,
                        ):
                            rospy.sleep(1)
                        try:
                            self.pub_joint_state.publish(joints)
                        except:
                            pass
                    try:
                        self.pub_eef_tf.publish(perception)
                    except:
                        pass

    def _callback_robot_state(self, state):
        """
        Transfer Robot state
        """
        robot_state = hardware_msg.RobotState()
        robot_state.state = state.data

        if state.data == hardware_msg.RobotState.READY_FOR_ACTION:
            rosparam.set_param("/robot_state", "READY")
        elif state.data == hardware_msg.RobotState.ACTION_RUNNING:
            rosparam.set_param("/robot_state", "RUNNING")
        try:
            self.pub_robot_state.publish(robot_state)
        except:
            pass