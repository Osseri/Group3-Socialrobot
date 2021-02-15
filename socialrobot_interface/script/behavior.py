#!/usr/bin/env python
import rospy
import rosparam
import tf
import moveit_commander
from std_msgs import msg as std_msg
from std_srvs import srv as std_srv
from sensor_msgs import msg as sensor_msg
from socialrobot_hardware import msg as hardware_msg
from socialrobot_behavior import srv as behavior_srv
from socialrobot_motion import srv as motion_srv
import interface


class BehaviorInterface(interface.InterfaceBase):
    def __init__(self):
        super(BehaviorInterface, self).__init__()
        """
        Module for Behavior Manager
        """
        rospy.loginfo("Initializing BehaviorInterface...")

        # get robot status
        self.left_group = moveit_commander.MoveGroupCommander("left_arm")
        self.right_group = moveit_commander.MoveGroupCommander("right_arm")
        self.left_eef_group = moveit_commander.MoveGroupCommander("left_eef")
        self.right_eef_group = moveit_commander.MoveGroupCommander("right_eef")
        self.base_group = []
        self.group = [self.right_group, self.left_group, self.base_group]

        # set the data label name for DB
        self.joint_state_name = "/behavior/jointState"

        # Subscribe data
        topic_robot_state = "/sim_interface/vrep_state"
        topic_joint_state = "/sim_interface/joint_states"
        topic_object_list = "/sim_interface/object_list"
        if rospy.has_param("robot_hw") and rospy.get_param("robot_hw") != "vrep":
            topic_robot_state = "/hw_interface/state"
            topic_joint_state = "/hw_interface/joint_states"
            topic_object_list = "/hw_interface/object_list"
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

    def clear_objects(self):
        rospy.loginfo("Clearing scene objects..." )
        reset = rospy.ServiceProxy("/motion_plan/reset", std_srv.Empty)
        reset()

    def get_objects(self):
        rospy.loginfo("Getting scene objects..." )
        

    def execute_action(self, action, motion):
        rospy.loginfo("Executing %s..." % action.name)

        proxy = rospy.ServiceProxy("/behavior/set_behavior", behavior_srv.SetBehavior)
        behavior_req = behavior_srv.SetBehaviorRequest()
        behavior_req.header.frame_id = action.planner[0]
        behavior_req.trajectory = motion.jointTrajectory
        behavior_req.path = motion.pathTrajectory

        # TODO: Check Bug ###############
        # 'proxy(behavior_req)' should be 'res = proxy(behavior_req)'
        res = behavior_srv.SetBehaviorResponse()
        proxy(behavior_req)
        return res.result == behavior_srv.SetBehaviorResponse.OK

    def get_motion(self, req):
        """
        Inpouts:    string planner_name
                    socialrobot_behavior/PlannerInputs inputs
        ---
        Outputs:    bool result
                    socialrobot_behavior/PlannerOutputs motion
        """
        proxy = rospy.ServiceProxy("/behavior/get_motion", behavior_srv.GetMotion)
        return proxy(req)

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
        # publish enf-effector joints, position and for context_manager
        if joint_state.name:
            for group in self.group:
                perception = std_msg.Float32MultiArray()
                perception.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0]
                if group:
                    joints = sensor_msg.JointState()
                    joints.header = joint_state.header
                    try:
                        if group.get_name() == "left_arm":
                            joint_names = self.left_eef_group.get_active_joints()
                            self._update_joints_info(joint_names, joint_state, joints)
                            (pos, ori) = self.listener.lookupTransform(
                                "/base_footprint",
                                "/left_end_effect_point",
                                rospy.Time(0),
                            )
                            perception.data = [pos[0], pos[1], pos[2], 0, 0, 0, 12.0]
                        elif group.get_name() == "right_arm":
                            joint_names = self.right_eef_group.get_active_joints()
                            self._update_joints_info(joint_names, joint_state, joints)
                            (pos, ori) = self.listener.lookupTransform(
                                "/base_footprint",
                                "/right_end_effect_point",
                                rospy.Time(0),
                            )
                            perception.data = [pos[0], pos[1], pos[2], 0, 0, 0, 11.0]
                    except (
                        tf.LookupException,
                        tf.ConnectivityException,
                        tf.ExtrapolationException,
                    ):
                        rospy.sleep(1)
                    self.pub_joint_state.publish(joints)
                self.pub_eef_tf.publish(perception)

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

        self.pub_robot_state.publish(robot_state)
