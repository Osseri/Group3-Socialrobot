#!/usr/bin/env python
import importlib
import roslib
roslib.load_manifest('socialrobot_hardware')

import os
import sys
from six import with_metaclass
import rospy
import rosparam
import rospkg
import actionlib

import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory, RobotState, ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from math import pi

from move_base_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_motion.srv import *
from socialrobot_hardware.msg import *
from socialrobot_hardware.srv import *
from socialrobot_behavior.msg import PlannerInputs
from socialrobot_behavior.srv import *
from socialrobot_perception_msgs.msg import Objects, Object

from behaviors.behavior import BehaviorBase


def load_behavior(module_name, behavior, interface):
    behavior_module = importlib.import_module(module_name)
    behavior_class = getattr(behavior_module, behavior + 'Behavior')
    behavior_instance = behavior_class(behavior.lower(), hardware_interface=interface)
    return behavior_instance


def search_behavior(dirname):
    behavior_list = []
    for file in os.listdir(dirname):
        if file.endswith(".py"):
            file_name = os.path.join(dirname, file)
            behavior_list.append((file_name.replace(dirname, '')).replace('.py', ''))
    return behavior_list


class Singleton(type):
    ''' a base class for singleton pattern '''
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class BehaviorManager(with_metaclass(Singleton)):
    def __init__(self):
        self.behavior_list = {}
        self.current_behavior = None
        self.robot_state = "READY"
        self.get_behavior_comm = False
        self.robot_name = 'skkurobot'
        self.detected_objects = []  #TODO: remove
        self.hw_info = rospy.get_param('robot_hw')
        if rospy.has_param('robot_name'):
            self.robot_name = rospy.get_param('robot_name')

        if rospy.has_param('robot_hw'):
            self.hw_info = rospy.get_param('robot_hw')
            if self.hw_info == 'vrep':
                topic_robot_state = "/sim_interface/vrep_state"
            else:
                topic_robot_state = "/hw_interface/state"
        else:
            topic_robot_state = "/sim_interface/vrep_state"

        #
        rospy.Subscriber(topic_robot_state, Int32, self.callback_robot_state)
        self.pub_motion = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)
        rospy.Subscriber('/perception/objects', Objects, self.callback_objects)

        # arm controller action
        self.ac = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.robocare_ac = actionlib.SimpleActionClient('/execute_trajectory', ExecuteTrajectoryAction)

        # service server
        rospy.Service('/social_robot/execute_action', ExecuteAction, self.callback_execute_action)

    def check_hardware(self):
        pass

    def check_controller(self):
        pass

    def add_behavior(self, **params):
        _planner_name = params.get('planner_name')
        _behavior_model = params.get('behavior')
        #print(_behavior_model)

        if _planner_name and _behavior_model:
            self.behavior_list[_planner_name] = _behavior_model

    def callback_objects(self, data):
        objects = []
        for obj in data.detected_objects:
            # print(obj)
            objects.append(obj)
        self.detected_objects = objects

        return

    def callback_execute_action(self, req):
        rospy.loginfo("Action Command is received.")
        res = ExecuteActionResponse()
        plan_req = GetMotionRequest()

        if req.action_name == 'approach':
            plan_req.planner_name = req.action_name
            # arm type
            if req.action_params[0] == 'left':
                plan_req.inputs.targetBody = plan_req.inputs.LEFT_ARM
            elif req.action_params[0] == 'right':
                plan_req.inputs.targetBody = plan_req.inputs.RIGHT_ARM
            # set approach direction
            plan_req.inputs.approachDirection = plan_req.inputs.APPROACH_SIDE

            # add dynamic objects
            plan_req.inputs.targetObject = [req.action_params[1]]
            for obj in self.detected_objects:
                if obj.name.data != 'obj_table':
                    #object name
                    plan_req.inputs.obstacle_ids.append(obj.name.data)
                    #object bounding box
                    plan_req.inputs.obstacles.append(obj.bb3d)

            # add static object(table)
            obs = BoundingBox3D()
            c = Pose()
            c.position.x = 0.550006
            c.position.y = 8.80659e-06
            c.position.z = 0.40
            c.orientation.x = 0
            c.orientation.y = 0
            c.orientation.z = 0.707
            c.orientation.w = 0.707
            obs.center = c
            v = Vector3()
            v.x = 1.1342161893844604
            v.y = 0.7088739275932312
            v.z = 0.80
            obs.size = v
            plan_req.inputs.obstacles.append(obs)
            plan_req.inputs.obstacle_ids.append('obj_table')

        elif req.action_name == 'movearm':
            # arm type
            if req.action_params[0] == 'left':
                plan_req.inputs.targetBody = plan_req.inputs.LEFT_ARM
            if req.action_params[0] == 'right':
                plan_req.inputs.targetBody = plan_req.inputs.RIGHT_ARM
            plan_req.planner_name = "movearm"
            # pose
            pose = req.params

            # add dynamic objects
            plan_req.inputs.targetObject = req.action_params
            for obj in self.detected_objects:
                if obj.name.data != 'obj_table':
                    #object name
                    plan_req.inputs.obstacle_ids.append(obj.name.data)
                    #object bounding box
                    plan_req.inputs.obstacles.append(obj.bb3d)

            if req.action_params[1] == 'cartesian':
                # cartesian space goal
                target_pose = Pose()
                target_pose.position.x = pose[0]
                target_pose.position.y = pose[1]
                target_pose.position.z = pose[2]
                target_pose.orientation.x = pose[3]
                target_pose.orientation.y = pose[4]
                target_pose.orientation.z = pose[5]
                target_pose.orientation.w = pose[6]
                plan_req.inputs.poseGoal = target_pose
            elif req.action_params[1] == 'joint':
                target_joint_state = JointState()
                target_joint_state.name = [
                    'Waist_Roll', 'Waist_Pitch', 'Shoulder_Pitch', 'Shoulder_Roll', 'Elbow_Pitch', 'Elbow_Yaw',
                    'Wrist_Pitch', 'Wrist_Roll'
                ]
                target_joint_state.position = pose
                plan_req.inputs.jointGoal = target_joint_state

        elif req.action_name == 'openclose':
            # pose template
            gripper_behavior = ''
            gripper_open = [0.0, 0.0]
            gripper_close = [1.0, 0.0]

            # arm type
            if req.action_params[0] == 'left':
                plan_req.inputs.targetBody = plan_req.inputs.LEFT_GRIPPER
            if req.action_params[0] == 'right':
                plan_req.inputs.targetBody = plan_req.inputs.RIGHT_GRIPPER

            plan_req.planner_name = "openclose"
            if req.action_params[1] == "open":
                gripper_behavior = gripper_open
            elif req.action_params[1] == "close":
                gripper_behavior = gripper_close
            gripper_box = [0.0, 0.0]
            gripper_circle = [0.0, 0.3]
            gripper_hook = [0.0, 1.0]

            # set grasp pose
            plan_req.inputs.graspPose = map(lambda x, y: x + y, gripper_behavior, gripper_box)

        #TODO-KIST: set grasp body pose

        # get motion
        motion_srv = rospy.ServiceProxy('/behavior/get_motion', GetMotion)
        motion_res = motion_srv(plan_req)

        # set behavior
        if motion_res.result:
            behavior_srv = rospy.ServiceProxy('/behavior/set_behavior', SetBehavior)
            behavior_req = SetBehaviorRequest()
            behavior_req.header.frame_id = plan_req.planner_name
            behavior_req.trajectory = motion_res.motion.jointTrajectory
            behavior_res = behavior_srv(behavior_req)

            res.result = 1
            return res
        else:
            res.result = 0
            return res

    def callback_robot_state(self, state):
        """
        check roobot moving state
        """

        if state.data == RobotState().READY_FOR_ACTION:
            self.robot_state = "READY"
        elif state.data == RobotState().ACTION_RUNNING:
            self.robot_state = "MOVING"

    def callback_get_behavior_list(self, req):
        res = GetBehaviorListResponse()
        res.behavior_list = map(str, self.behavior_list.keys())

        return res

    def separate_joints(self, trajectory):
        import copy
        traj = copy.copy(trajectory)
        traj.joint_names = traj.joint_names[2:]
        for i, pt in enumerate(traj.points):
            traj.points[i].positions = pt.positions[2:]
            traj.points[i].velocities = pt.velocities[2:]
            traj.points[i].accelerations = pt.accelerations[2:]

        return traj

    def callback_set_behavior(self, req):
        res = SetBehaviorResponse()
        res.result = SetBehaviorResponse.ERROR
        rospy.loginfo("[Socialrobot behavior] Checking behavior request")
        planner_name = req.header.frame_id
        joint_trajectory = req.trajectory
        path_trajectory = req.path
        behavior_model = self.behavior_list.get(planner_name)
        print(planner_name, behavior_model.hardware)

        #TODO: integrate manipulator, gripper, mobile control

        # gripper control
        if behavior_model.hardware == PlannerInputs.LEFT_GRIPPER or behavior_model.hardware == PlannerInputs.RIGHT_GRIPPER:
            rospy.loginfo("[Socialrobot behavior] Setting gripper behavior...")
            self.current_behavior = planner_name
            behavior_model.reset_motion_ref(trajectory=joint_trajectory)             
            res.result = self.wait_for_gripper()   

        # mobile control
        elif behavior_model.hardware == PlannerInputs.MOBILE_BASE:
            rospy.loginfo("[Socialrobot behavior] Setting mobile behavior...")
            # having mobile path trajectory
            if len(path_trajectory.poses)>0:
                path_srv = rospy.ServiceProxy('/set_path', SetPathTrajectory)
                path = SetPathTrajectoryRequest()
                path.trajectory = path_trajectory
                path_res = path_srv(path)

                
                res.result = SetBehaviorResponse.OK
                return res
            # having mobile goal pose
            else:
                client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
                client.wait_for_server()

                #send waypoints 
                for goal_pose in behavior_model.goal_pose:
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose = goal_pose

                    # send goal position to action server
                    client.send_goal(goal)
                    wait = client.wait_for_result()
                    if wait:
                        res.result = SetBehaviorResponse.OK

        # manipulator control
        elif behavior_model.hardware == PlannerInputs.LEFT_ARM or behavior_model.hardware == PlannerInputs.RIGHT_ARM:
            rospy.set_param('/social_robot/moveit_plan_state', True)
            rospy.loginfo("[Socialrobot behavior] Setting arm behavior...")
            # robocare robot
            if (self.robot_name == 'social_robot' and self.hw_info == 'hw'):
                self.robocare_ac.wait_for_server()
                # set trajectory goal
                goal = ExecuteTrajectoryGoal()
                goal.trajectory.joint_trajectory = joint_trajectory
                self.robocare_ac.send_goal(goal)
                self.robocare_ac.wait_for_result()
                self.robocare_ac.get_result()
                rospy.logwarn("Action finished.")
                res.result = self.wait_for_social_robot()
            else:
                # connect to arm controller server
                self.ac.wait_for_server()
                goal = FollowJointTrajectoryActionGoal().goal
                goal.trajectory = joint_trajectory
                self.ac.send_goal(goal)
                rospy.loginfo("Wait until robot is stopped.")
                self.ac.wait_for_result()
                # wait more seconds to update moveit scene
                wait_in_seconds = 3
                while wait_in_seconds:
                    rospy.loginfo("Wait %i secs." % wait_in_seconds)
                    rospy.sleep(1)
                    if self.robot_state != "READY":
                        break
                    wait_in_seconds -= 1
                self.ac.get_result()
                res.result = SetBehaviorResponse.OK
        rospy.loginfo("Action is finished.")
        return res

    def wait_for_social_robot(self):
        rospy.loginfo("Wait until robot moving is stopped.")
        # wait few seconds until robot is moving because of delay
        is_moving = True
        elapsed = 0
        while is_moving:
            is_moving = rospy.get_param('/social_robot/moveit_plan_state')
            rospy.loginfo("def wait_for_social_robot(). %i secs elapsed" % elapsed)
            rospy.sleep(1)
            elapsed += 1
        return SetBehaviorResponse.OK

    def wait_for_gripper(self):
        rospy.loginfo("Wait until robot gripper is stopped.")
        # wait few seconds until robot is moving because of delay
        wait_in_seconds = 3
        while wait_in_seconds:
            rospy.loginfo("[wait_for_gripper] Wait %i secs." % wait_in_seconds)
            rospy.sleep(1)
            if self.robot_state != "READY":
                break
            wait_in_seconds -= 1
        # moving start
        while (self.robot_state == "MOVING"):
            rospy.sleep(1)
            if (self.robot_state == "READY"):
                break
        return SetBehaviorResponse.OK

    def callback_get_motion(self, req):
        planner_name = req.planner_name
        behavior_model = self.behavior_list.get(planner_name)
        ret = behavior_model.get_motion(req.inputs)
        res = GetMotionResponse()
        if ret.planResult == MotionPlanResponse.SUCCESS:
            res.result = True
            res.motion.jointTrajectory = ret.jointTrajectory
            res.motion.pathTrajectory = ret.pathTrajectory
            behavior_model.hardware = req.inputs.targetBody
        else:
            res.result = False
        return res

    def update(self):
        behavior_model = self.behavior_list.get(self.current_behavior)

        if behavior_model:
            state = behavior_model.loop_until_done()

            if state == BehaviorBase.DONE_STATE:
                behavior_model.loop_until_done()
                self.current_behavior = None
                return 1
            elif state == BehaviorBase.ERROR_STATE:
                self.current_behavior = None
                return -1

        return 0


##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('behavior')

    # get joint information
    robot = moveit_commander.RobotCommander()
    robot.get_current_state()

    # hardware interface(vrep or hw)
    hw_if = 'vrep'  # By default, vrep!
    if rospy.has_param('robot_hw'):
        hw_if = rospy.get_param('robot_hw')

    # get behavior list
    rospack = rospkg.RosPack()
    behavior_dir = rospack.get_path('socialrobot_behavior') + '/script/behaviors/'
    behavior_list = search_behavior(behavior_dir)
    if 'behavior' in behavior_list:
        behavior_list.remove('behavior')
    if '__init__' in behavior_list:
        behavior_list.remove('__init__')

    # Behavior Manager
    bm = BehaviorManager()
    for behavior in behavior_list:
        module_name = 'behaviors.' + behavior
        bm.add_behavior(planner_name=behavior.lower(), behavior=load_behavior(module_name, behavior, hw_if))

    # ros service
    srv_get_behavior_list = rospy.Service('~get_behavior_list', GetBehaviorList, bm.callback_get_behavior_list)
    srv_set_behavior = rospy.Service('~set_behavior', SetBehavior, bm.callback_set_behavior)
    srv_get_requirements = rospy.Service('~get_motion', GetMotion, bm.callback_get_motion)

    # Start
    rospy.loginfo('[BehaviorManager: %s] Service Started!' % hw_if)

    loop_freq = 10  # 10hz
    r = rospy.Rate(loop_freq)

    while not rospy.is_shutdown():

        # bm update and get state
        behavior_state = bm.update()
        r.sleep()
