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
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory, RobotState
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

from behaviors.behavior import BehaviorBase

def load_behavior(module_name, behavior, interface):
    behavior_module = importlib.import_module(module_name)
    behavior_class = getattr(behavior_module, behavior+'Behavior')
    behavior_instance = behavior_class(behavior.lower(), hardware_interface=interface)
    return behavior_instance

def search_behavior(dirname):
    behavior_list = []
    for file in os.listdir(dirname):
        if file.endswith(".py"):
            file_name = os.path.join(dirname, file)
            behavior_list.append((file_name.replace(dirname,'')).replace('.py',''))
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
        if rospy.has_param('robot_name'):
            self.robot_name  = rospy.get_param('robot_name')            

        if rospy.has_param('robot_hw'):
            hw_info = rospy.get_param('robot_hw')
            if hw_info == 'vrep':
                topic_robot_state = "/sim_interface/vrep_state"
            else:
                topic_robot_state = "/hw_interface/state"
        else:
            topic_robot_state = "/sim_interface/vrep_state"

        #
        rospy.Subscriber(topic_robot_state, Int32, self.callback_robot_state)
        self.pub_motion = rospy.Publisher('/move_group/display_planned_path',DisplayTrajectory ,queue_size=10)
        # arm controller action
        self.ac = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

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

    def callback_robot_state(self,state):
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

    def callback_set_behavior(self, req):
        rospy.loginfo("Getting behavior command.")
        res = SetBehaviorResponse()
        res.result = SetBehaviorResponse.ERROR

        planner_name = req.header.frame_id
        joint_trajectory = req.trajectory
        behavior_model = self.behavior_list.get(planner_name)
        
        #TODO: integrate manipulator, gripper, mobile control
        # gripper control
        if behavior_model.hardware == PlannerInputs.LEFT_GRIPPER or behavior_model.hardware == PlannerInputs.RIGHT_GRIPPER:
            self.current_behavior = planner_name
            behavior_model.reset_motion_ref(trajectory = joint_trajectory)      
          
            # wait few seconds until robot is moving because of delay
            cnt = 0  
            while(self.robot_state == "READY"):
                cnt += 1
                rospy.sleep(1)
                if cnt > 5:
                    break
            # wait until robot moving is stopped.
            rospy.loginfo("Wait until robot is stopped.")
            while(self.robot_state == "MOVING"):     
                rospy.sleep(1)
                if(self.robot_state == "READY"):       
                    res.result = SetBehaviorResponse.OK   
                    rospy.loginfo("Action is finished.")     

        # mobile control
        elif behavior_model.hardware == PlannerInputs.MOBILE_BASE:
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
            # connect to arm controller server
            self.ac.wait_for_server()
            goal = FollowJointTrajectoryActionGoal().goal
            goal.trajectory = joint_trajectory

            rospy.loginfo("Wait until robot is stopped.")
            self.ac.send_goal(goal)
            self.ac.wait_for_result()
            self.ac.get_result()
            rospy.loginfo("Action is finished.")
            res.result = SetBehaviorResponse.OK   
              
        return res

    def callback_get_motion(self, req):
        planner_name = req.planner_name
        behavior_model = self.behavior_list.get(planner_name)
        ret = behavior_model.get_motion(req.inputs)
        res = GetMotionResponse()
        if ret.planResult == MotionPlanResponse.SUCCESS:
            res.result = True
            res.motion.jointTrajectory = ret.jointTrajectory
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
    hw_if = 'vrep' # By default, vrep!
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
    srv_get_behavior_list = rospy.Service('~get_behavior_list',
                GetBehaviorList, 
                bm.callback_get_behavior_list)
    srv_set_behavior = rospy.Service('~set_behavior', 
                SetBehavior, 
                bm.callback_set_behavior)
    srv_get_requirements = rospy.Service('~get_motion', 
                GetMotion, 
                bm.callback_get_motion)
       
    # Start
    rospy.loginfo('[BehaviorManager: %s] Service Started!' % hw_if)

    loop_freq = 10 # 10hz
    r = rospy.Rate(loop_freq)

    while not rospy.is_shutdown():
        
        # bm update and get state
        behavior_state = bm.update()
        r.sleep()
