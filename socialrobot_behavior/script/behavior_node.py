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
from socialrobot_actionlib.srv import *
from socialrobot_actionlib.msg import *

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
        self.hw_info = rospy.get_param('robot_hw', default="social_robot")
        if rospy.has_param('robot_name'):
            self.robot_name = rospy.get_param('robot_name')

        # publisher
        self.status_pub = rospy.Publisher("/socialrobot/behavior/status", String, queue_size=10)

        # controller action for vrep simulator
        self.arm_ac = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.gripper_ac = actionlib.SimpleActionClient('gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.mobile_ac = actionlib.SimpleActionClient('mobile_controller/follow_path_trajectory', FollowPathTrajectoryAction)

    def check_hardware(self):
        pass

    def check_controller(self):
        pass

    def set_behavior_requirements(self, action):
        '''
        action [socialrobot_actionlib/Action]
        '''
        behavior_name = action.name.replace('_','').lower()
        self.behavior_list[behavior_name]['group'] = action.group
        self.behavior_list[behavior_name]['planner'] = action.planner
        #self.behavior_list[behavior_name]['parameters'] = [x.replace('?','') for x in action.parameters]

    def add_behavior(self, **params):
        _planner_name = params.get('planner_name')
        _behavior_model = params.get('behavior')

        if _planner_name and _behavior_model:
            self.behavior_list[_planner_name] = {'parameters':None, 
                                                 'module':None, 
                                                 'group':None, 
                                                 'planner':None}
            self.behavior_list[_planner_name]['module'] = _behavior_model

    def callback_get_behavior_list(self, req):
        res = GetBehaviorListResponse()
        res.behavior_list = map(str, self.behavior_list.keys())

        return res

    def callback_set_behavior(self, req):
        res = SetBehaviorResponse()
        res.result = SetBehaviorResponse.ERROR
        behavior_name = req.behavior_name
        rospy.loginfo("[Socialrobot behavior] Checking %s behavior request", behavior_name)
        joint_trajectory = req.trajectory
        path_trajectory = req.path

        self.current_behavior = behavior_name
        behavior = self.behavior_list.get(behavior_name)
        behavior_model = behavior['module']
        behavior_model.reset_motion_ref(joint=joint_trajectory, path=path_trajectory)

        # request behavior motion to the action server
        if ['gripper'] == behavior['group']:
            # connect to gripper controller server
            self.gripper_ac.wait_for_server()
            goal = FollowJointTrajectoryActionGoal().goal
            goal.trajectory = behavior_model.behavior_data.get('joint')
            #print(goal)
            self.gripper_ac.send_goal(goal)
            rospy.loginfo("Wait until robot gripper is stopped.")
            self.robot_state = "MOVING_GRIPPER"
            self.gripper_ac.wait_for_result()
            rospy.loginfo("Gripper behavior is done.")
            res.result = SetBehaviorResponse.OK

        elif ['arm'] == behavior['group']:
            # connect to arm controller server
            self.arm_ac.wait_for_server()
            goal = FollowJointTrajectoryActionGoal().goal
            goal.trajectory = behavior_model.behavior_data.get('joint')
            #print(goal)
            self.arm_ac.send_goal(goal)
            rospy.loginfo("Wait until robot arm is stopped.")
            self.robot_state = "MOVING_ARM"
            self.arm_ac.wait_for_result()
            rospy.loginfo("Arm behavior is done.")
            res.result = SetBehaviorResponse.OK

        elif ['mobile'] == behavior['group']:
            # connect to mobile controller server
            self.mobile_ac.wait_for_server()
            goal = FollowPathTrajectoryActionGoal().goal
            goal.trajectory = behavior_model.behavior_data.get('path')
            
            self.mobile_ac.send_goal(goal)
            rospy.loginfo("Wait until robot mobile is stopped.")
            self.robot_state = "MOVING_BASE"
            self.mobile_ac.wait_for_result()
            rospy.loginfo("Mobile behavior is done.")
            res.result = SetBehaviorResponse.OK
        else:
            rospy.logerr("Cannot load %s behavior module", behavior_name)

        rospy.loginfo("Action is finished.")
        self.robot_state = "READY"
        return res

    def callback_get_motion(self, req):
        # load behavior module
        behavior_name = req.requirements.name
        try:
            behavior_model = self.behavior_list.get(behavior_name)['module']
        except:
            rospy.logerr("cannot load %s behavior module.", behavior_name)
            res.result = False
            return res

        # calculate motion trajectory
        ret = behavior_model.get_motion(req.requirements)

        # response
        res = GetMotionResponse()
        if ret.planResult == MotionPlanResponse.SUCCESS:
            res.result = True
            res.motion.jointTrajectory = ret.jointTrajectory
            res.motion.pathTrajectory = ret.pathTrajectory
        else:
            res.result = False
        return res

    def callback_get_requirements(self, req):
        res = GetRequirementsResponse()
        
        # load behavior module
        planner_name = req.behavior_name
        if planner_name not in self.behavior_list:
            rospy.logerr("[Socialrobot behavior] %s is not in behavior module list.",planner_name)
            res.result = False
            return res

        behavior_model = self.behavior_list.get(planner_name)['module']

        # response
        requirements = behavior_model.check_requirement()
        if len(requirements)>0:
            res.requirements = requirements
            res.result = True
        else:
            res.result = False
        return res

    def update(self):
        msg = String(data=self.robot_state)
        self.status_pub.publish(msg)
        return


##############################
# Main function
##############################
if __name__ == '__main__':
    # ros initialize
    rospy.init_node('behavior')

    # hardware interface(vrep or hw)
    hw_if = 'vrep'  # By default, vrep!
    if rospy.has_param('robot_hw'):
        hw_if = rospy.get_param('robot_hw')

    # get behavior module list
    rospack = rospkg.RosPack()
    behavior_dir = rospack.get_path('socialrobot_behavior') + '/script/behaviors/'
    behavior_list = search_behavior(behavior_dir)
    
    for except_module in ['behavior', 'utils', '__init__']:
        if except_module in behavior_list:
            behavior_list.remove(except_module)
        
    # Behavior Manager
    rospy.loginfo("Loading behavior modules...")
    bm = BehaviorManager()
    for i, behavior in enumerate(behavior_list):
        behavior_list[i] = behavior.lower()
        module_name = 'behaviors.' + behavior
        #try:
        bm.add_behavior(planner_name=behavior.lower(), behavior=load_behavior(module_name, behavior, hw_if))
        #except:
        #	rospy.logerr("[Behavior] %s cannot load.", behavior)

    # get primitive action list
    srv_act_list = rospy.ServiceProxy('/actionlib/get_action_list', GetActionList)
    rospy.loginfo("[BehaviorManager] wait for action library service...")
    rospy.wait_for_service('/actionlib/get_action_list')
    get_action_req = GetActionListRequest()
    get_action_req.action_type = GetActionListRequest().AVAILABLE_ACTIONS
    get_action_res = srv_act_list(get_action_req)
    action_list = get_action_res.actions

    # set behavior requirements
    srv_act_info = rospy.ServiceProxy('/actionlib/get_action_info', GetActionInfo)
    rospy.wait_for_service('/actionlib/get_action_info')
    for act in action_list:
        act_name = act.data.replace('_','').lower()
        if act_name in behavior_list:
            req = GetActionInfoRequest()
            req.action_name = act.data
            res = srv_act_info(req)
            try:
                bm.set_behavior_requirements(res.action)
            except:
                pass

    # ros service
    srv_get_behavior_list = rospy.Service('~get_behavior_list', GetBehaviorList, bm.callback_get_behavior_list)
    srv_get_requirements = rospy.Service('~get_requirements', GetRequirements, bm.callback_get_requirements)
    srv_get_motion = rospy.Service('~get_motion', GetMotion, bm.callback_get_motion)
    srv_set_behavior = rospy.Service('~set_behavior', SetBehavior, bm.callback_set_behavior)

    # Start
    rospy.loginfo('[BehaviorManager: %s Service Started!' % hw_if)

    loop_freq = 10  # 10hz
    r = rospy.Rate(loop_freq)

    while not rospy.is_shutdown():

        # bm update and get state
        behavior_state = bm.update()
        r.sleep()
