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

import moveit_commander
from math import pi

from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_motion.srv import *
from socialrobot_hardware.srv import *
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

    def callback_get_behavior_list(self, req):
        res = GetBehaviorListResponse()
        res.behavior_list = map(str, self.behavior_list.keys())

        return res

    def callback_set_behavior(self, req):
        res = SetBehaviorResponse()

        planner_name = req.header.frame_id
        joint_trajectory = req.trajectory
        
        behavior_model = self.behavior_list.get(planner_name)

        if behavior_model:
            self.current_behavior = planner_name
            res.result = SetBehaviorResponse.OK
            behavior_model.reset_motion_ref(trajectory = joint_trajectory)
        else:
            res.result = SetBehaviorResponse.ERROR

        return res

    def callback_get_motion(self, req):
        planner_name = req.planner_name
        behavior_model = self.behavior_list.get(planner_name)
        ret = behavior_model.get_motion(req.inputs)

        res = GetMotionResponse()
        if ret == MotionPlanResponse.SUCCESS:
            res.result = True
            res.motion.jointTrajectory = behavior_model.motion_trajectory
        else:
            res.result = False
        return res

    def update(self):
        behavior_model = self.behavior_list.get(self.current_behavior)
        #print(behavior_model)
        
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
    # moveit commander: to get a robot description
    moveit_commander.roscpp_initialize(sys.argv)
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
        
        # pub bm state

        # ros spin
        r.sleep()
