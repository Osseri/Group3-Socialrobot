#!/usr/bin/env python
import importlib
import roslib
import rospkg
import rospy
import rosparam

import os
import os.path
import sys
from six import with_metaclass 

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import JointState
from socialrobot_hardware.srv import *

from interfaces.interface import InterfaceBase


def load_interface(module, robot_name):
    interface_module = importlib.import_module(module)
    interface_class = getattr(interface_module, robot_name+'Interface')
    interface_instance = interface_class(robot_name.lower())
    return interface_instance

def search_interface(dirname):
    interface_list = []
    for file in os.listdir(dirname):
        if file.endswith(".py"):
            file_name = os.path.join(dirname, file)
            interface_list.append((file_name.replace(dirname,'')).replace('.py',''))
    return interface_list

class Singleton(type):
    ''' a base class for singleton pattern '''
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class HardwareInterface():
    '''
    socialrobot interface
    '''

    def __init__(self, robot_name):
        # Robot initialize
        rospy.set_param('robot_hw', 'hw')
        self.robot_name = robot_name
        self.robot_interface = ''
        self.robot_running = False    

        self.obstacle_poses = {}
        self.obstacle_boundingboxes = {} 

        # ROS topic
        self.pub_joint_state = rospy.Publisher('~joint_states', JointState, queue_size=10)

        # ROS services
        srv_set_motion = rospy.Service('~set_motion', SetJointTrajectory, self.callback_set_motion)
        srv_get_state = rospy.Service('~get_state', GetRobotState, self.callback_get_state)

    def __del__(self):
        print('Hardware interface is closed.')

    def add_interface(self, interface):
        self.robot_interface = interface        
        return

    def callback_set_motion(self, req):   
        return self.robot_interface.set_motion(req)

    def callback_get_state(self, req):
        res = GetRobotStateResponse()        
        self.robot_interface.get_state(req)
        return res


    def update(self):
        joint_goal = self.robot_interface.update()
        return 


##############################
# Main function
##############################
if __name__ == '__main__':   

    # Initialize ROS node
    rospy.init_node('hw_interface')

    # To get a robot description
    robot_name = "skkurobot"
    if rospy.has_param('/robot_name'):
        robot_name = rospy.get_param('/robot_name')
    else:
        rospy.logerr('cannot find /robot-name parameter')

    # get interface list
    rospack = rospkg.RosPack()    
    interface_dir = rospack.get_path('socialrobot_hardware') + '/scripts/interfaces/'
    interface_list = search_interface(interface_dir)
    if 'interface' in interface_list:
        interface_list.remove('interface')
    if '__init__' in interface_list:
        interface_list.remove('__init__')

    # set hardware interface
    hw_if = HardwareInterface(robot_name)
    module = 'interfaces.' + robot_name
    hw_if.add_interface(load_interface(module, robot_name))

    while not rospy.is_shutdown():
        hw_if.update()