#!/usr/bin/env python
import os
import os.path
import sys
import signal
import math
import time

import rospy
import rosparam
import rospkg

from sensor_msgs.msg import JointState
from vision_msgs.msg import BoundingBox3D
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_hardware.srv import *
from socialrobot_hardware.msg import *

try:
    from legacy_api.simConst import *
    import bluezero_api.b0RemoteApi as client
except:
    print ('--------------------------------------------------------------')
    print ('"b0.py" could not be imported. This means very probably that')
    print ('either "b0.py" or the b0RemoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "b0.py"')
    print ('--------------------------------------------------------------')
    print ('')

class VrepInterface(object):

    def __init__(self, ros_if):
        # vrep client variables
        self.do_next_step = True
        self.object_handle = None
        self.robot_handle = None
        self.joint_handle = None
        self.object_handle = None
        self.ros_if = ros_if

        # connect to vrep server
        self.client = client.RemoteApiClient('b0RemoteApi_client', 'b0RemoteApi')

        # get scene models
        self.active_joints = self.get_joint_lists()
        self.active_objects = self.get_object_lists()

        # set simualtion synchronous mode
        self.client.simxSynchronous(True)

        # vrep synchronous fuction subscribers
        self.client.simxGetSimulationStepStarted(self.client.simxDefaultSubscriber(self._simulation_step_start_cb))
        self.client.simxGetSimulationStepDone(self.client.simxDefaultSubscriber(self._simulation_step_done_cb))

        # create vrep joint subscribers
        self.create_joint_subscriber()
        # create vrep object subscribers
        self.create_object_subscriber()

        # print('=====================Joint===================')
        # print(self.active_joints)
        # print('=====================Objects===================')
        # print(self.active_objects)

    def __del__(self):
        self.stop_simulation()

    def _simulation_step_start_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

    def _simulation_step_done_cb(self, msg):
        # get simualtino time
        sim_time = msg[1][b'simulationTime']

        # after step done publish ros msg
        self.ros_if.pub_joint_state.publish(self.ros_if.convert_joint_state(self.get_joint_state()))
        self.ros_if.pub_obj_base.publish(self.ros_if.get_object_base(self.active_objects))  

        # change do_next_step state
        self.do_next_step = True

    def start_simulation(self):
        self.client.simxStartSimulation(self.client.simxDefaultPublisher())
        rospy.loginfo("VREP: start vrep simulation with bluezero remote API")

    def stop_simulation(self):
        self.client.simxStopSimulation(self.client.simxDefaultPublisher())
        rospy.loginfo("VREP: stop simulation")

    def step_simulation(self):
        while not self.do_next_step:
            self.client.simxSpinOnce()
        self.do_next_step = False
        self.client.simxSynchronousTrigger()

    def get_joint_lists(self):
        rospy.loginfo("VREP: Getting active joints...")
        active_joints = {}

        # get joint type object handles
        joint_handles = self.client.simxGetObjects(sim_object_joint_type, self.client.simxServiceCall())[1]
        for h in joint_handles:
            name = self.client.simxGetObjectName(h, False, self.client.simxServiceCall())[1]
            mode = self.client.simxGetObjectIntParameter(h, sim_jointintparam_motor_enabled, self.client.simxServiceCall())[1]
            if mode == 1: #motor enabled
                active_joints[h] = {}
                active_joints[h]['name'] = name
                
        return active_joints

    def get_object_lists(self):
        rospy.loginfo("VREP: Getting active objects...")
        active_objects = {}

        # get joint type object handles
        object_handles = self.client.simxGetObjects(sim_object_shape_type, self.client.simxServiceCall())[1]
        for h in object_handles:
            name = self.client.simxGetObjectName(h, False, self.client.simxServiceCall())[1]
            if 'obj_' in name or 'aff_' in name:
                active_objects[h] = {}
                active_objects[h]['name'] = name
            elif name == 'base_footprint':
                self.robot_handle = h
        return active_objects

    def create_joint_subscriber(self):
        rospy.loginfo("VREP: Creating joint subscribers...")
        for h in self.active_joints:
            callback_func = self.create_joint_callback(h)
            self.active_joints[h]['cb'] = callback_func
            self.active_joints[h]['pos'] = 0.0
            self.client.simxGetJointPosition(h, self.client.simxCreateSubscriber(self.active_joints[h]['cb'], dropMessages=True))

    def create_joint_callback(self, handle):
        def joint_callback(*inputs):
            if len(inputs)>0:
               self.active_joints[handle]['pos'] = inputs[0][1]
        return joint_callback

    def create_object_subscriber(self):
        rospy.loginfo("VREP: Creating object subscribers...")
        for h in self.active_objects:
            pose_callback_func = self.create_object_pose_callback(h)
            shape_callback_func = self.create_object_shape_callback(h)
            self.active_objects[h]['pose_cb'] = pose_callback_func
            self.active_objects[h]['shape_cb'] = shape_callback_func
            
            self.active_objects[h]['pose'] = [0.0]*7
            self.active_objects[h]['shape'] = [0.0]*6

            self.client.simxGetObjectPose(h, self.robot_handle, 
                                            self.client.simxCreateSubscriber(self.active_objects[h]['pose_cb'], 
                                            dropMessages=True))
            for j in range(6):
                self.client.simxGetObjectFloatParameter(h, 15+j, 
                                                    self.client.simxCreateSubscriber(self.active_objects[h]['shape_cb'][j], 
                                                    dropMessages=True))

    def create_object_pose_callback(self, handle):
        def pose_callback(*inputs):
            if len(inputs)>0:
               self.active_objects[handle]['pose'] = inputs[0][1]
        return pose_callback

    def create_object_shape_callback(self, handle):
        def min_x_callback(*inputs):
            if len(inputs)>0:
               self.active_objects[handle]['shape'][0] = inputs[0][1]        
        def min_y_callback(*inputs):
            if len(inputs)>0:
               self.active_objects[handle]['shape'][1] = inputs[0][1]      
        def min_z_callback(*inputs):
            if len(inputs)>0:
               self.active_objects[handle]['shape'][2] = inputs[0][1]      
        def max_x_callback(*inputs):
            if len(inputs)>0:
               self.active_objects[handle]['shape'][3] = inputs[0][1]      
        def max_y_callback(*inputs):
            if len(inputs)>0:
               self.active_objects[handle]['shape'][4] = inputs[0][1]      
        def max_z_callback(*inputs):
            if len(inputs)>0:
               self.active_objects[handle]['shape'][5] = inputs[0][1]
        return [min_x_callback, min_y_callback, min_z_callback, max_x_callback, max_y_callback, max_z_callback]

    def get_joint_state(self):
        joint_state = {}
        for h in self.active_joints:
            name = self.active_joints[h]['name']
            pos = self.active_joints[h]['pos']
            joint_state[name] = pos
        if len(joint_state)>0:
            return joint_state
        else:
            rospy.logwarn("VREP: No joints in the scene.")
            return {}

##############################
# ROS Msg Converter
##############################
class RosInterface:
    '''
    some utility functions to interface with ROS
    '''
    ACTION_READY = 0
    ACTION_BUSY = 1

    def __init__(self):
        self.obstacles = {}
        self.joints = {}
        
        self.action_state = RosInterface.ACTION_READY
        self.action_start_time = 0
        self.current_time = 0
        self.desired_trajectory = {}
        
        # ROS topic
        self.pub_joint_state = rospy.Publisher('~joint_states', JointState, queue_size=10)
        self.pub_vrep_state = rospy.Publisher('~vrep_state', Int32, queue_size=10)
        self.pub_obj_info = rospy.Publisher('~objects', ObjectInfo, queue_size=10)
        self.pub_obj_base = rospy.Publisher('~objects_base', ObjectInfo, queue_size=10)

        # ROS services
        #srv_get_obstacles = rospy.Service('~get_obstacles', VrepGetObstacles, self._get_obstacle_cb)
        #srv_set_motion = rospy.Service('~set_motion', VrepSetJointTrajectory, self._set_motion_cb)
        #srv_get_state = rospy.Service('~get_state', VrepGetSimState, self._get_state_cb)

    def callback_get_obstacle(self, req):
        '''
        ROS service callback
         * get_obstacle
        '''
        # service response
        res = VrepGetObstaclesResponse()

        # get obstacles from the vrep interface
        obstacles = self.obstacles

        for i,obs in enumerate(obstacles):
            obs_name = obs.get('name')
            obs_pose = obs.get('pose')
            obs_shape = obs.get('shape')

            # convert vrep shape info. to bounding box
            bb3d = BoundingBox3D()
            bb3d.center.position = Point(*obs_pose[0:3])
            bb3d.center.orientation = Quaternion(*obs_pose[3:7])
            bb3d.size.x = abs(obs_shape[3] - obs_shape[0])
            bb3d.size.y = abs(obs_shape[4] - obs_shape[1])
            bb3d.size.z = abs(obs_shape[5] - obs_shape[2])

            res.names.append(obs_name)
            res.obstacles.append(bb3d)
        return res

    def callback_set_motion(self, req):
        res = VrepSetJointTrajectoryResponse()
        rospy.loginfo("VREP: set_motion!")
        
        self.desired_trajectory['duration'] = req.duration
        self.desired_trajectory['trajectory'] = req.trajectory

        traj_length, joint_names, positions = self.update_goal()
        self.set_trajectory(traj_length, joint_names, positions)

        return RosInterface.ACTION_READY

    def callback_get_state(self, req):
        res = VrepGetSimStateResponse()
        #if self.action_state is RosInterface.ACTION_BUSY:
        #    res.state = VrepGetSimStateResponse.RUNNING_STATE
        #elif self.action_state is RosInterface.ACTION_READY:
        #    res.state = VrepGetSimStateResponse.READY_STATE
        res.state = self.action_state
        
        return res

    def get_object(self, obstacles):
        info = ObjectInfo()

        for i,obs in enumerate(obstacles):
            obs_name = obs.get('name')
            obs_pose = obs.get('pose')
            obs_shape = obs.get('shape')

            # convert vrep shape info. to bounding box
            bb3d = BoundingBox3D()
            bb3d.center.position = Point(*obs_pose[0:3])
            bb3d.center.orientation = Quaternion(*obs_pose[3:7])
            bb3d.size.x = abs(obs_shape[3] - obs_shape[0])
            bb3d.size.y = abs(obs_shape[4] - obs_shape[1])
            bb3d.size.z = abs(obs_shape[5] - obs_shape[2])

            info.names.append(obs_name)
            info.obstacles.append(bb3d)

        return info

    def get_object_base(self, obstacles):
        info = ObjectInfo()

        for h in obstacles:
            obs_name = obstacles[h]['name']
            obs_pose = obstacles[h]['pose']
            obs_shape = obstacles[h]['shape']

            # convert vrep shape info. to bounding box
            bb3d = BoundingBox3D()
            bb3d.center.position = Point(*obs_pose[0:3])
            bb3d.center.orientation = Quaternion(*obs_pose[3:7])
            bb3d.size.x = abs(obs_shape[3] - obs_shape[0])
            bb3d.size.y = abs(obs_shape[4] - obs_shape[1])
            bb3d.size.z = abs(obs_shape[5] - obs_shape[2])

            info.names.append(obs_name)
            info.obstacles.append(bb3d)

        return info

    def convert_joint_state(self, data):
        '''
        generate sensor_msgs/JointState from the vrep data
        '''
        msg = JointState()
        if len(data)>0:
            msg.header.stamp = rospy.Time.now()
            msg.name = data.keys()
            msg.position = data.values()
            msg.velocity = [0.0] * len(data)
            msg.effort = [0.0] * len(data)
        return msg

    def update(self):
        pass

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('vrep_interface')
    rospack = rospkg.RosPack()
    
    # Initialize vrep_interface and ros_interface
    ros_if = RosInterface()
    vrep_if = VrepInterface(ros_if)
    
    # Start simulation
    vrep_if.start_simulation()

    while not rospy.is_shutdown():
        vrep_if.step_simulation()

    vrep_if.stop_simulation()