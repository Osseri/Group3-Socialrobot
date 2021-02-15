#!/usr/bin/env python
import roslib
roslib.load_manifest('socialrobot_hardware')
import rospkg

import os
import os.path
import sys
import signal
import rospy
import rosparam
import math

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')


##############################
# V-REP Interface Class
##############################
class VrepInterface():
    '''
    vrep interface
    '''
    dT = 20 # simulation step(20ms)
    active_joints = {}
    active_objects = {}

    def __init__(self):
        # VREP initialize
        sim.simxFinish(-1)  # just in case, close all opened connections
        self.client_id = sim.simxStart('127.0.0.1', 19997, True, True, 1000, 5)  # Connect to V-REP
        assert self.client_id != -1, 'Fail to connect the VREP!'

        if self.client_id != -1:
            sim.simxSynchronous(self.client_id, False)
            sim.simxSetFloatingParameter(self.client_id, sim.sim_floatparam_simulation_time_step, self.dT, sim.simx_opmode_blocking)


        self.object_handle = None
        self.robot_handle = None
        self.sim_running = False    

        self.obstacle_poses = {}
        self.obstacle_boundingboxes = {} 

        print('Connected to V-REP')
        rospy.set_param('robot_hw', 'vrep')


    def __del__(self):
        if self.client_id != -1:
            sim.simxStopSimulation(self.client_id, sim.simx_opmode_blocking)
            sim.simxGetPingTime(self.client_id)

            sim.simxFinish(self.client_id)

        print('Disconnected from the VREP')


    def step(self):
        '''
        one step forward
        '''
        # check connection
        if sim.simxGetConnectionId(self.client_id) != -1:
            # send a trigger to the server
            sim.simxSynchronousTrigger(self.client_id)
            sim.simxGetPingTime(self.client_id)

        else:
            print('[VREP] Connection Error!')
            return -1
        
        return 0


    def get_obstacles(self):
        '''
        return obstacle names, pose, bounding box
        '''
        obstacles = []
        
        # get pose (x,y,z, qx, qy, qz, qw)
        for object_name in self.active_objects:
            h = self.active_objects.get(object_name)
            _, position = sim.simxGetObjectPosition(clientID=self.client_id,
                    objectHandle=h,
                    relativeToObjectHandle=-1,
                    operationMode=sim.simx_opmode_buffer)
                    #operationMode=sim.simx_opmode_blocking)
                    
            _, quat = sim.simxGetObjectQuaternion(clientID=self.client_id,
                    objectHandle=h,
                    relativeToObjectHandle=-1,
                    operationMode=sim.simx_opmode_buffer)
                    #operationMode=sim.simx_opmode_blocking)

            obs_item = {}
            obs_item['name'] = object_name
            obs_item['pose'] = position + quat
            obs_item['shape'] = self.obstacle_boundingboxes.get(object_name)
            obstacles.append(obs_item)
        
        return obstacles

    def get_obstacles_base(self):
        '''
        return obstacle names, pose, bounding box
        '''
        obstacles = []
        
        # get pose (x,y,z, qx, qy, qz, qw)
        for object_name in self.active_objects:
            h = self.active_objects.get(object_name)
            _, position = sim.simxGetObjectPosition(clientID=self.client_id,
                    objectHandle=h,
                    relativeToObjectHandle=self.robot_handle,
                    #operationMode=sim.simx_opmode_buffer)
                    operationMode=sim.simx_opmode_blocking)
                    
            _, quat = sim.simxGetObjectQuaternion(clientID=self.client_id,
                    objectHandle=h,
                    relativeToObjectHandle=self.robot_handle,
                    #operationMode=sim.simx_opmode_buffer)
                    operationMode=sim.simx_opmode_blocking)
                    
            obs_item = {}
            obs_item['name'] = object_name
            obs_item['pose'] = position + quat
            obs_item['shape'] = self.obstacle_boundingboxes.get(object_name)
            obstacles.append(obs_item)
        
        return obstacles

    def get_obstacle_pose(self, obs_id):
        '''
        [Deprecated]
        return obstacle pose
        '''
        return 0

    def get_active_joints(self):
        '''
        return action joints
        '''

        return list(self.active_joints.keys())


    def get_joint_states(self):
        '''
        return joint states
        '''
        _joint_state = {}

        ret, handles, idata, fdata, sdata = sim.simxGetObjectGroupData(clientID=self.client_id,
                        dataType=15,
                        objectType=sim.sim_object_joint_type, 
                        #operationMode=sim.simx_opmode_blocking)
                        operationMode=sim.simx_opmode_buffer)

        for joint_name in self.active_joints:
            h = self.active_joints.get(joint_name)
            # _, q = sim.simxGetJointPosition(clientID=self.client_id,
            #         jointHandle=h,
            #         operationMode=sim.simx_opmode_blocking)
            #         #operationMode=sim.simx_opmode_buffer)

            if h in handles:
                idx = handles.index(h)
                _joint_state[joint_name] = fdata[idx*2]

        return _joint_state


    def set_target_joint_positions(self, **joint_goal):
        '''
        set joint target position
        '''
        for ii, jn in enumerate(joint_goal):
            if jn in self.active_joints:
                jh = self.active_joints.get(jn)
                goal = joint_goal.get(jn)
                #if jn=='7dof_RISE_joint_1':
                #    print("name: %s, goal: %f" % (jn, goal))

                res = sim.simxSetJointTargetPosition(clientID=self.client_id,
                        jointHandle=jh,
                        operationMode=sim.simx_opmode_oneshot,
                        targetPosition=goal)



    def start_simulation(self):
        '''
        V-REP simulation start
        '''
        # start simulation
        res = sim.simxStartSimulation(self.client_id, sim.simx_opmode_oneshot)
        if res > 2:
            print('VREP:Cannot start a simulation[error: {}]'.format(res))
            return -1

        # # set sync mode(joint_state)
        # for joint_name in self.active_joints:
        #     h = self.active_joints.get(joint_name)
        #     sim.simxGetJointPosition(clientID=self.client_id,
        #             jointHandle=h,
        #             operationMode=sim.simx_opmode_streaming)
        ret, handles, idata, fdata, sdata = sim.simxGetObjectGroupData(clientID=self.client_id,
                        dataType=15,
                        objectType=sim.sim_object_joint_type, 
                        operationMode=sim.simx_opmode_streaming)

        # set sync state(get_object_pose)
        for object_name in self.active_objects:
            h = self.active_objects.get(object_name)
            sim.simxGetObjectPosition(clientID=self.client_id,
                    objectHandle=h,
                    relativeToObjectHandle=-1,
                    operationMode=sim.simx_opmode_streaming)

            sim.simxGetObjectQuaternion(clientID=self.client_id,
                    objectHandle=h,
                    relativeToObjectHandle=-1,
                    operationMode=sim.simx_opmode_streaming)

        
        self.sim_running = True
        return 0


    def stop_simulation(self):
        '''
        V-REP simulation stop
        '''
        if self.client_id != -1:
            sim.simxStopSimulation(self.client_id, sim.simx_opmode_blocking)
            sim.simxGetPingTime(self.client_id)


    def load_sim_model(self, model_path):
        '''
        load a vrep simulation model
        '''
        # stop simulation
        if self.sim_running:
            sim.simxStopSimulation(self.client_id, sim.simx_opmode_blocking)
            sim.simxGetPingTime(self.client_id)
            self.sim_running = False

        if self.client_id != -1:
            sim.simxSynchronous(self.client_id, False)
            sim.simxSetFloatingParameter(self.client_id, sim.sim_floatparam_simulation_time_step, self.dT, sim.simx_opmode_blocking)
        else:
            return -1

        # check scene and model files
        if not os.path.exists(model_path):
            print('Invaild path!:' + model_path)
            return -1

        # close and load the scene
        if sim.simxGetConnectionId(self.client_id) != -1:
            sim.simxCloseScene(self.client_id, sim.simx_opmode_blocking)

            if sim.simxLoadScene(self.client_id, str(model_path), 1, sim.simx_opmode_blocking):
                print('VREP: scene load error')
                return -1

            # load model -> modified because of some problems
            #res, self.object_handle = sim.simxLoadModel(self.clientID, str(object_path), 1, sim.simx_opmode_blocking)
            #res, self.object_handle = sim.simxGetObjectHandle(clientID=self.client_id,
            #                objectName=target_object,
            #                operationMode=sim.simx_opmode_blocking)
            #if res:
            #    raise EnvironmentError()
        else:
            print('VREP: connection error')
            return -1


        # get action joint and objects
        ret, self.active_joints = self.get_joint_lists(self.client_id)
        ret, self.active_objects = self.get_object_lists(self.client_id)

        # get object bounding boxes
        self.obstacle_boundingboxes = self.get_object_boundingbox(self.client_id, **self.active_objects)

        print('VREP: simulation scene loaded')

        return 0


    def get_joint_lists(self, cid):
        active_joints = {}

        # check objects(joint object names)
        ret,handles,_,_,sdata = sim.simxGetObjectGroupData(clientID=cid, 
                dataType=0,  
                objectType=sim.sim_object_joint_type, 
                operationMode=sim.simx_opmode_blocking)
        if ret != 0:
            return -1, active_joints
        # check objects(joint object types)
        ret,_,idata,_,_ = sim.simxGetObjectGroupData(clientID=cid, 
                dataType=16,  
                objectType=sim.sim_object_joint_type, 
                operationMode=sim.simx_opmode_blocking)
        if ret != 0:
            return -1, active_joints

        all_joints = {}
        for i,h in enumerate(handles):
            if (idata[2*i]==10) and (idata[2*i+1]==5):
                all_joints[sdata[i]] = h

        for i,key in enumerate(all_joints):
            ret, mode = sim.simxGetObjectIntParameter(clientID=cid,
                    objectHandle=all_joints[key],
                    parameterID=sim.sim_jointintparam_motor_enabled,
                    operationMode=sim.simx_opmode_blocking)

            if ret != 0:
                return -1, active_joints
            
            elif True:#mode==1:
                active_joints[key] = all_joints[key]

        return 0, active_joints


    def get_object_lists(self, cid):
        obstacles = {}
        # check objects(joint object names)
        ret,handles,_,_,sdata = sim.simxGetObjectGroupData(clientID=cid, 
                dataType=0,  
                objectType=sim.sim_object_shape_type, 
                operationMode=sim.simx_opmode_blocking)
        if ret != 0:
            return -1, obstacles

        for i,n in enumerate(sdata):
            if n.split('_')[0] == 'obj':
                obstacles[n] = handles[i]
            elif n == 'base_footprint':
                self.robot_handle = handles[i]

        return 0, obstacles

    
    def get_object_boundingbox(self, cid, **obstacles):
        #pose = {}
        boundingbox = {}

        for i,n in enumerate(obstacles):
            h = obstacles.get(n)
            bb = [.0] * 6

            for j in range(6):
                res, bb[j] = sim.simxGetObjectFloatParameter(clientID=cid, 
                            objectHandle=h, 
                            operationMode=sim.simx_opmode_blocking, 
                            parameterID=15+j)
            
            boundingbox[n] = bb

        return boundingbox

    def set_trajectory(self, traj_length, joint_names, positions):
        inputInts = [traj_length]
        inputFloats = positions
        inputStrings = joint_names
        inputBuffer=bytearray()

        try:
            res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(self.client_id,
                        'socialrobot',
                        sim.sim_scripttype_childscript,
                        'set_trajectory',
                        inputInts=inputInts,inputFloats=inputFloats,inputStrings=inputStrings,inputBuffer=inputBuffer,
                        operationMode=sim.simx_opmode_blocking)
        except:
            print('error in set_traj')

    def get_state(self):
        inputInts = [0]
        inputFloats = [.0]
        inputStrings = ['']
        inputBuffer=bytearray()

        try:
            res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(self.client_id,
                        'socialrobot',
                        sim.sim_scripttype_childscript,
                        'get_state',
                        inputInts,inputFloats,inputStrings,inputBuffer,
                        sim.simx_opmode_blocking)
        except:
            print('error in get_state')
        
        return res, retInts


##############################
# ROS Msg Converter
##############################
from sensor_msgs.msg import JointState
from vision_msgs.msg import BoundingBox3D
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_hardware.srv import *
from socialrobot_hardware.msg import *
import rosparam
import rospkg

class RosInterface:
    '''
    some utility functions to interface with ROS
    '''
    ACTION_READY = 0
    ACTION_BUSY = 1

    def __init__(self, vrep_if, joint_remap=None):
        self.action_state = RosInterface.ACTION_READY
        self.action_start_time = 0
        self.current_time = 0

        self.vrep_if = vrep_if
        self.desired_trajectory = {}

        self.dT = self.vrep_if.dT

        if isinstance(joint_remap, dict):
            self.joint_axis = joint_remap.get('axis')
            self.joint_ratio = joint_remap.get('ratio')
            self.joint_offset = joint_remap.get('offset')
        if not self.joint_axis:
            self.joint_axis = {}
        if not self.joint_ratio:
            self.joint_ratio = {}
        if not self.joint_offset:
            self.joint_offset = {}

        
    def callback_get_obstacle(self, req):
        '''
        ROS service callback
         * get_obstacle
        '''
        # service response
        res = VrepGetObstaclesResponse()

        # get obstacles from the vrep interface
        obstacles = self.vrep_if.get_obstacles()

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
        self.vrep_if.set_trajectory(traj_length, joint_names, positions)

        return RosInterface.ACTION_READY


    def callback_get_state(self, req):
        res = VrepGetSimStateResponse()
        #if self.action_state is RosInterface.ACTION_BUSY:
        #    res.state = VrepGetSimStateResponse.RUNNING_STATE
        #elif self.action_state is RosInterface.ACTION_READY:
        #    res.state = VrepGetSimStateResponse.READY_STATE
        res.state = self.action_state
        
        return res

    def get_object(self):
        info = ObjectInfo()
        obstacles = self.vrep_if.get_obstacles()

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

    def get_object_base(self):
        info = ObjectInfo()
        obstacles = self.vrep_if.get_obstacles_base()

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

    def update_goal(self):
        # if there is no desired trajectory, empty joint goal will be returned
        joint_goal = {}
        self.current_time = 0

        # get trajectory
        traj_duration = self.desired_trajectory['duration']
        traj = self.desired_trajectory['trajectory']
        traj_length = len(traj.points)
        joint_names = traj.joint_names
        positions = []

        while self.current_time <= traj_duration:
            t_from_start = self.current_time
            
            # check the duration of the action
            if t_from_start >= traj_duration:
                for i,n in enumerate(traj.joint_names):
                    axis = self.joint_axis.get(n, 1)
                    r = self.joint_ratio.get(n, 1)
                    offset = self.joint_offset.get(n, 1) * math.pi / 180.0

                    joint_goal[n] = traj.points[-1].positions[i] / (axis * r) - offset
            else:

                # find the segment of the trajectory
                idx = int((traj_length-1) * t_from_start / traj_duration)
                #print("time: %1.2f, len: %d,  idx: %d" % (t_from_start, len(traj.points), idx))
                # ratio(0~1): position in the segment
                r = ((traj_length-1) * t_from_start / traj_duration) % 1
                #print('idx: %d, r:%f' % (idx,r))

                # interpolation
                for i,n in enumerate(traj.joint_names):
                    p1 = traj.points[idx].positions[i]
                    p2 = traj.points[idx+1].positions[i]
                    p = p1 + r*(p2-p1)

                    axis = self.joint_axis.get(n, 1)
                    r = self.joint_ratio.get(n, 1)
                    offset = self.joint_offset.get(n, 1) * math.pi / 180.0

                    joint_goal[n] = p / (axis * r) - offset  
                    #print('%s: %f' % (n,joint_goal[n]*180/math.pi))   

            self.current_time += (self.dT/1000.0)
            positions += list(joint_goal.values())
            #print(joint_goal)
        
        return traj_length, list(joint_goal.keys()), positions

    def convert_joint_state(self, **data):
        '''
        generate sensor_msgs/JointState from the vrep data
        '''
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = data.keys()
        msg.position = data.values()
        msg.velocity = [0.0] * len(data)
        msg.effort = [0.0] * len(data)

        for i,n in enumerate(msg.name):
            axis = self.joint_axis.get(n, 1)
            r = self.joint_ratio.get(n, 1)
            offset = self.joint_offset.get(n, 0) * math.pi / 180.0

            msg.position[i] = (msg.position[i] + offset) * axis * r

        return msg

    def convert_object_pose(self, **data):
        pass

    def update(self):
        res, state = self.vrep_if.get_state()

        if res == 0:
            self.action_state = state[0]



##############################
# Main function
##############################
if __name__ == '__main__':
    # loop freqency
    loop_freq = 20

    # Check joint remap configuration
    joint_remap = None
    try:
        rp = rospkg.RosPack()
        conf_path = rp.get_path('socialrobot_hardware') + '/config/jointremap.yaml'
        joint_remap = rosparam.load_file(conf_path)[0][0]
    except:
        rospy.logerr('Cannot load configuration files...')

    # Initialize vrep_interface and ros_interface
    vrep_if = VrepInterface()
    ros_if = RosInterface(vrep_if, joint_remap=joint_remap)

    # Initialize ROS node
    rospy.init_node('vrep_interface')

    # To get a robot description
    robot_name = rospy.get_param('/robot_name')
    sim_env = rospy.get_param(rospy.get_name() + '/vrep_environment')
    sim_model_path = rospy.get_param('/robot_description_path') + '/vrep_model/' + robot_name + '_' + sim_env + '.ttt'

    # load sim model
    if vrep_if.load_sim_model(sim_model_path):
        rospy.logerr('VREP:Simulation model load error!')
        rospy.signal_shutdown('Quit')

    # start vrep simulation
    if vrep_if.start_simulation():
        rospy.logerr('VREP:Simulation start error!')
        rospy.signal_shutdown('Quit')

    # ROS topic
    pub_joint_state = rospy.Publisher('~joint_states', JointState, queue_size=10)
    pub_vrep_state = rospy.Publisher('~vrep_state', Int32, queue_size=10)
    pub_obj_info = rospy.Publisher('~objects', ObjectInfo, queue_size=10)
    pub_obj_base = rospy.Publisher('~objects_base', ObjectInfo, queue_size=10)

    # ROS services
    srv_get_obstacles = rospy.Service('~get_obstacles', VrepGetObstacles, ros_if.callback_get_obstacle)
    srv_set_motion = rospy.Service('~set_motion', VrepSetJointTrajectory, ros_if.callback_set_motion)
    srv_get_state = rospy.Service('~get_state', VrepGetSimState, ros_if.callback_get_state)
    #srv_get_obstacle_pose = rospy.Service('check_action', CheckAction, self._callback_check_action)
    #srv_getMotion = rospy.Service('get_motion', GetMotion, self._callback_get_motion)

    rospy.loginfo('[VREP Interface] Service Started!')

    r = rospy.Rate(loop_freq)
    count = 0
    joint_state = vrep_if.get_joint_states()
    while not rospy.is_shutdown():
        ######################
        # read 
        ######################
        
        # vrep state
        if vrep_if.sim_running:
            vrep_state = 1
        else:
            vrep_state = 0

        # joint state
        if count > 10:
            joint_state = vrep_if.get_joint_states()
            count = 0
        else:
            count += 1


        ######################
        # write
        ######################
        # write states
        #dT = 1/loop_freq
        ros_if.update()

        # publish topic
        pub_vrep_state.publish(ros_if.action_state)
        pub_joint_state.publish(ros_if.convert_joint_state(**joint_state))
        pub_obj_info.publish(ros_if.get_object())        
        pub_obj_base.publish(ros_if.get_object_base())        

    vrep_if.stop_simulation()
