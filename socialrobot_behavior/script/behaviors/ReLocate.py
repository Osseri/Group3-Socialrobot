import abc
from abc import ABCMeta
from six import with_metaclass 
import rospy
import rosparam
import actionlib
import math
import numpy as np 

from socialrobot_motion.srv import *
from socialrobot_behavior.srv import *
from geometry_msgs.msg import * 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from rearrange_node import msg as rearr_msg
from rearrange_node import srv as rearr_srv
from behavior import BehaviorBase

import tf
import tf.transformations as tfm

RED = ColorRGBA(1.0, 0.0, 0.0, 1.0)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 1.0)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)
t_BLUE = ColorRGBA(0.0, 0.0, 1.0, 0.5)

def degToRad(deg):
    rad = deg / 180.0 * math.pi
    return rad

def pClosest(start_point, target_points, k): 
    target_points.sort(key = lambda pt: (pt.x-start_point.x)**2 + (pt.y-start_point.y)**2) 
    return target_points[:k]

class ReLocateBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(ReLocateBehavior, self).__init__(name, **params)
        
        armplan_srv = '/motion_plan/move_arm'
        self.listener = tf.TransformListener()
        self.talker = tf.TransformBroadcaster(queue_size=10)
        self.srv_plan = rospy.ServiceProxy(armplan_srv, MotionPlan)
        self.objects = {'target': None, 'rearrange_target': None, 'obstacles': [], 'workspace': None}
        self.rot_x = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (1,0,0))
        self.rot_y = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0,1,0))
        self.rot_z = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0,0,1))

        # get hardware 
        robot_name = rosparam.get_param("/robot_name")
        if robot_name == 'skkurobot':
            self.gripper_pose = {'left': '7dof_RISE_wrist_link',
                                'right': '6dof_connection_link',
                                'left_offset': [-0.02, 0.08],
                                'right_offset': [-0.025, 0.08]
                                }  
        elif robot_name == 'social_robot':
            self.gripper_pose =  {'left': 'LHand_base',
                                'right': 'RHand_base',
                                'left_offset': [-0.0055, 0.03],
                                'right_offset': [-0.0055, 0.03]
                                }

    def check_requirement(self):
        rospy.loginfo('checking...%s' % self._name)
        return True

    def prepare_behavior(self):
        rospy.loginfo('preparing...%s' % self._name)
        return True

    def run_behavior(self):           
        return True

    def get_motion(self, inputs):
        '''
        return the trajectory 
        '''
        rospy.loginfo("Calculating relocation motion..")
        res = MotionPlanResponse()
        ret , srv_response = self._get_motion(inputs)
        if ret == MotionPlanResponse.SUCCESS:
            rospy.loginfo("Relocation planning is done.")
            self.motion_trajectory = srv_response.jointTrajectory
            res.planResult = MotionPlanResponse.SUCCESS
            res.jointTrajectory = srv_response.jointTrajectory
            return res
        else:
            rospy.loginfo("Relocation planning is failed.")
            res.planResult = MotionPlanResponse.ERROR_NO_SOLUTION
            return res 

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True

    def _get_motion(self, inputs):
        """check motion path is exist

        Args:
            inputs ([socialrobot_behavior/PlannerInputs]): planner inputs

        Returns:
            [bool]: motion plan result
            [socialrobot_motion/MotionPlanResponse]: motion trajectory
        """
        # get rearrange position
        motion_plan_res = MotionPlanResponse()
        rearrange_result, rearrange_positions = self._get_rearrage_positions(inputs)
        if rearrange_result:
            # check whether the motion trajectory to rearrange position is exist
            plan_result, motion_plan_res = self._check_positions(inputs, rearrange_positions)
            if not plan_result:
                rospy.logerr("[Socialrobot Behavior] cannot find motion to rearrange positions.")
                self.srv_visualize(lifetime=-1)
                return MotionPlanResponse.ERROR_FAIL, motion_plan_res
        else:
            rospy.logerr("[Socialrobot Behavior] cannot find rearrange positions.")
            self.srv_visualize(lifetime=-1)
            return MotionPlanResponse.ERROR_FAIL, motion_plan_res
        self.srv_visualize(lifetime=-1)
        return MotionPlanResponse.SUCCESS, motion_plan_res

    def _check_positions(self, inputs, positions):
        """[summary]

        Args:
            inputs ([type]): planner inputs
            positions ([geometry_msgs/Point]): candidate positions
        """
        # sort by distance choose k nearest points
        robot_part = inputs.targetBody
        sorted_positions = []
        
        if robot_part == socialrobot_behavior.msg.PlannerInputs.LEFT_ARM:        
            start_point = Vector3(x=0.0, y=0.6245)            
            sorted_positions = pClosest(start_point, positions, 5)
        elif robot_part == socialrobot_behavior.msg.PlannerInputs.RIGHT_ARM:       
            start_point = Vector3(x=0.0, y=-0.6245)     
            sorted_positions = pClosest(start_point, positions, 5)

        for pos in sorted_positions:
            plan_req = MotionPlanRequest()
            plan_req.targetBody = robot_part

            # get obstacles
            plan_req.obstacle_ids = inputs.obstacle_ids
            plan_req.obstacles = inputs.obstacles

            # set grasped object
            plan_req.targetObject = [inputs.targetObject[1]]

            # get bounding box of target object 
            target_bb3d = self.objects['target']  

            sample_angle = [90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255, 270] 

            for idx, i in enumerate(sample_angle):
                wrist_pose, eef_pose = self.get_goal_pose(robot_part, i, pos)
                
                # publish sample pose
                self.talker.sendTransform((eef_pose.position.x, eef_pose.position.y, eef_pose.position.z), (eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z, eef_pose.orientation.w), rospy.Time.now(), "approach_sample_pose", "base_footprint")

                if wrist_pose:
                    # target pose
                    plan_req.targetPose = wrist_pose
                    plan_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL
                    
                    plan_res = self.srv_plan(plan_req) 
                    if plan_res.planResult == MotionPlanResponse.SUCCESS:
                        return True, plan_res
                    else:
                        pass                        
            return False, None

    def get_goal_pose(self, robot_part, angle, goal_position):

        # get transforms between wrist and end-effector
        base_to_eef_trans = base_to_eef_rot = []
        eef_to_wrist_trans = eef_to_wrist_rot = []        
        is_trans = False

        while is_trans == False:             
            try:    
                if robot_part is MotionPlanRequest.LEFT_ARM:  
                    base_to_eef_trans, base_to_eef_rot = self.listener.lookupTransform('/base_footprint', '/left_end_effect_point', rospy.Time(0))
                    eef_to_wrist_trans, eef_to_wrist_rot = self.listener.lookupTransform('/left_end_effect_point', self.gripper_pose['left'], rospy.Time(0))
                    offset = self.gripper_pose['left_offset']

                elif robot_part is MotionPlanRequest.RIGHT_ARM:
                    base_to_eef_trans, base_to_eef_rot = self.listener.lookupTransform('/base_footprint', '/right_end_effect_point', rospy.Time(0))
                    eef_to_wrist_trans, eef_to_wrist_rot = self.listener.lookupTransform('/right_end_effect_point', self.gripper_pose['right'], rospy.Time(0))
                    offset = self.gripper_pose['right_offset']
                is_trans = True
            except:
                pass
        is_trans = False

        # for i in azimuth_angle:
        eef_pose = Pose()
        wrist_pose = Pose()

        i = angle
        target_size = self.objects['rearrange_target'].size
        z_offset = 0.01   
        
        eef_pose.position.x = goal_position.x + (target_size.x/2 + offset[1])*math.cos(degToRad(i))
        eef_pose.position.y = goal_position.y + (target_size.x/2 + offset[1])*math.sin(degToRad(i))
        eef_pose.position.z = goal_position.z + z_offset

        # 
        init_quat = [0, -0.707, 0, 0.707]
        quat = tfm.quaternion_multiply(init_quat, tfm.quaternion_about_axis(degToRad(i), (1,0,0)))

        eef_pose.orientation.x = quat[0]
        eef_pose.orientation.y = quat[1]
        eef_pose.orientation.z = quat[2]
        eef_pose.orientation.w = quat[3]

        if robot_part == MotionPlanRequest.RIGHT_ARM:
            quat = tfm.quaternion_multiply(np.array([quat[0], quat[1], quat[2], quat[3]]), self.rot_z(180))

            eef_pose.orientation.x = quat[0]
            eef_pose.orientation.y = quat[1]
            eef_pose.orientation.z = quat[2]
            eef_pose.orientation.w = quat[3]           

        # transform to wrist from eef
        trans1_mat = tf.transformations.translation_matrix([eef_pose.position.x, eef_pose.position.y, eef_pose.position.z])
        rot1_mat   = tf.transformations.quaternion_matrix([eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z, eef_pose.orientation.w])
        mat1 = np.dot(trans1_mat, rot1_mat)

        trans2_mat = tf.transformations.translation_matrix(eef_to_wrist_trans)
        rot2_mat = tf.transformations.quaternion_matrix(eef_to_wrist_rot)
        mat2 = np.dot(trans2_mat, rot2_mat)

        mat3 = np.dot(mat1, mat2)
        trans3 = tf.transformations.translation_from_matrix(mat3) 
        rot3 = tf.transformations.quaternion_from_matrix(mat3)

        wrist_pose.position.x = trans3[0]
        wrist_pose.position.y = trans3[1]
        wrist_pose.position.z = trans3[2]

        wrist_pose.orientation.x = rot3[0]
        wrist_pose.orientation.y = rot3[1]
        wrist_pose.orientation.z = rot3[2]
        wrist_pose.orientation.w = rot3[3]


        return wrist_pose, eef_pose
        
    def _get_rearrage_positions(self, inputs):
        """ get rearrange position by rearrange service

        Args:
            [socialrobot_behavior/PlannerInputs]: planner inputs

        Returns:
            [geometry_msgs/Point[]]: rearrange candidate positions
        """
        rospy.wait_for_service("rearrange_srv")
        try:
            f_check_srv = rospy.ServiceProxy("rearrange_srv", rearr_srv.rearrange_env_srv)
            req = self.get_rearrange_request(inputs)
            self.srv_visualize(request=req, lifetime=0)
            resp = f_check_srv(req)   
            if len(resp.rearrange_positions)>0:
                self.srv_visualize(request=req, response=resp, lifetime=0)
                return True, resp.rearrange_positions
            else:
                self.srv_visualize(request=req, lifetime=1)
                return False, None
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return False, None

    def get_rearrange_request(self, inputs):
        req = rearr_srv.rearrange_env_srvRequest()

        robot_part = inputs.targetBody
        if robot_part == socialrobot_behavior.msg.PlannerInputs.LEFT_ARM:
            req.robot_pose = [0.0, 0.6245] 
        elif robot_part == socialrobot_behavior.msg.PlannerInputs.RIGHT_ARM:
            req.robot_pose = [0.0, -0.6245] 
        
        
        for i,obj in enumerate(inputs.obstacle_ids):
            if obj == inputs.targetObject[0]:
                self.objects['target'] = inputs.obstacles[i]
            elif obj == inputs.targetObject[1]:
                self.objects['rearrange_target'] = inputs.obstacles[i]
            elif obj == 'obj_table':
                self.objects['workspace'] = inputs.obstacles[i]
            else:
                self.objects['obstacles'].append([obj,inputs.obstacles[i]])
                
        if not self.objects['rearrange_target']:
            rospy.logerr("No rearrange target object for relocation.")
        if not self.objects['workspace']:
            rospy.logerr("No workspace object for relocation.")
        
        # Grasp target
        if not self.objects['target']:
            rospy.logwarn("No target object for relocation.")
            req.target = self.get_env_object_info(
                "empty", Point(x=0.55, y=0, z=0), 
                        Quaternion(x=0, y=0, z=0, w=1), 
                        Vector3(x=6.5000e-02, y=6.5000e-02, z=2.3544e-01)
            )
        else:
            req.target = self.get_env_object_info(
                inputs.targetObject[0], self.objects['target'].center.position, 
                                        self.objects['target'].center.orientation, 
                                        self.objects['target'].size
            )
        # The obstacles on the workspace object
        for obs in inputs.targetObject[1:]:
            req.objects.append(
                self.get_env_object_info(
                    obs, self.objects['rearrange_target'].center.position, 
                            self.objects['rearrange_target'].center.orientation, 
                            self.objects['rearrange_target'].size
                )
            )
        for obs in self.objects['obstacles']:
            req.objects.append(
                self.get_env_object_info(
                    obs[0], obs[1].center.position, 
                            obs[1].center.orientation, 
                            obs[1].size
                )
            )
        # TODO: remove table information hard-coding
        # The workspace object
        if self.objects['workspace']:

            req.workspace = self.get_env_object_info(
                "obj_table", Point(x=+5.5001e-01/4*3, y=0, z=0.4), 
                                Quaternion(x=0, y=0, z=0, w=1), 
                                Vector3(x=0.7088739275932312/2, y=1.15, z=0.8)
            )
        return req

    def get_env_object_info(self, name, position, orientation, size):
        return rearr_msg.env_object_info_msg(
            object_name=[name,],
            object_position=position,
            object_orientation=orientation,
            object_scale=size,
        )

    def create_marker(self, position, orientation, scale, color_rgba, shape, lifetime=0, id=0):
        import random
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = rospy.Time()
        marker.ns = "marker"
        marker.id = id
        marker.lifetime = rospy.Duration(lifetime)
        marker.type = shape
        marker.action = marker.ADD
        if lifetime<0:
            marker.action = marker.DELETEALL
        marker.color = color_rgba
        marker.pose.position = position
        marker.pose.orientation = orientation
        marker.scale = scale
        return marker

    def srv_visualize(self, request=None, response=None, lifetime=0):
        # request
        marker_pub = rospy.Publisher(
                "rearrange_node/markers", MarkerArray, queue_size=10, latch=True
            )        
        req = MarkerArray()

        if request != None:            
            req.markers.append(
                self.create_marker(request.workspace.object_position,
                                   request.workspace.object_orientation, 
                                   request.workspace.object_scale, 
                                   WHITE, Marker.CUBE, lifetime=lifetime, id=1)
            )
            req.markers.append(
                self.create_marker(request.target.object_position, 
                                    request.target.object_orientation, 
                                    request.target.object_scale, 
                                    GREEN, Marker.CYLINDER, lifetime=lifetime, id=2)
            )
            id=3
            for obstacle in request.objects:
                req.markers.append(self.create_marker(obstacle.object_position, 
                                                              obstacle.object_orientation,  
                                                              obstacle.object_scale, 
                                                              RED, Marker.CYLINDER,
                                                              lifetime=lifetime, id=id))
                id+=1
            marker_pub.publish(req)

        # response
        if response !=None:
            for point in response.rearrange_positions:
                req.markers.append(
                    self.create_marker(
                        point,
                        Quaternion(0, 0, 0, 1),
                        Vector3(0.06, 0.06, 0.2),
                        t_BLUE,
                        Marker.CYLINDER,
                        lifetime=lifetime, id=id
                    )
                )
                id+=1
        if lifetime<0:
            req.markers.append(
                self.create_marker(
                    Point(0,0,0),
                    Quaternion(0, 0, 0, 1),
                    Vector3(0,0,0),
                    t_BLUE,
                    Marker.CYLINDER,
                    lifetime=lifetime, id=0
                )
            )

        while marker_pub.get_num_connections() <1:
            if rospy.is_shutdown():
                return
            rospy.sleep(1)
        marker_pub.publish(req)


if __name__ == "__main__":
    rospy.init_node('test_relocate_object_node')
    reloc = ReLocateBehavior('relocate')


    from socialrobot_behavior import msg as behavior_msg
    from socialrobot_behavior import srv as behavior_srv
    plan_req = behavior_msg.PlannerInputs()
    plan_req.targetBody = behavior_msg.PlannerInputs.LEFT_ARM
    plan_req.obstacle_ids
    plan_req.obstacles

    # obstacles
    # red_gotica
    from vision_msgs.msg import *
    obs1 = BoundingBox3D()
    c1 = Pose()
    c1.position.x = +3.0000e-01
    c1.position.y = +9.9997e-02
    c1.position.z = +8.2886e-01
    c1.orientation.x = 1.31936e-05
    c1.orientation.y = 2.20794e-10
    c1.orientation.z = 6.07222e-07
    c1.orientation.w = 1
    obs1.center = c1
    v1 = Vector3()
    v1.x = 0.0618015 
    v1.y = 0.059508 
    v1.z = 0.23814
    obs1.size = v1

    # gotica
    obs2 = BoundingBox3D()
    c2 = Pose()
    c2.position.x = +4.0000e-01
    c2.position.y = -1.5003e-02
    c2.position.z = +8.2886e-01
    c2.orientation.x = 1.31627e-05 
    c2.orientation.y = 2.26816e-10
    c2.orientation.z = -1.15535e-18   
    c2.orientation.w = 1.0              
    obs2.center = c2
    v2 = Vector3()
    v2.x = 0.065 
    v2.y = 0.065 
    v2.z = 0.23544
    obs2.size = v2
    
    # table
    obs3 = BoundingBox3D()
    c = Pose()
    c.position.x = 0.550006
    c.position.y = 8.80659e-06
    c.position.z = 0.365011
    c.orientation.x = 0
    c.orientation.y = 0
    c.orientation.z = 0.707
    c.orientation.w = 0.707
    obs3.center = c
    v = Vector3()
    v.x = 1.1342161893844604
    v.y = 0.7088739275932312
    v.z = 0.6899999976158142
    obs3.size = v

    # add obstacles 
    plan_req.obstacle_ids = ['obj_red_gotica', 'obj_gotica']
    plan_req.targetObject = ['obj_gotica', 'obj_red_gotica']   #1.target 2. rearrange_target
    plan_req.obstacles.append(obs1)
    plan_req.obstacles.append(obs2)
    #plan_req.obstacles.append(obs3)
    

    print(reloc.get_motion(plan_req))