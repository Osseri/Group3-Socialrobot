import abc
from abc import ABCMeta
from six import with_metaclass 
import rospy
import rosparam

import tf
import tf.transformations as tfm
import math
import numpy as np 

from socialrobot_msgs.msg import Object
from socialrobot_motion.srv import *
from socialrobot_behavior.srv import *
from geometry_msgs.msg import * 
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import BoundingBox3D
from visualization_msgs.msg import MarkerArray, Marker
from rearrange_node import msg as rearr_msg
from rearrange_node import srv as rearr_srv

from behavior import BehaviorBase
import utils

RED = ColorRGBA(1.0, 0.0, 0.0, 1.0)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 1.0)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)
t_BLUE = ColorRGBA(0.0, 0.0, 1.0, 0.5)


def degToRad(deg):
    rad = deg / 180.0 * math.pi
    return rad

def pose_2_mat(trans, rot):
    trans_vec = [trans['x'], trans['y'], trans['z']]
    rot_vec = [rot['x'], rot['y'], rot['z'], rot['w']]

    trans_mat = tf.transformations.translation_matrix(trans_vec)
    rot_mat = tf.transformations.quaternion_matrix(rot_vec)
    T = np.dot(trans_mat, rot_mat) 
    return T

def mat_2_pose(T):
    trans = tf.transformations.translation_from_matrix(T) 
    rot = tf.transformations.quaternion_from_matrix(T)
    
    pose_trans = {'x': trans[0], 'y': trans[1], 'z': trans[2]}
    pose_rot = {'x': rot[0], 'y': rot[1], 'z': rot[2], 'w': rot[3]}
    return pose_trans, pose_rot

def pClosest(start_point, target_points, k): 
    target_points.sort(key = lambda pt: (pt.x-start_point.x)**2 + (pt.y-start_point.y)**2) 
    return target_points[:k]

class RelocateObstacleBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(RelocateObstacleBehavior, self).__init__(name, **params)

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
            self.robot_pose = {
                'left': '7dof_RISE_wrist_link',
                'right': '6dof_connection_link',
                'left_offset': [-0.02, 0.08],
                'right_offset': [-0.025, 0.08]
            }
        elif robot_name == 'social_robot':
            self.robot_pose = {
                'left': 'LHand_base',
                'right': 'RHand_base',
                'left_offset': [-0.005, 0.03],
                'right_offset': [-0.005, 0.03], # grasp, pre-grasp offset
                'left_shoulder': [0.0, 0.6245],
                'right_shoulder': [0.0, -0.6245] # footprint based shoulder pos
            }

        self.input_args = ['robot_group',
                            'target_object',
                            'static_object',
                            'dynamic_object']
        self.hardware_group = ['arm']

    def prepare_behavior(self):
        rospy.loginfo('preparing..%s' % self._name)
        return True

    def run_behavior(self):
        rospy.loginfo('running..%s' % self._name)
        return 1

    def get_motion(self, inputs):
        '''
        return the trajectory 
        '''
        rospy.loginfo("Calculating relocation motion.")
        res = self._call_ros_service(inputs)
        return res

    def finish_behavior(self):
        rospy.loginfo('finishing..%s' % self._name)
        return True

    def _call_ros_service(self, requirements):
        """
        requirements:
            target_object[0] : grasp target
            target_object[1] : relocate target
            target_object[2] : table
        """
        plan_res = MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

        # get relocation object info
        if len(requirements.target_object) < 2:
            rospy.logerr("Target object need minimum 2 arguments for relocation. [GRASP, RELOCATE, TABLE(option)]")
            return plan_res          

        # get re-arrange position
        ret, pos = self.get_rearrage_positions(requirements)
        if ret:
            # check whether the motion trajectory to rearrange position is exist
            plan_result, motion_plan = self.check_positions(requirements, pos)

            if not plan_result:
                rospy.logerr("[RelocteBehavior] cannot find motion to rearrange positions.")
            else:
                rospy.loginfo("[RelocteBehavior] found motion to rearrange position.")
                return motion_plan
                
            return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_NO_SOLUTION)

        return MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

    def get_rearrage_positions(self, requirements):
        """ 
        get rearrange position by rearrange service
        """

        f_check_srv = rospy.ServiceProxy("rearrange_srv", rearr_srv.rearrange_env_srv)
        # create request message of rearrange_node
        req = self.create_rearrange_request(requirements)
        self.srv_visualize(request=req, lifetime=0)
        print('rearrange requests:', req)
        resp = f_check_srv(req) 
        print('rearrange responses:', resp)

        if len(resp.rearrange_positions)>0:
            self.srv_visualize(request=req, response=resp, lifetime=0)
            
        if len(resp.rearrange_positions)<1 or requirements.target_object[2].id=='obj_fridge':
            self.srv_visualize(request=req, lifetime=1)
            rospy.logerr("[RelocteBehavior] No solution to relocate obstacle.")
            #HARD-CODING : If no solution
            temp_pos = geometry_msgs.msg.Point(x=0.55,y=-0.22,z=0.895)
            temp_list = [temp_pos]
            return True, temp_list
            
        return True, resp.rearrange_positions

    def create_rearrange_request(self, requirements):
        req = rearr_srv.rearrange_env_srvRequest()

        robot_group = requirements.robot_group[0] 
        if robot_group == socialrobot_behavior.msg.PlannerInputs.LEFT_ARM or robot_group == socialrobot_behavior.msg.PlannerInputs.LEFT_GRIPPER:
            req.robot_pose = self.robot_pose['left_shoulder']
        elif robot_group == socialrobot_behavior.msg.PlannerInputs.RIGHT_ARM or robot_group == socialrobot_behavior.msg.PlannerInputs.RIGHT_GRIPPER:
            req.robot_pose = self.robot_pose['right_shoulder']
        else:
            rospy.logerr("[RelocteBehavior] robot group for relocation behavior is unvailable.")
        
        # set relocate target
        obstacles = requirements.static_object + requirements.dynamic_object
        
        for i,obj in enumerate(obstacles):
            if obj.id == requirements.target_object[0].id:
                self.objects['target'] = obj
            elif obj.id == requirements.target_object[1].id:
                self.objects['rearrange_target'] = obj
            elif obj.id == requirements.target_object[2].id:
                self.objects['workspace'] = obj
            else:
                #TODO:distinguish static and dynamic objects
                if obj.id in ['obj_diget', 'obj_gotica', 'obj_red_gotica', 'obj_milk', 'obj_juice', 'obj_white_gotica']:
                    self.objects['obstacles'].append(obj)
        
        if self.objects['rearrange_target'] == None:
            rospy.logerr("[RelocteBehavior] No rearrange target object for relocation.")
        
        # Grasp target
        if not self.objects['target']:
            rospy.logerr("[RelocteBehavior] No target object for relocation.")
        else:
            req.target = self.get_env_object_info(self.objects['target'])

        # The obstacles on the workspace object
        req.objects.append(self.get_env_object_info(self.objects['rearrange_target']))
        for obs in self.objects['obstacles']:
            req.objects.append(self.get_env_object_info(obs))

        # The workspace object        
        if self.objects['workspace']:
            req.workspace = self.get_env_object_info(self.objects['workspace'], req.target)
        else:
            rospy.logerr("[RelocteBehavior] No workspace object for relocation.")

        return req

    def check_positions(self, requirements, positions):

        # sort by distance choose k nearest points
        sorted_positions = []

        robot_group = requirements.robot_group[0] 
        if robot_group == socialrobot_behavior.msg.PlannerInputs.LEFT_ARM or robot_group == socialrobot_behavior.msg.PlannerInputs.LEFT_GRIPPER:        
            start_point = Vector3(x=0.0, y=self.robot_pose['left_shoulder'][1])            
            sorted_positions = pClosest(start_point, positions, 5)
        elif robot_group == socialrobot_behavior.msg.PlannerInputs.RIGHT_ARM or robot_group == socialrobot_behavior.msg.PlannerInputs.RIGHT_GRIPPER:       
            start_point = Vector3(x=0.0, y=self.robot_pose['right_shoulder'][1])     
            sorted_positions = pClosest(start_point, positions, 5)

        # calculate motion to relocate position
        for pos in sorted_positions:
            plan_req = MotionPlanRequest()
            plan_req.targetBody = robot_group

            # get obstacles
            obstacles = requirements.static_object + requirements.dynamic_object
            for obs in obstacles:
                plan_req.obstacle_ids.append(obs.id)
                plan_req.obstacles.append(obs.bb3d)

            # set grasped object
            plan_req.targetObject = [requirements.target_object[1].id]

            # get bounding box of target object 
            target_bb3d = self.objects['target'].bb3d

            sample_poses = self.create_sample_poses(robot_group, pos)

            pre_eef = []
            way_eef = []
            goal_eef = []

            # calculate wrist goal pose from eef pose
            self.get_eef_poses(robot_group, sample_poses, requirements.target_object[1], pre_eef, way_eef, goal_eef)
            pre_wrist, way_wrist, goal_wrist = self.eef_to_wrist(pre_eef, way_eef, goal_eef, robot_group)

            for idx in range(len(goal_eef)):
                rospy.loginfo('[RelocteBehavior] %d/%d calculating relocate pose..', idx+1, len(goal_eef))
                
                # keep eef orientation
                #TODO: describe eef frame based 
                plan_req.orientation_constraint.position.x = 3.14
                plan_req.orientation_constraint.position.y = 0.4 
                plan_req.orientation_constraint.position.z = 0.4

                # publish goal eef pose
                self.publish_goal_eef_pose(goal_eef[idx], "approach_sample_pose", "base_footprint")

                # relocate motion
                plan_req.targetPose = [goal_wrist[idx]]
                plan_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL
                plan_res = self.srv_plan(plan_req)
                if plan_res.planResult == MotionPlanResponse.SUCCESS:
                    return True, plan_res
                else:
                    pass 
                                     
        return False, None

    def publish_goal_eef_pose(self, eef_pose, eef_frame, base_frame):
        self.talker.sendTransform((eef_pose.position.x, eef_pose.position.y, eef_pose.position.z),
            (eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z,eef_pose.orientation.w), rospy.Time.now(), eef_frame, base_frame)

    def create_sample_poses(self, robot_group, goal_pos):
        '''
        create sample grasp pose around target object
        '''
        candidate_poses = []
        rot_deg = []
        #left arm
        if robot_group is socialrobot_behavior.msg.PlannerInputs.LEFT_ARM or robot_group is socialrobot_behavior.msg.PlannerInputs.LEFT_ARM_WITHOUT_WAIST:
            rot_deg = range(0,91,15)
        #right arm
        elif robot_group is socialrobot_behavior.msg.PlannerInputs.RIGHT_ARM or robot_group is socialrobot_behavior.msg.PlannerInputs.RIGHT_ARM_WITHOUT_WAIST:
            rot_deg = range(0,-91,-15)
        #rot_deg.reverse()
        for deg in rot_deg:
            rot_pose = utils.create_pose([0, 0, 0],
                                         self.rot_z(deg))
            candidate_poses.append(utils.transform_pose(rot_pose, Pose(position=goal_pos,
                                                                        orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0))))

        return candidate_poses

    def get_eef_poses(self, robot_group, candidate_poses, target_object, pre_eef_poses, waypoint_eef_poses, goal_eef_poses):
        '''
        calculate eef poses from approaching angles and target object
        '''
        target_size = target_object.bb3d.size
        target_pos = target_object.bb3d.center.position
        target_rot = target_object.bb3d.center.orientation

        # pre_eef_poses = []
        # waypoint_eef_poses = []
        # goal_eef_poses = []

        if robot_group is socialrobot_behavior.msg.PlannerInputs.LEFT_ARM or robot_group is socialrobot_behavior.msg.PlannerInputs.LEFT_ARM_WITHOUT_WAIST:
            offset = self.robot_pose['left_offset']
        elif robot_group is socialrobot_behavior.msg.PlannerInputs.RIGHT_ARM or robot_group is socialrobot_behavior.msg.PlannerInputs.RIGHT_ARM_WITHOUT_WAIST:
            offset = self.robot_pose['right_offset']

        # gripper end-effector position from target object
        for desired_pose in candidate_poses:

            # pre-grasp
            trans_pose = utils.create_pose([-(offset[1] + target_size.x/2.0), 0, 0],
                                            [0, 0, 0, 1])                                            
            pre_grasp_pose = utils.get_grasp_pose(utils.transform_pose(trans_pose, desired_pose), robot_group)
            
            # waypoint
            trans_pose = utils.create_pose([-((offset[0]+offset[1])/2.0 + target_size.x/2.0), 0, 0],
                                            [0, 0, 0, 1])   
            waypoint_pose = utils.get_grasp_pose(utils.transform_pose(trans_pose, desired_pose), robot_group)
            
            # grasp
            trans_pose = utils.create_pose([-(offset[0] + target_size.x/2.0), 0, 0],
                                            [0, 0, 0, 1])   
            grasp_pose = utils.get_grasp_pose(utils.transform_pose(trans_pose, desired_pose), robot_group)

            pre_eef_poses.append(pre_grasp_pose)
            waypoint_eef_poses.append(waypoint_pose)
            goal_eef_poses.append(grasp_pose)

        #return pre_eef_poses, waypoint_eef_poses, goal_eef_poses
    
    def eef_to_wrist(self, pre_eef_poses, waypoint_eef_poses, goal_eef_poses, robot_group):

        # get transform between eef and wrist
        eef_to_wrist_trans = None
        eef_to_wrist_rot = None
        is_trans = False
        while is_trans == False:
            try:
                if robot_group is socialrobot_behavior.msg.PlannerInputs.LEFT_ARM or robot_group is socialrobot_behavior.msg.PlannerInputs.LEFT_ARM_WITHOUT_WAIST:
                    eef_to_wrist_trans, eef_to_wrist_rot = self.listener.lookupTransform(
                        '/left_end_effect_point', self.robot_pose['left'], rospy.Time(0))

                elif robot_group is socialrobot_behavior.msg.PlannerInputs.RIGHT_ARM or robot_group is socialrobot_behavior.msg.PlannerInputs.RIGHT_ARM_WITHOUT_WAIST:
                    eef_to_wrist_trans, eef_to_wrist_rot = self.listener.lookupTransform(
                        '/right_end_effect_point', self.robot_pose['right'], rospy.Time(0))
                if eef_to_wrist_trans != None and eef_to_wrist_rot !=None:
                    is_trans = True
            except:
                pass
            
        # vector to pose
        eef_to_wrist_pose = utils.create_pose(eef_to_wrist_trans, eef_to_wrist_rot)

        # transpose
        pre_wrist_poses = []
        waypoint_wrist_poses = []
        goal_wrist_poses = []
        for i in range(len(pre_eef_poses)):
            pre_wrist_poses.append(utils.transform_pose(eef_to_wrist_pose, pre_eef_poses[i]))
            waypoint_wrist_poses.append(utils.transform_pose(eef_to_wrist_pose, waypoint_eef_poses[i]))
            goal_wrist_poses.append(utils.transform_pose(eef_to_wrist_pose, goal_eef_poses[i]))

        return pre_wrist_poses, waypoint_wrist_poses, goal_wrist_poses

    def get_env_object_info(self, env_object, target_object=None):
        '''
        setup workspace bounding box
        '''
        workspace_object = env_object

        # if object has workspace affordance, transpose them based on robot frame  
        if target_object:  
            transformed_object = utils.transform_object(env_object)
            rospy.logerr('transformed')
            #print(transformed_object)
            for aff in transformed_object.affordance:
                if aff.id == 'workspace' or aff.id == 'table':   
                    workspace_object = aff
                    workspace_object.bb3d.size.y -= (target_object.object_scale.y + 0.05)
                    rospy.logwarn('workspace')
                    #print(workspace_object)

        return rearr_msg.env_object_info_msg(
            object_name=[workspace_object.id],
            object_position=workspace_object.bb3d.center.position,
            object_orientation=workspace_object.bb3d.center.orientation,
            object_scale=workspace_object.bb3d.size,
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
            i=0
            while(i<1000):
                marker_pub.publish(req)
                i+=1

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

        i=0
        while(i<1000):
            marker_pub.publish(req)
            i+=1