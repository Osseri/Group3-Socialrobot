from six import with_metaclass
import math
import rospy
import rosparam

from socialrobot_motion.srv import *
from socialrobot_behavior.msg import *
from socialrobot_behavior.srv import *
from socialrobot_hardware.srv import *
from social_robot_dual_arm_planner.srv import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from socialrobot_msgs import msg as social_msg

import tf
import tf.transformations as tfm
from tf.transformations import quaternion_from_euler, quaternion_matrix
import numpy as np
import copy
from scipy.spatial import KDTree
from behavior import BehaviorBase


def degToRad(deg):
    rad = deg / 180.0 * math.pi
    return rad


class TransferObjectBehavior(BehaviorBase):
    ''' Joint Trajectory Following '''
    def __init__(self, name, **params):
        super(TransferObjectBehavior, self).__init__(name, **params)
        self.listener = tf.TransformListener()
        if self._hardware_if == 'vrep':
            self.service_name = '/sim_interface/set_motion'
            self.service_type = VrepSetJointTrajectory
        elif self._hardware_if == 'hw':
            self.service_name = '/hw_interface/set_motion'
            self.service_type = SetJointTrajectory
        armplan_srv = '/motion_plan/move_arm'

        self.talker = tf.TransformBroadcaster(queue_size=10)
        self.srv_plan = rospy.ServiceProxy(armplan_srv, MotionPlan)
        self.pub_pts = rospy.Publisher("/arm_workspace", MarkerArray, queue_size=10)

        # initial pose
        self.robot_config = {}
        self.approach_option = None
        self.rot_x = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (1, 0, 0))
        self.rot_y = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 1, 0))
        self.rot_z = lambda x: tfm.quaternion_about_axis(x / 180. * math.pi, (0, 0, 1))

        # get hardware
        robot_name = rosparam.get_param("/robot_name")

        if robot_name == 'skkurobot':
            self.robot_config = {
                'left': '7dof_RISE_wrist_link',
                'right': '6dof_connection_link',
                'left_offset': [-0.02, 0.08],
                'right_offset': [-0.025, 0.08],
                'left_shoulder_frame': [0,0,0],
                'right_shoulder_frame': [0,0,0],
                'arm_length': 0.6
            }
        elif robot_name == 'social_robot':
            self.robot_config = {
                'left': 'LHand_base',
                'right': 'RHand_base',
                'left_offset': [-0.0055, 0.03],
                'right_offset': [-0.0055, 0.03],
                'left_shoulder_frame': [0, 0.17025, 0.9205],
                'right_shoulder_frame': [0, -0.17025, 0.9205],
                'arm_length': 0.50
            }

        self.input_args = ['robot_group',
                            'target_object',
                            'goal_position',
                            'static_object',
                            'dynamic_object']
        self.hardware_group = ['arm']

    def prepare_behavior(self):
        rospy.loginfo('preparing...%s' % self._name)
        return True

    def run_behavior(self):
        rospy.loginfo('running...%s' % self._name)        
        return 1

    def get_motion(self, inputs):
        '''
        return the trajectory 
        '''
        rospy.loginfo("Calculating transferring motion..")
        res = self._call_ros_service(inputs)
        return res

    def finish_behavior(self):
        rospy.loginfo('finishing...%s' % self._name)
        return True

    def get_sample_degree(self, step, max_degree):
        sample_list = [0.0]
        for i in range(1, max_degree + 1, step):
            sample_list.append(i)
            sample_list.append(-i)

        return sample_list

    def _call_ros_service(self, requirements):
        try:
            plan_req = MotionPlanRequest()
            plan_res = MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

            # target body
            robot_group = requirements.robot_group[0]    
            if robot_group == MotionPlanRequest.RIGHT_GRIPPER:
                robot_group = MotionPlanRequest.RIGHT_ARM
                arm_origin = self.robot_config['right_shoulder_frame']
            elif robot_group  == MotionPlanRequest.RIGHT_ARM or robot_group == MotionPlanRequest.RIGHT_ARM_WITHOUT_WAIST:
                robot_group = requirements.robot_group[0]
                arm_origin = self.robot_config['right_shoulder_frame']
            elif robot_group == MotionPlanRequest.LEFT_GRIPPER:
                robot_group = MotionPlanRequest.LEFT_ARM  
                arm_origin = self.robot_config['left_shoulder_frame']  
            elif robot_group == MotionPlanRequest.LEFT_ARM or robot_group == MotionPlanRequest.LEFT_ARM_WITHOUT_WAIST:
                robot_group = requirements.robot_group[0]
                arm_origin = self.robot_config['left_shoulder_frame']  
            plan_req.targetBody = robot_group

            # dual arm move service 
            if robot_group == MotionPlanRequest.BOTH_ARM:
                return self.get_dualarm_move(requirements)

            # get constraints
            constraints = requirements.constraints
            for const in constraints:
                if 'slow' in constraints:
                    plan_req.acceleration_scaling_factor = 0.05
                    plan_req.velocity_scaling_factor = 0.05

            # set obstacles
            obstacles = requirements.static_object + requirements.dynamic_object
            for obs in obstacles:
                plan_req.obstacle_ids.append(obs.id)
                plan_req.obstacles.append(obs.bb3d)

            # get trasnfer object info
            if len(requirements.target_object) == 0:
                rospy.logerr("Transfer target is not decided.")
                return plan_res

            elif len(requirements.target_object) == 2:
                # transform based on footprint
                grasp_object = self._utils.transform_object(requirements.target_object[0])
                goal_object = requirements.target_object[1]
                plan_req.targetObject = [grasp_object.id]

            elif len(requirements.target_object) == 1:
                # transform based on footprint
                grasp_object = self._utils.transform_object(requirements.target_object[0])
                plan_req.targetObject = [grasp_object.id]

                # there is no goal, set static goal pose
                goal_object = social_msg.Object()
                goal_object.bb3d.center.position.x = 0.8
                goal_object.bb3d.center.position.y = 0.0
                goal_object.bb3d.center.position.z = 1.0
           
            # create workspace surface points
            points = self._create_workspace_surface_points(500, self.robot_config['arm_length'], arm_origin)
            
            # find closest points from goal position
            goal_pt = [goal_object.bb3d.center.position.x, 
                        goal_object.bb3d.center.position.y, 
                        goal_object.bb3d.center.position.z]
            closest_points = self._find_closest_points(points, goal_pt, num_pt=10)

            markers = self._xyz_array_to_marker_array(closest_points+[goal_pt], 
                                                    stamp=rospy.Time.now(), 
                                                    frame_id='base_footprint')
                                
            
            goal_pose = {'frame':'', 'poses':[]}
            aff_goal_poses = []

            # get goal pose from perception
            for aff in grasp_object.affordance:
                aff_pose = copy.deepcopy(goal_pose)
                aff_pose['frame'] = aff.id
                for gr_pt in aff.grasp_point:
                    aff_pose['poses'].append(gr_pt)
                aff_goal_poses.append(aff_pose)

            # find approaching pose
            for i, pt in enumerate(closest_points):
                self.pub_pts.publish(markers)

                # build default sample poses
                goal_eef = []
                sample_poses = self.create_sample_poses(robot_group, pt)

                # calculate wrist goal pose from eef pose
                self.get_eef_poses(robot_group, sample_poses, goal_eef)
                goal_wrist = self.eef_to_wrist(goal_eef, robot_group)

                for idx in range(len(goal_eef)):
                    rospy.loginfo('[TransferObject Behavior] %d/%d calculating pose to goal nearby...', idx+1, len(goal_eef))

                    # publish goal eef pose
                    self.publish_goal_eef_pose(goal_eef[idx], "approach_sample_pose", "base_footprint")

                    plan_req.targetPose = [goal_wrist[idx]]
                    plan_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL                

                    # keep eef orientation
                    #TODO: describe eef frame based 
                    plan_req.orientation_constraint.position.x = 3.14 #ignore this axis
                    plan_req.orientation_constraint.position.y = 0.4 #30degree
                    plan_req.orientation_constraint.position.z = 0.4

                    motion_plan = self.srv_plan(plan_req)

                    if motion_plan.planResult == MotionPlanResponse.SUCCESS:
                        rospy.loginfo('Transferring motion found!')
                        return motion_plan

            plan_req.planResult = MotionPlanResponse.ERROR_NO_SOLUTION
            return plan

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            plan_req.planResult = MotionPlanResponse.ERROR_FAIL
            return plan

    def get_eef_poses(self, robot_group, candidate_poses, goal_eef_poses):
        '''
        calculate eef poses from approaching angles and target object
        '''

        # gripper end-effector position from target object
        for desired_pose in candidate_poses:

            trans_pose = self._utils.create_pose([0, 0, 0],
                                            [0, 0, 0, 1])   
            eef_pose = self._utils.get_grasp_pose(self._utils.transform_pose(trans_pose, desired_pose), robot_group)
            goal_eef_poses.append(eef_pose)

    def eef_to_wrist(self, goal_eef_poses, robot_group):

        # get transform between eef and wrist
        eef_to_wrist_trans = None
        eef_to_wrist_rot = None
        is_trans = False
        while is_trans == False:
            try:
                if robot_group is PlannerInputs.LEFT_ARM or robot_group is PlannerInputs.LEFT_ARM_WITHOUT_WAIST:
                    eef_to_wrist_trans, eef_to_wrist_rot = self.listener.lookupTransform(
                        '/left_end_effect_point', self.robot_config['left'], rospy.Time(0))

                elif robot_group is PlannerInputs.RIGHT_ARM or robot_group is PlannerInputs.RIGHT_ARM_WITHOUT_WAIST:
                    eef_to_wrist_trans, eef_to_wrist_rot = self.listener.lookupTransform(
                        '/right_end_effect_point', self.robot_config['right'], rospy.Time(0))
                is_trans = True
            except:
                pass
            
        # vector to pose
        eef_to_wrist_pose = self._utils.create_pose(eef_to_wrist_trans, eef_to_wrist_rot)

        # transpose
        goal_wrist_poses = []
        for i in range(len(goal_eef_poses)):
            goal_wrist_poses.append(self._utils.transform_pose(eef_to_wrist_pose, goal_eef_poses[i]))

        return goal_wrist_poses

    def create_sample_poses(self, robot_group, goal_point):
        '''
        create sample grasp pose around goal position
        '''
        candidate_poses = []
        target_pose = Pose()
        target_pose.position.x = goal_point[0]
        target_pose.position.y = goal_point[1]
        target_pose.position.z = goal_point[2]
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 1.0

        if robot_group is PlannerInputs.LEFT_ARM or robot_group is PlannerInputs.LEFT_ARM_WITHOUT_WAIST:
            rot_deg = [0, -15, -30, -45, -60, -75, -90]
        elif robot_group is PlannerInputs.RIGHT_ARM or robot_group is PlannerInputs.RIGHT_ARM_WITHOUT_WAIST:
            rot_deg = [0, 15, 30, 45, 60, 75, 90]
        rot_deg.reverse()
        for deg in rot_deg:
            rot_pose = self._utils.create_pose([0, 0, 0],
                                         self.rot_z(deg))
            candidate_poses.append(self._utils.transform_pose(rot_pose, target_pose))

        return candidate_poses

    def _create_workspace_surface_points(self, n, r, origin):
        print("Generating fixed %d points on a sphere centered at the origin" %n)
        alpha = 4.0*np.pi*r*r/n
        d = np.sqrt(alpha)
        m_nu = int(np.round(np.pi/d))
        d_nu = np.pi/m_nu
        d_phi = alpha/d_nu
        count = 0

        points = []
        for m in range (0,m_nu):
            nu = np.pi*(m+0.5)/m_nu
            m_phi = int(np.round(2*np.pi*np.sin(nu)/d_phi))
            for n in range (0,m_phi):
                phi = 2*np.pi*n/m_phi
                x = r*np.sin(nu)*np.cos(phi)
                y = r*np.sin(nu)*np.sin(phi)
                z = r*np.cos(nu)            
                count = count +1
                
                points.append([x+origin[0],y+origin[1],z+origin[2]])

        return np.array(points)

    def _xyz_array_to_pointcloud2(self, points, stamp=None, frame_id=None):
        '''
        Create a sensor_msgs.PointCloud2 from an np array of points.
        '''
        msg = PointCloud2()
        if stamp:
            msg.header.stamp = stamp
        if frame_id:
            msg.header.frame_id = frame_id
        if len(points.shape) == 3:
            msg.height = points.shape[1]
            msg.width = points.shape[0]
        else:
            msg.height = 1
            msg.width = len(points)
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12*points.shape[0]
        msg.is_dense = int(np.isfinite(points).all())
        msg.data = np.asarray(points, np.float32).tostring()

        return msg
    
    def _xyz_array_to_marker_array(self, points, stamp=rospy.Time.now(), frame_id='base_footprint'):
        msg = MarkerArray()
        for id, pt in enumerate(points):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.015
            marker.scale.y = 0.015
            marker.scale.z = 0.015
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = pt[0]
            marker.pose.position.y = pt[1]
            marker.pose.position.z = pt[2]
            marker.lifetime = rospy.Duration(2.0)
            marker.ns = 'closest_point'
            marker.id = id
            msg.markers.append(marker)
        
        return msg

    def _find_closest_points(self, point_array, point, num_pt=3):
        tree = KDTree(point_array, leafsize=10)
        dist, ind = tree.query(point, k=num_pt)

        closest_points = []
        for i in ind:
            closest_points.append(list(point_array[i]))
        return closest_points

    def publish_goal_eef_pose(self, eef_pose, eef_frame, base_frame):
        self.talker.sendTransform((eef_pose.position.x, eef_pose.position.y, eef_pose.position.z),
            (eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z,eef_pose.orientation.w), rospy.Time.now(), eef_frame, base_frame)

    def get_dualarm_move(self, requirements):
        grasp_srv = rospy.ServiceProxy("/dual_arm_planning/move", DualArmMove)   # SNU method
        plan_res = MotionPlanResponse(planResult=MotionPlanResponse.ERROR_FAIL)

        plan_req = DualArmMoveRequest()
        plan_req.obj_width = 0.325

        current_joints = requirements.current_position.joint_state
        print(current_joints)
        robocare_joints = ['Waist_Roll', 'Waist_Pitch', 
        'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll', 'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll']
        joint_states = []
        for joint in robocare_joints:
            idx = current_joints.name.index(joint)
            joint_states.append(current_joints.position[idx])

        plan_req.current_joints = joint_states


        grasp_res = grasp_srv(plan_req)

        if len(grasp_res.move_trajectory.points) > 0:
            rospy.loginfo('Transferring motion found!')
            plan_res.planResult = MotionPlanResponse.SUCCESS
            plan_res.jointTrajectory = grasp_res.move_trajectory
        else:
            rospy.logerror('Transferring motion cannot found!')

        return plan_res