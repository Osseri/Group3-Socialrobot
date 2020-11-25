from six import with_metaclass
from graspit_commander.graspit_commander import GraspitCommander
from graspit_commander.graspit_commander import Planner, SearchContact, SearchSpace
from geometry_msgs.msg import Pose

from socialrobot_motion.srv import *
from std_msgs.msg import *

import os
import rospy

#####################
### Gripper model ###
#####################

#  name : robot_name + "_left", "_right"
#  path : ../.graspit/models/robots/

####################
### Object model ###
####################

#  path : ../socialrobot_motion/mesh/graspit/

###################
### Input value ###
###################

#  necessary : targetBody, obstacle_ids, obstacles, targetObject
#  choose(default) : gripper_pose(current gripper pose in moveit!)

####################
### Output value ###
####################

#  planResult, endEffectorPose, graspQuality, dofs


class Singleton(type):
    '''
    for singleton pattern
    '''
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(
                Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class GraspPlanner(with_metaclass(Singleton)):
    def __init__(self, robot_name, **params):
        self.robot = robot_name
        self.object_list = {}
        self.robot_name = ''

        # object model file path
        self.dir_path = os.path.abspath(os.path.dirname(
            os.path.dirname(__file__))) + "/mesh/graspit/"

        # moveit params
        self.robot_comm = params['robot']
        group_list = self.robot_comm.get_group_names()
        self.left_arm = None
        if 'left_arm' in group_list:
            self.left_arm = self.robot_comm.get_group('left_arm')
        self.right_arm = None
        if 'right_arm' in group_list:
            self.right_arm = self.robot_comm.get_group('right_arm')

    def reset(self):
        self.object_list = {}
        # clear world
        try:
            GraspitCommander.clearWorld()
        except:
            return -1

        # load gripper model
        try:
            GraspitCommander.importRobot(self.robot_name)
        except:
            rospy.logerr('[Grasp planner] can not import robot model..')
            return -1

        return 0

    def load_model(self, name, obstacle_boundingboxes, graspable=False):
        object_dir_path = self.dir_path + name + ".xml"
        if graspable:
            try:
                try:
                    GraspitCommander.importGraspableBody(object_dir_path)
                except:
                    self.add_box(name, obstacle_boundingboxes.size, graspable)
            except:
                rospy.logerr('[Grasp planner] can not import %s..' % name)
                return -1

            _bodies = GraspitCommander.getGraspableBodies()
            self.object_list[name] = (_bodies.ids[-1], 'graspable')
        else:
            try:
                try:
                    GraspitCommander.importObstacle(object_dir_path)
                except:
                    self.add_box(name, obstacle_boundingboxes.size, graspable)
            except:
                rospy.logerr('[Grasp planner] can not import %s..' % name)
                return -1

            _bodies = GraspitCommander.getBodies()
            self.object_list[name] = (_bodies.ids[-1], 'obstacle')

        return 0

    def add_box(self, name, size, graspable):
        box_xml_dir_path = self.dir_path + "box.xml"
        box_wrl_dir_path = self.dir_path + "box.wrl"

        # read box.wrl file
        list_file = open(box_wrl_dir_path, 'r').read().split('\t')

        # change scale (size unit : m)
        list_file[1] = 'scale         ' + str(size.x*10**3) + ' ' + \
            str(size.y*10**3) + ' ' + str(size.z*10**3) + ' ' + '\r\n'

        # overwrite file
        file = open(box_wrl_dir_path, 'w')
        for i in range(len(list_file)):
            file.write(list_file[i] + '\t')
        file.close()

        # load box model
        if graspable:
            GraspitCommander.importGraspableBody(box_xml_dir_path)
        else:
            GraspitCommander.importObstacle(box_xml_dir_path)

    def setRobotPose(self, pose):
        if pose == Pose():
            return -1
        try:
            GraspitCommander.setRobotPose(pose, 0)
        except:
            return -1
        return 0

    def setObjectPose(self, name, pose):
        idx, object_type = self.object_list.get(name, (-1, -1))
        if idx < 0:
            return -1
        try:
            if object_type == 'graspable':
                GraspitCommander.setGraspableBodyPose(idx, pose)
            else:
                GraspitCommander.setBodyPose(idx, pose)
        except:
            return -1
        return 0

    def get_gripperstate_from_moveit(self, target_gripper, gripper_pose):
        pose_res = None
        self.robot_name = self.robot
        if target_gripper == MotionPlanRequest.LEFT_GRIPPER:
            self.robot_name = self.robot + "_left"
            if self.left_arm:
                pose_res = self.left_arm.get_current_pose()
        elif target_gripper == MotionPlanRequest.RIGHT_GRIPPER:
            self.robot_name = self.robot + "_right"
            if self.right_arm:
                pose_res = self.right_arm.get_current_pose()

        if gripper_pose == Pose():
            return pose_res.pose
        else:
            return gripper_pose

    def callback_plan_grasp(self, req):
        rospy.loginfo("[Grasp planner] planning..")
        res = MotionPlanResponse()

        ### set params ###
        target_gripper = req.targetBody
        obstacle_list = req.obstacle_ids
        obstacle_boundingboxes = req.obstacles
        target_object = req.targetObject
        gripper_pose = self.get_gripperstate_from_moveit(
            target_gripper, req.gripper_pose)

        ### environment reset ###
        ret = self.reset()

        #### set gripper pose ###
        ret += self.setRobotPose(gripper_pose)

        ### set environment ###
        for i, name in enumerate(obstacle_list):
            graspable = False
            if name == target_object:
                graspable = True

            # load object
            ret += self.load_model(name,
                                   obstacle_boundingboxes[i], graspable=graspable)
            # set object pose
            obstacle_pose = obstacle_boundingboxes[i].center
            ret += self.setObjectPose(name, obstacle_pose)

        ### check error ###
        if ret < 0:
            res.planResult = MotionPlanResponse.ERROR_FAIL
            return res

        ### set graspit option ###
        GraspitCommander.toggleAllCollisions(True)
        GraspitCommander.approachToContact()

        ### planning ###
        planner = Planner(Planner.SIM_ANN)
        se = "GUIDED_POTENTIAL_QUALITY_ENERGY"
        ss = SearchSpace(SearchSpace.SPACE_APPROACH)
        # ss = SearchSpace(SearchSpace.SPACE_COMPLETE)
        # sc = SearchContact(SearchContact.CONTACT_LIVE)
        sc = SearchContact(SearchContact.CONTACT_PRESET)
        ms = 35000
        # ms = 40000

        grasp_res = GraspitCommander.planGrasps(planner=planner,
                                                search_space=ss,
                                                search_energy=se,
                                                search_contact=sc,
                                                max_steps=ms)

        # return result
        _grasps = grasp_res.grasps
        if _grasps:
            res.planResult = MotionPlanResponse.SUCCESS
        else:
            res.planResult = MotionPlanResponse.ERROR_NO_SOLUTION

        for i, _grasp in enumerate(_grasps):
            _dofs = _grasp.dofs
            # if _grasp.epsilon_quality > -1:
            if i == 0:
                res.dofs.layout.dim.append(MultiArrayDimension(
                    'length', len(_grasps), len(_grasps)*len(_dofs)))
                res.dofs.layout.dim.append(MultiArrayDimension(
                    'dofs_size', len(_dofs), len(_dofs)))
            res.endEffectorPose.append(_grasp.pose)
            res.graspQuality.append(_grasp.epsilon_quality)
            res.dofs.data += list(_dofs)

        rospy.loginfo("[Grasp planner] planning is over..")
        return res
