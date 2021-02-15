#!/usr/bin/env python
import rospy
import rospkg

from interface import InterfaceBase
from std_msgs.msg import String
import socialrobot_interface
from socialrobot_actionlib.srv import *
from socialrobot_actionlib.msg import *
from mongodb_store.message_store import MessageStoreProxy
import cPickle as pickle

class ActionLibInterface(InterfaceBase):
    def __init__(self):   
        super(ActionLibInterface, self).__init__()
        '''
        Module for Action Library
        '''
        rospy.loginfo("Initializing ActionLibInterface...")

        # set the data label name for DB
        self.use_mongodb = False
        self.msg_store = None
        self.db_action_list = "/actionlib/action_list"
        if rospy.has_param("/use_mongodb"):
            self.use_mongodb = rospy.get_param("/use_mongodb")
        if self.use_mongodb:
            self.msg_store = MessageStoreProxy()
            self.msg_store.insert_named(self.db_action_list, socialrobot_interface.msg.ActionLibrary())

        # actionlib service list
        self.srv_check_action = rospy.ServiceProxy('/actionlib/check_action', CheckAction)
        self.srv_action_list = rospy.ServiceProxy('/actionlib/get_action_list', GetActionList)
        self.srv_action_info = rospy.ServiceProxy('/actionlib/get_action_info', GetActionInfo)
        self.srv_primitive_action = rospy.ServiceProxy('/actionlib/get_primitive_action', GetPrimitiveActionList)
        self.srv_get_domain = rospy.ServiceProxy('/actionlib/get_domain', GetDomain)

    def get_domain(self, hardware_list=["Gripper","Arm","Mobile"]):
        '''
        set robot hardware environment &
        get domain information
        '''
        rospy.loginfo("Waiting for the /actionlib/get_domain service...")
        rospy.wait_for_service('/actionlib/get_domain')

        try:
            rospy.loginfo("Connection success, getting domain...") 
            # set the robot hardware domain
            get_domain_req = GetDomainRequest()
            get_domain_req.group_list = hardware_list

            # deserialize the data
            domain_info = self.srv_get_domain(get_domain_req)
            domain_info.types = pickle.loads(domain_info.types)
            domain_info.actions = pickle.loads(domain_info.actions)
            domain_info.predicates = pickle.loads(domain_info.predicates)

        except Exception as e:
            rospy.loginfo("Service call failed : %s"%e)
            
        return domain_info

    def check_action(self, req):
        '''
        Check whether the action available
        '''
        res = socialrobot_interface.srv.ActionlibResponse()

        rospy.loginfo("Waiting for the /actionlib/check_action service...")
        rospy.wait_for_service('/actionlib/check_action')
        try:
            rospy.loginfo("Connection success, checking action...") 

            action_name = req.action[0].name

            # Knowledge
            check_req = CheckActionRequest()
            check_res = self.srv_check_action(check_req)

        except Exception as e:
            rospy.loginfo("Service call failed : %s"%e)
            
        return res

    def get_action_info(self, req):
        '''
        Transfer action information
        '''
        rospy.loginfo("Waiting for the /actionlib/get_action_info service...")
        rospy.wait_for_service('/actionlib/get_action_info')
        try:
            rospy.loginfo("Connection success, getting action info...")            

            # set action name
            req = GetActionInfoRequest()
            req.action_name = 'hold_object'

            res = self.srv_action_info(req)

        except Exception as e:
            rospy.loginfo("Service call failed : %s"%e)

        return res


    def get_action_list(self, hardware_list=["Gripper","Arm","Mobile"]):
        '''
        Callback function for action list
        '''
        rospy.loginfo("Waiting for the /actionlib/get_action_list service...")
        rospy.wait_for_service('/actionlib/get_action_list')
        try:
            # get domain
            self.get_domain(hardware_list)

            # get all actions
            get_action_req = GetActionListRequest()
            get_action_req.action_type = GetActionListRequest().ALL
            get_action_res = self.srv_action_list(get_action_req)
            action_list = get_action_res.actions

        except Exception as e:
            rospy.loginfo("Service call failed : %s"%e)

        return action_list

    def decode_plan(self, plan):
        '''
        Convert compound action sequence(symbloic) into primivitve action sequence(metric)
        '''
        primitive_action_seq = []
        act_list = []

        rospy.loginfo("Waiting for the /actionlib/decode_action service...")
        rospy.wait_for_service('/actionlib/decode_action')
        try:
            decodeAction = rospy.ServiceProxy('/actionlib/decode_action', GetPrimitiveActionList)
            rospy.loginfo("Connection success, generating action_sequence...")
            
            for idx, action in enumerate(plan):                
                #action decoding
                res = decodeAction(action)
                for act in res.primitive_action:
                    primitive_action_seq.append(act)

            return res

        except Exception as e:
            rospy.loginfo("Service call failed : %s"%e)

        return res


    def get_action_list(self):
        '''
        Request available action list from actionlib
        '''

        # get actions
        get_action_req = GetActionListRequest()
        if (1):
            get_action_req.action_type = GetActionListRequest.AVAILABLE_ACTIONS
        else:
            get_action_req.action_type = GetActionListRequest.ALL

        get_action_res = self.srv_action_list(get_action_req)
        action_list = get_action_res.actions

        return action_list
