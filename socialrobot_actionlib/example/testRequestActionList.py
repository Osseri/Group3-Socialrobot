#!/usr/bin/env python
import rospy

from socialrobot_actionlib.srv import *
from socialrobot_actionlib.msg import *

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('example')

    # service list
    get_action_srv = rospy.ServiceProxy('/actionlib/get_action_list', GetActionList)

    # get all actions
    get_action_req = GetActionListRequest()
    get_action_req.action_type = GetActionListRequest().ALL
    get_action_res = get_action_srv(get_action_req)
    action_list = get_action_res.actions
    print 'All action list:'
    for action in action_list:
        print action.data 

    # set robot hardware list
    from testRequestDomainInfo import initDomain
    initDomain(['Arm','Gripper','Mobile'])

    # get available actions
    get_action_req.action_type = GetActionListRequest().PRIMITIVE_ACTIONS
    get_action_res = get_action_srv(get_action_req)
    action_list = get_action_res.actions
    print '\nAvailable action list:'
    for action in action_list:
        print action.data