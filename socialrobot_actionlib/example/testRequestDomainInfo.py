#!/usr/bin/env python
import cPickle as pickle
import rospy

from socialrobot_actionlib.srv import *
from socialrobot_actionlib.msg import *


def initDomain(hardware_list):
    rospy.init_node('example')

    # service list
    rospy.wait_for_service('/actionlib/get_domain')
    get_domain_srv = rospy.ServiceProxy('/actionlib/get_domain', GetDomain)

    # set the robot hardware domain
    get_domain_req = GetDomainRequest()
    get_domain_req.group_list = hardware_list

    # deserialize the data
    domain_info = get_domain_srv(get_domain_req)
    domain_info.types = pickle.loads(domain_info.types)
    domain_info.actions = pickle.loads(domain_info.actions)
    domain_info.predicates = pickle.loads(domain_info.predicates)

    return domain_info

if __name__ == '__main__':
    
    domain_info = initDomain(['gripper'])

    if domain_info.result == True:
        # print available actions
        print domain_info

