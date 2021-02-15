#!/usr/bin/env python
import rospy

from socialrobot_actionlib.srv import *
from socialrobot_actionlib.msg import *
import cPickle as pickle

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('example')

    # service list
    srv = rospy.ServiceProxy('/actionlib/get_action_info', GetActionInfo)

    # set action name
    req = GetActionInfoRequest()
    req.action_name = 'relocate_obstacle'
    req.params = ['obj_left_hand', 'obj_gotica', 'obj_red_gotica', 'pos_red_gotica', 'relocateposition']
    res = srv(req)

    print(res)