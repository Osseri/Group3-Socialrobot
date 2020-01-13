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
    srv = rospy.ServiceProxy('/actionlib/get_action_info', GetActionInfo)

    # set action name
    req = GetActionInfoRequest()
    req.action_name = 'hold_object'

    res = srv(req)

    print res

