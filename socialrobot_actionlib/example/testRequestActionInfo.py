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
    req.action_name = 'open_container'
    req.params = ['obj_mobile', 'obj_right_hand', 'obj_juice', 'pos_mobile', 'pos_right_hand', 'pos_juice']
    res = srv(req)

    print(res)