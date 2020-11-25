#!/usr/bin/env python
import rospy

from socialrobot_actionlib.srv import *
from socialrobot_actionlib.msg import *

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('decode_example')

    # service list
    srv = rospy.ServiceProxy('/actionlib/decode_action', GetPrimitiveActionList)

    # set action info
    req= GetPrimitiveActionListRequest()
    req.compound_action = socialrobot_actionlib.msg.Action()
    req.compound_action.name = 'hold_object'
    req.compound_action.parameters = ['left_arm', 'object', 'initPosition' ,'goalPosition']
    
    # get primitive actions
    res = srv(req)
    print (res)
    print ('Compound action: %s'%req.compound_action.name)
    prim = []
    for act in res.primitive_action:
        prim.append(act.name)
    print ('Primitive actions: %s'%prim)