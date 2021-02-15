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
    req.compound_action.name = 'open_container'       
    req.compound_action.parameters = ['obj_socialrobot', 'obj_right_hand', 'obj_fridge' ,'pos_socialrobot', 'pos_right_hand','pos_fridge']
    
    # get primitive actions
    res = srv(req)
    #print (res)
    print ('Compound action:')
    print(req.compound_action.name, req.compound_action.parameters) 
    print ('\nPrimitive actions:')
    for act in res.primitive_action:
        print(act.name, act.parameters)
   