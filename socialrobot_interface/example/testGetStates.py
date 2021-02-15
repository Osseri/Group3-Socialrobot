#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from socialrobot_actionlib.msg import Problem, Predicate
from socialrobot_interface.msg import *
import actionlib


if __name__ == '__main__':
    rospy.init_node("test_get_states")

    msg_store = MessageStoreProxy()
    try:
        msg_state = msg_store.query_named("/knowledge/current_state", Problem._type)

        for state in msg_state[0].facts:      
            print state
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


