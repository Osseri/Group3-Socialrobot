#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
import socialrobot_perception_msgs.msg

import StringIO


if __name__ == '__main__':
    rospy.init_node("test_get_objects")

    msg_store = MessageStoreProxy()
  
    try:
        msg_object = msg_store.query_named("/vision/objects", socialrobot_perception_msgs.msg.Objects._type)

        for obj in msg_object[0].detected_objects:      
            print obj

        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
