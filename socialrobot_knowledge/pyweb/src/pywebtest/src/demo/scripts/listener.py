#!/usr/bin/env python 

import rospy
from std_msgs.msg import String
import requests
import json
from flask import jsonify
from rosjava_custom_srv.msg import Person

def callback(data):
    s=json.dumps({"1":"2"})
    #s={'1':'2','3':'4'}
    d={"5":"5"}
    session = requests.session()
    res = session.post("http://127.0.0.1:5000", json=s,data=d)#.json()
    
    
    rospy.loginfo("objName: %s" % res)#data)
    

def listener():
    rospy.init_node('web_test')
    rospy.Subscriber("realtime_display", String, callback)

    # spin() simply keeps python from exiting until this node is stopped

    rospy.spin()


if __name__=='__main__':
      listener()

