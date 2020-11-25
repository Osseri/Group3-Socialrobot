#!/usr/bin/env python
#-*- coding: utf-8 -*-
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import rospy
import std_msgs
from socialrobot_perception_msgs.msg import Voice

def callback(data):
    print data.sentence.decode('utf-8') 

def publisher():
    rospy.init_node('perception_voice', anonymous=True)
    #rospy.Subscriber('/perception/voice', Voice, callback)
    pub = rospy.Publisher('/perception/voice', Voice, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    ## get object source
    data = Voice()
    data.sentence = ''
    source = ''
    
    while not rospy.is_shutdown():

        ## talker
        data.voice_id = 'human01'
        ## time
        data.stamp = rospy.Time.now() 

        # requirement :utf-8 decoding
        if rospy.has_param('~voice_source'):
            source = rospy.get_param('~voice_source')

        data.sentence = source
        #rospy.loginfo(source)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
