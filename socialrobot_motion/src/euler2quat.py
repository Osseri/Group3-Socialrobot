#!/usr/bin/python

import sys
import rospy
import math
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
  
if __name__ == '__main__':

  if len(sys.argv) == 4:
    print "your input:", sys.argv[1], sys.argv[2], sys.argv[3]
    r = math.radians(float(sys.argv[1]))
    p = math.radians(float(sys.argv[2]))
    y = math.radians(float(sys.argv[3]))
    # RPY to convert: 90deg, 0, -90deg
    q = quaternion_from_euler(r,p,y)

    print "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])
