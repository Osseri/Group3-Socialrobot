#!/usr/bin/env python
import roslib
roslib.load_manifest('socialrobot_motion')

import os
import os.path
import sys
import signal
import rospy
import rosparam


import moveit_commander
import moveit_msgs.msg

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('example')

    robot = moveit_commander.RobotCommander()

    group_name = "left_arm" 
    group = moveit_commander.MoveGroupCommander(group_name)

    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
        
    print "============ Printing group state"
    print group.get_active_joints()
    print ""
