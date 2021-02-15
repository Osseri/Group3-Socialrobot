#!/usr/bin/env python

import rospy
import rospkg
from std_msgs.msg import String
import diagnostic_msgs.msg

from socialrobot_task_msgs.srv import *
from socialrobot_task_msgs.msg import *
import socialrobot_interface.msg
import socialrobot_interface.srv


if __name__ == '__main__':
    #Initialize a ROS node
    rospy.init_node("testProblemGenerator")
    rospy.loginfo("ProblemGenerator trying to connect to the knowledge interface...")
    rospy.wait_for_service('/socialrobot_interface/knowledge/get_problem')
    
    #Request problem predicates
    problem_srv = rospy.ServiceProxy('/socialrobot_interface/knowledge/get_problem', socialrobot_interface.srv.Knowledge)

    get_problem_req = socialrobot_interface.srv.KnowledgeRequest()
    problem_info = problem_srv(get_problem_req).problem

    print problem_info