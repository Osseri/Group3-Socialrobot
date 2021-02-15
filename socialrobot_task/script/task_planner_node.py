#!/usr/bin/env python

import sys
import cPickle as pickle
import rospy
import rospkg
import rosparam
from std_msgs.msg import String

from socialrobot_task.srv import *
from socialrobot_actionlib.msg import *

# import task planner modules
from ProblemGenerator import ProblemGenerator
from DomainGenerator import DomainGenerator
from TaskPlanner import TaskPlanner
from FastDownwardPlanner import FastDownwardPlanner

ros_root = rospkg.get_ros_root()
r = rospkg.RosPack()
PKG_PATH = r.get_path("socialrobot_task")
sys.path.append(PKG_PATH + "/libs")

DOMAIN_PATH = PKG_PATH + "/pddl/domain.pddl"
PROBLEM_PATH = PKG_PATH + "/pddl/problem.pddl"


class TaskManager:
    def __init__(self, plan_method):
        """
        Initialize a ROS node
        """
        # ROS services
        rospy.Service(
            "/task_plan/get_action_sequence",
            GetActionSeq,
            self._callback_get_action_sequence,
        )
        rospy.loginfo("[TaskManager] Service Started!")

        self.tp = TaskPlanner()
        self.pg = ProblemGenerator()
        self.dg = DomainGenerator()
        self.plan_method = plan_method

    def __del__(self):
        rospy.loginfo("[TaskManager] Service terminated!")

    def _get_domain_name(self):
        domain_name = ""
        if rospy.has_param("/robot_name"):
            domain_name = rospy.get_param("/robot_name")
        else:
            domain_name = "socialrobot"
        return domain_name

    def _callback_get_action_sequence(self, req):
        """
        callback for service
        """
        res = GetActionSeqResponse()
        domain = self._get_domain_name()

        # generate PDDL domain & problem files
        if not domain == "":
            self.pg.generate(domain)
            self.dg.generate(domain)
        else:
            rospy.logerr("[Task manager] Set the domain name.")
            res.plan_result = res.FAIL
            return res

        # call the task planner
        try:
            if self.plan_method == "pddl4j":
                planner = TaskPlanner()
                return planner.resolvProblem(res, DOMAIN_PATH, PROBLEM_PATH)

            elif self.plan_method == "fd":
                planner = FastDownwardPlanner()
                return planner.resolvProblem(res, DOMAIN_PATH, PROBLEM_PATH)

        except Exception as e:
            print("Service call failed : %s" % e)
            res.plan_result = res.FAIL
            return res


if __name__ == "__main__":

    # ros initialize
    rospy.init_node("task_manager")

    # Task manager
    planner = "fd"
    tm = TaskManager(planner)

    rospy.spin()
