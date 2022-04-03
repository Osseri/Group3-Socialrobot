#!/usr/bin/env python
import rospy
from socialrobot_interface import srv as interface_srv
from socialrobot_msgs import srv as social_srv
from socialrobot_actionlib import msg as actionlib_msg
from std_srvs.srv import *

def requestTaskClient():
	rospy.wait_for_service("/socialrobot/set_goal_predicates")
	res = social_srv.SetGoalPredicatesResponse()
	try:
		# reset system
		srv = rospy.ServiceProxy("/socialrobot/reset_task", Empty)
		req = EmptyRequest()
		srv(req)

		# request task
		srv = rospy.ServiceProxy("/socialrobot/set_goal_predicates", social_srv.SetGoalPredicates)
		req	= social_srv.SetGoalPredicatesRequest()

		# add predicates
		# predicate = actionlib_msg.Predicate()
		# predicate.name = 'inContGeneric'
		# predicate.args = ['obj_mug','obj_coffee']
		# predicate.is_negative = False
		# req.goal_predicates.append(predicate)	

		predicate = actionlib_msg.Predicate()
		predicate.name = 'belowOf'
		predicate.args = ['obj_tray', 'obj_white_gotica']
		predicate.is_negative = False
		req.goal_predicates.append(predicate)	

		predicate = actionlib_msg.Predicate()
		predicate.name = 'graspedBy'
		predicate.args = ['obj_dual_hand', 'obj_tray']
		predicate.is_negative = False
		req.goal_predicates.append(predicate)	

		predicate = actionlib_msg.Predicate()
		predicate.name = 'delivered'
		predicate.args = ['obj_tray', 'obj_human']
		predicate.is_negative = False
		req.goal_predicates.append(predicate)	

		# predicate = actionlib_msg.Predicate()
		# predicate.name = 'delivered'
		# predicate.args = ['obj_courier_box', 'obj_human']
		# predicate.is_negative = False
		# req.goal_predicates.append(predicate)	

		# predicate = actionlib_msg.Predicate()
		# predicate.name = 'graspedBy'
		# predicate.args = ['obj_right_hand', 'obj_milk']
		# predicate.is_negative = False
		# req.goal_predicates.append(predicate)	

		# predicate = actionlib_msg.Predicate()
		# predicate.name = 'graspedBy'
		# predicate.args = ['obj_right_hand', 'obj_gotica']
		# predicate.is_negative = False
		# req.goal_predicates.append(predicate)	

		# predicate = actionlib_msg.Predicate()
		# predicate.name = 'transferred'
		# predicate.args = ['obj_juice', 'obj_human']
		# predicate.is_negative = False
		# req.goal_predicates.append(predicate)			

		# predicate = actionlib_msg.Predicate()
		# predicate.name = 'openedHand'
		# predicate.args = ['obj_dual_hand']
		# predicate.is_negative = True
		# req.goal_predicates.append(predicate)


		res = srv(req)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)
	return res

if __name__ == '__main__':
    try:
        rospy.init_node('test_node')
        result = requestTaskClient()
        print(result)
    except rospy.ROSInterruptException:
        print("Error")