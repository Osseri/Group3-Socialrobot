#!/usr/bin/env python
import rospy
from socialrobot_msgs import srv as social_robot_srv
from std_srvs.srv import *

#from arbi_msg import srv as social_robot_srv

def requestTaskClient():
	rospy.wait_for_service("/socialrobot/set_task")
	res = social_robot_srv.SetTaskResponse()
	try:
		# # reset system
		# srv = rospy.ServiceProxy("/socialrobot/reset_task", Empty)
		# req = EmptyRequest()
		# srv(req)

		# request task
		srv = rospy.ServiceProxy("/socialrobot/set_task", social_robot_srv.SetTask)
		req	= social_robot_srv.SetTaskRequest()
		req.actionName = 'handover'			# open / close / handover / grab
		req.actionParam1 = 'courier_box'
		req.actionParam2 = 'human'
		req.actionParam3 = ''
		req.actionParam4 = ''
		req.actionParam5 = ''
		req.actionConstraint = 'dual'	# left / right / dual
		req.actionID = 0
		
		res = srv(req)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)
	return res

if __name__ == '__main__':
    try:
        rospy.init_node('social_task_relocation')
        result = requestTaskClient()
        print(result)
    except rospy.ROSInterruptException:
        print("Error")