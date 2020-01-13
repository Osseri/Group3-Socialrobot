import rospy
from rosjava_custom_srv.msg import *
from rosjava_custom_srv.srv import *

rospy.init_node('context_manager_service')
context_srv = rospy.ServiceProxy("/context_manager/monitor/service", MonitorSimilarService)

req = MonitorSimilarServiceRequest()
# req.predicate = "currentObjectPerception"
# req.param1 = "Object"
# req.param2 = "Perception"
# req.param3 = "0"
# req.param4 = "0"
# req.status = 100
# req.manager = "TaskManager"

req.predicate = "currentObjectPerception"
req.param1 = "Object"
req.param2 = "Perception"
req.param3 = "0"
req.param4 = "0"
req.status = 100
req.manager = "TaskManager"

res = context_srv(req)
print res