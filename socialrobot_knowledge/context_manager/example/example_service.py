
import rospy
from rosjava_custom_srv.msg import *
from rosjava_custom_srv.srv import *
import time
 
PREDICATES = [
    ["on_Physical", "Top", "Bottom", "0", "0"],
    ["empty_hand", "Hand", "0", "0", "0"],
    ["opened_hand", "Hand", "0", "0", "0"],
    ["graspedBy", "Object", "Hand", "0", "0", "100"],
    ["detected_object", "Object", "0", "0", "0"],
    ["locatedAt", "Object", "Place", "0", "0"],
    ["in_ContGeneric", "Object1", "Object2", "0", "0"],
    ["aboveOf", "Object1", "Object2", "0", "0"],
    ["belowOf", "Object1", "Object2", "0", "0"],
    ["inFrontOf", "Object1", "Object2", "0", "0"],
    ["behind", "Object1", "Object2", "0", "0"],
    ["near", "Object1", "Object2", "0", "0"],
    ["empty_container", "Object", "0", "0", "0"],
    ["grasped_with_both_hand", "A", "0", "0", "0", "100"],
    ["grasped_with_one_and_both_hand", "A", "0", "0", "0", "100"],
    ["currentBothHandArtifact", "A", "0", "0", "0", "100"],
    ["currentOneAndBothHandArtifact", "A", "0", "0", "0", "100"],
]

rospy.init_node('context_manager_service')
context_srv = rospy.ServiceProxy("/context_manager/monitor/service", MonitorSimilarService)
rospy.wait_for_service("/context_manager/monitor/service", timeout=1)     
      
# query for current state predicates
req = MonitorSimilarServiceRequest()
req.status = 100
req.manager = "TaskManager"
for pred in PREDICATES:
    print("Reasoning for %s predicate." %pred[0])
    req.predicate = pred[0]
    req.param1 = pred[1]
    req.param2 = pred[2]
    req.param3 = pred[3]
    req.param4 = pred[4]
    res = context_srv(req)
    # check response is not empty
    for response in res.response:
        print(response.predicate, response.param1.replace('http://www.arbi.com/ontologies/arbi.owl#',''), 
                                response.param2.replace('http://www.arbi.com/ontologies/arbi.owl#',''), 
                                response.param3.replace('http://www.arbi.com/ontologies/arbi.owl#',''), 
                                response.param4.replace('http://www.arbi.com/ontologies/arbi.owl#',''))
    print("\n")
