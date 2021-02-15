import rospy
from rosjava_custom_srv.msg import *
from rosjava_custom_srv.srv import *
from socialrobot_actionlib.msg import Problem, Predicate

class ContextKnowledge():
    predicates = [
    ['on_Physical', 'Top', 'Bottom', '0', '0'],
    ['empty_hand', 'Hand', '0', '0', '0'],
    ['opened_hand', 'Hand', '0', '0', '0'],
    ['graspedBy', 'Object', 'Hand', '0', '0'],
    ['detected_object', 'Object', '0', '0', '0'],
    ['locatedAt', 'Object', 'Place', '0', '0'],
    ['in_ContGeneric', 'Object1', 'Object2', '0', '0'],
    ['aboveOf', 'Object1', 'Object2', '0', '0'],
    ['belowOf', 'Object1', 'Object2', '0', '0'],
    ['inFrontOf', 'Object1', 'Object2', '0', '0'],
    ['behind', 'Object1', 'Object2', '0', '0'],
    ['near', 'Object1', 'Object2', '0', '0'],
    ['empty_container', 'Object', '0', '0', '0']]
    
    def __init__(self):
        context_topic = "/context_manager/monitor/provision_for_tm"
        #rospy.Subscriber(context_topic, MonitorServiceResponse, self._callback_state, queue_size=10)
        self.context_srv = rospy.ServiceProxy("/context_manager/monitor/service", MonitorSimilarService)


    def update(self):
        current_states = Problem()

        # query for current state predicates
        req = MonitorSimilarServiceRequest()
        req.status = 100
        req.manager = "TaskManager"
        for pred in self.predicates:   
            print("Reasoning for %s predicate." %pred[0])
            req.predicate = pred[0]
            req.param1 = pred[1]
            req.param2 = pred[2]
            req.param3 = pred[3]
            req.param4 = pred[4]            
            res = self.context_srv(req)

            # check response is not empty
            for response in res.response:
                state = self._convert_format(response)
                if 'obj_bakey' not in state.args and 'pos_bakey' not in state.args:
                    current_states.facts.append(state) 

        print(current_states)
        

    def _callback_state(self, data):
        pred = data.predicate
        return

    def _convert_format(self, state):
        # convert data format    
        pred = Predicate()    
        pred.name = state.predicate    
        if "_" in pred.name:
            pred.name = pred.name.replace("_",'')

        for i, j in enumerate([state.param1,state.param2,state.param3,state.param4]):
            if "'http://www.arbi.com/ontologies/arbi.owl#" in j:
                j = j.replace("'http://www.arbi.com/ontologies/arbi.owl#",'obj_')
            if "_1'" in j:
                j = j.replace("_1'",'')
            if "'" in j:
                j = j.replace("'",'')
            # if "[" in j and "]" in j:
            #     j = j.replace("[",'')
            #     j = j.replace("]",'')
            #     j = j.split(',')
            #     for idx, string in enumerate(j):
            #         j[idx] = float(string)
            if j:
                pred.args.append(j)
        # exeption only for 'locatedAt'   
        if pred.name=='locatedAt':  
            pred.args[1] = pred.args[0].replace('obj','pos')
        return pred

if __name__ == "__main__":
    rospy.init_node("state_subscrber")
    ck = ContextKnowledge()
    ck.update()

