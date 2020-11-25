#!/usr/bin/env python
import rospy
import rospkg
from socialrobot_actionlib.msg import *
from socialrobot_actionlib.srv import *
from socialrobot_task.srv import *
from interface import InterfaceBase
import yaml

class TaskInterface(InterfaceBase):
    def __init__(self):   
        super(TaskInterface, self).__init__()
        rospy.loginfo("Initializing TaskInterface...")

        # set the data label name for DB
        self.current_action = "/task/current_action"
        self.current_task = "/task/current_task"
        self.current_states = "/task/current_states"
        self.current_goal = "/task/current_goal"

        # 

    
    def generate_problem_pddl(self, file_name="default", problem=[]):
        if file_name:
            #generate from script
            predicate = self._load_problem(file_name)
            problem = self._generate_problem(predicate)
        else:
            #generate from knowledge
            problem = self.createProblem(problem)

        return self._set_problem(problem)

    def get_plan(self):
        '''
        Get the result of Task planning from Task manager
        '''                
        return self._get_action_sequence()
                 
    def decode_plan(self, compoud_action):
        decodeAction = rospy.ServiceProxy('/actionlib/decode_action', GetPrimitiveActionList)
        primitive_action_seq = []
        for idx, action in enumerate(compoud_action):
            #action decoding
            res = decodeAction(action)
            for act in res.primitive_action:
                primitive_action_seq.append(act)
        return primitive_action_seq

    def _get_action_sequence(self):
        print("Waiting for the action_sequence service...")
        rospy.wait_for_service('/task_plan/get_action_sequence')
        try:
            requestActions = rospy.ServiceProxy('/task_plan/get_action_sequence', GetActionSeq)
            req = GetActionSeqRequest()
            res = requestActions(req)

        except Exception as e:
            print("Service call failed : %s"%e)
        
        return res
        
    def _add_object(self, problem, instance_name, obj_type):
        msg = problem
        key_value = diagnostic_msgs.msg.KeyValue()
        key_value.key = obj_type		#object type
        key_value.value = instance_name	#object arguments array
        msg.objects.append(key_value)   
        return msg

    def _add_fact(self, problem, name, args, is_negative = False):
        msg = problem
        predicate = Predicate()
        predicate.name = name			#preicate name
        predicate.args = args			#predicate arguments
        predicate.is_negative = is_negative
        msg.facts.append(predicate)   
        return msg

    def _add_goal(self, problem, name, args, is_negative = False):
        msg = problem
        predicate = Predicate()
        predicate.name = name			#preicate name
        predicate.args = args			#predicate arguments
        predicate.is_negative = is_negative
        msg.goals.append(predicate)   
        return msg

    def _set_problem(self, problem):
        # service
        domain_srv = rospy.ServiceProxy('/task_plan/set_problem', SetProblem)
        return domain_srv(problem)

    def _load_problem(self, domain="default"):

        rospack = rospkg.RosPack()
        task_pkg_dir = rospack.get_path("socialrobot_task")
        domain_name = domain

        file_path = task_pkg_dir + '/pddl/example/prob_' + domain_name + '.yaml'
        with open(file_path, 'r') as f:
            problem = yaml.load(f) 

        return problem

    def _generate_problem(self, predicate):	
        problem = Problem()
        problem.problem_name = predicate['Demo']['problem']
        problem.domain_name = predicate['Demo']['domain']

        # add instances
        for obj in predicate['Demo']['object']:
            if len(obj)>2:
                self._add_object(problem, obj[0], obj[1], True)
            else:
                self._add_object(problem, obj[0], obj[1])

        # add attributes
        for obj in predicate['Demo']['fact']:
            if len(obj)>2:
                self._add_fact(problem, obj[0], obj[1], True)
            else:
                self._add_fact(problem, obj[0], obj[1])

        # add goals
        for obj in predicate['Demo']['goal']:
            if len(obj)>2:
                self._add_goal(problem, obj[0], obj[1], True)
            else:
                self._add_goal(problem, obj[0], obj[1])

        return problem

    def createProblem(self, problem):	

        
        # add instances
        #add object 
        # for obj in problem.objects:
        #     self._add_object(problem, obj.value, obj.key)

        # # add attributes
        # for pred in problem.facts:
        #     self._add_fact(problem, pred.name, pred.args)

        # # add goals
        # for goal in problem.goals:
        #     self._add_goal(problem, goal.name, goal.args)

        return problem