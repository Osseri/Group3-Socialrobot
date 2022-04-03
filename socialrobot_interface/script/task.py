#!/usr/bin/env python
import rospy
import rospkg
import diagnostic_msgs
from socialrobot_actionlib import msg as sactlib_msg
from socialrobot_actionlib import srv as sactlib_srv
from socialrobot_task import srv as task_srv
import interface
import yaml


class TaskInterface(interface.InterfaceBase):
    def __init__(self):
        super(TaskInterface, self).__init__()
        rospy.loginfo("Initializing TaskInterface...")

        # set the data label name for DB
        self.current_action = "/task/current_action"
        self.current_task = "/task/current_task"
        self.current_states = "/task/current_states"
        self.current_goal = "/task/current_goal"

    def generate_problem_pddl(self, current_task, file_name=""):
        if file_name:
            print('Load default pddl file.')
            # generate from script
            predicate = self._load_problem(file_name)
            problem = self._generate_problem(predicate, current_task.goal_predicates)
        else:
            # generate from knowledge
            self.createProblem(current_task)
        return self._set_problem(current_task.problem)

    def get_plan(self):
        """
        Get the result of Task planning from Task manager
        """
        return self._get_action_sequence()

    def decode_plan(self, plan, facts=[]):
        '''
        plan [list] : compound action list
        facts [list] : current predicate states list
        '''
        decodeAction = rospy.ServiceProxy(
            "/actionlib/decode_action", sactlib_srv.GetPrimitiveActionList
        )  
        #action decoding
        primitive_action_seq = []
        print('\n=======compound action========')
        for idx, action in enumerate(plan):  
            act_list = []
            act_list.append(action['compound_action'].name)
            for param in action['compound_action'].values:
                act_list.append(param)
            print(idx, act_list)

            req_act = sactlib_srv.GetPrimitiveActionListRequest()
            req_act.compound_action = action['compound_action']
            req_act.current_states = facts
            res = decodeAction(req_act)
            plan[idx]['primitive_actions'] = res.primitive_action
            for prim in res.primitive_action:
                primitive_action_seq.append(prim)

        print('\n=======primitive action========')
        for idx, action in enumerate(primitive_action_seq):
            act_list = []
            act_list.append(action.name)
            for param in action.values:
                act_list.append(param)
            print(idx, act_list)

    def _get_action_sequence(self):
        print("Waiting for the action_sequence service...")
        rospy.wait_for_service("/task_plan/get_action_sequence")
        try:
            requestActions = rospy.ServiceProxy(
                "/task_plan/get_action_sequence", task_srv.GetActionSeq
            )
            req = task_srv.GetActionSeqRequest()
            res = requestActions(req)
            print(res)
        except Exception as e:
            print("Service call failed : %s" % e)
        return res

    def _add_object(self, problem, instance_name, obj_type):
        key_value = diagnostic_msgs.msg.KeyValue()
        key_value.key = obj_type  # object type
        key_value.value = instance_name  # object arguments array
        if key_value not in problem.objects:
            problem.objects.append(key_value)

    def _add_fact(self, problem, name, args, is_negative=False):
        fact = self._make_predicate(name, args, is_negative)
        if fact not in problem.facts:
            problem.facts.append(fact)

    def _add_goal(self, problem, name, args, is_negative=False):
        pred = self._make_predicate(name, args, is_negative)
        if pred not in problem.goals:
            problem.goals.append(pred)

    def _make_predicate(self, name, args, is_negative):
        predicate = sactlib_msg.Predicate()
        predicate.name = name  # preicate name
        predicate.args = args  # predicate arguments
        predicate.is_negative = is_negative
        return predicate

    def _set_problem(self, problem):
        # service
        domain_proxy = rospy.ServiceProxy("/task_plan/set_problem", task_srv.SetProblem)
        return domain_proxy(problem)

    def _load_problem(self, domain="default"):
        rospack = rospkg.RosPack()
        task_pkg_dir = rospack.get_path("socialrobot_task")
        domain_name = domain

        file_path = "%s/pddl/example/prob_%s.yaml"%(task_pkg_dir, domain_name)
        with open(file_path, "r") as f:
            problem = yaml.load(f)
        return problem

    def _generate_problem(self, predicate, goal):
        problem = sactlib_msg.Problem()
        problem.problem_name = predicate["Demo"]["problem"]
        problem.domain_name = predicate["Demo"]["domain"]

        # add instances
        for obj in predicate["Demo"]["object"]:
            self._add_object(problem, obj[0], obj[1])
        # add attributes
        for obj in predicate["Demo"]["fact"]:
            self._add_fact(problem, obj[0], obj[1], len(obj) > 2)
        # add goals
        if goal != None:
             self._add_goal(problem, goal.name, goal.args, is_negative=goal.is_negative)
        else:
            for obj in predicate["Demo"]["goal"]:
                self._add_goal(problem, obj[0], obj[1], len(obj) > 2)
        return problem

    def createProblem(self, current_task):
        # set task domain name
        current_task.problem.problem_name = "social_task"
        current_task.problem.domain_name = "social_robot"
        for fact in current_task.problem.facts:
            for arg in fact.args:
                if 'obj_' in arg:
                    self._add_object(current_task.problem, arg, 'Object')
                elif 'pos_' in arg: 
                    self._add_object(current_task.problem, arg, 'Position')
        for goal in current_task.goal_predicates:
            self._add_goal(current_task.problem, goal.name, goal.args, is_negative=goal.is_negative)
        return

    def get_action_info(self, action):
        """get pddl action definitions of action

        Args:
            action ([socialrobot_actionlib/Action]): action

        Returns:
            [bool]: True if success
        """
        srv = rospy.ServiceProxy('/actionlib/get_action_info', sactlib_srv.GetActionInfo)

        # set action name
        req = sactlib_srv.GetActionInfoRequest()
        req.action_name = action.name
        req.params = action.values
        res = srv(req)

        return res