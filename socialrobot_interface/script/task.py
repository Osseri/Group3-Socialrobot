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

    def generate_problem_pddl(self, file_name="", pddl_problem=[], goal=None):
        if file_name:
            # generate from script
            predicate = self._load_problem(file_name)
            problem = self._generate_problem(predicate, goal)
        else:
            # generate from knowledge
            problem = self.createProblem(pddl_problem, goal)

        return self._set_problem(problem), problem

    def get_plan(self):
        """
        Get the result of Task planning from Task manager
        """
        return self._get_action_sequence()

    def decode_plan(self, compoud_action):
        decodeAction = rospy.ServiceProxy(
            "/actionlib/decode_action", sactlib_srv.GetPrimitiveActionList
        )
        primitive_action_seq = []

        print('\n=======compound action========')
        for idx, action in enumerate(compoud_action):
            act_list = []
            act_list.append(action.name)
            for param in action.parameters:
                act_list.append(param)
            print idx, act_list

			#action decoding
            res = decodeAction(action)
            for act in res.primitive_action:
                primitive_action_seq.append(act)

        print('\n=======primitive action========')            
        for idx, action in enumerate(primitive_action_seq):
            act_list = []
            act_list.append(action.name)
            for param in action.parameters:
                act_list.append(param)
            print idx, act_list

        return primitive_action_seq

    def _get_action_sequence(self):
        print("Waiting for the action_sequence service...")
        rospy.wait_for_service("/task_plan/get_action_sequence")
        try:
            requestActions = rospy.ServiceProxy(
                "/task_plan/get_action_sequence", task_srv.GetActionSeq
            )
            req = task_srv.GetActionSeqRequest()
            res = requestActions(req)
        except Exception as e:
            print("Service call failed : %s" % e)
        return res

    def _add_object(self, problem, instance_name, obj_type):
        msg = problem
        key_value = diagnostic_msgs.msg.KeyValue()
        key_value.key = obj_type  # object type
        key_value.value = instance_name  # object arguments array
        if key_value not in msg.objects:
            msg.objects.append(key_value)
        return msg

    def _add_fact(self, problem, name, args, is_negative=False):
        msg = problem
        fact = self._make_predicate(name, args, is_negative)
        if fact not in msg.facts:
            msg.facts.append(fact)
        return msg

    def _add_goal(self, problem, name, args, is_negative=False):
        msg = problem
        pred = self._make_predicate(name, args, is_negative)
        if pred not in msg.goals:
            msg.goals.append(pred)
        return msg

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

    def createProblem(self, problem, goal_list):
        # set task domain name
        problem.problem_name = "social_task"
        problem.domain_name = "social_robot"

        for fact in problem.facts:
            for arg in fact.args:
                if 'obj_' in arg:
                    self._add_object(problem, arg, 'Object')
                elif 'pos_' in arg: 
                    self._add_object(problem, arg, 'Position')
        for goal in goal_list:
            self._add_goal(problem, goal.name, goal.args, is_negative=goal.is_negative)

        return problem

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
        req.params = action.parameters
        res = srv(req)

        return res