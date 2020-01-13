# A problem item used to represent PDDL model
string problem_name
string domain_name

#object
diagnostic_msgs/KeyValue[] objects      #key:type , value:name

# init
socialrobot_task_msgs/Predicate[] facts

# goal
socialrobot_task_msgs/Predicate[] goals
