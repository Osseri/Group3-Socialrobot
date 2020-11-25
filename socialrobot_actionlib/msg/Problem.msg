# A problem item used to represent PDDL model
string problem_name
string domain_name

#object
diagnostic_msgs/KeyValue[] objects      #key:type , value:name

# init
socialrobot_actionlib/Predicate[] facts

# goal
socialrobot_actionlib/Predicate[] goals
