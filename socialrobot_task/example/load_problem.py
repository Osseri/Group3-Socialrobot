import rospy
import rospkg
import rosparam
import yaml
import os
import sys

def load_problem(domain="default"):

    rospack = rospkg.RosPack()
    task_pkg_dir = rospack.get_path("socialrobot_task")
    domain_name = domain

    file_path = task_pkg_dir + '/pddl/example/prob_' + domain_name + '.yaml'
    with open(file_path, 'r') as f:
        problem = yaml.load(f) 

    return problem

if __name__ == "__main__":
    problem = load_problem("handover")

    print problem['Demo']['problem']
    print problem['Demo']['domain']
    for obj in problem['Demo']['object']:
        print obj
    for obj in problem['Demo']['fact']:
        print obj
    for obj in problem['Demo']['goal']:
        print obj