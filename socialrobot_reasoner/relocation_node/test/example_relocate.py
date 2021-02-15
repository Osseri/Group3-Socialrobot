#!/usr/bin/env python

#Example run
import rospy
from relocation_node import srv as reloc_srv

robot_height = 0.075
robot_pose = [0.05, 0.1065]    #base (0,0), left(0, 0.1065), right(0,-0.1065)
target_id = 0
N = 2
R = [6.5000e-02, 6.5000e-02]  # object radius
H = [2.3544e-01, 2.3544e-01]  # object height
X = [+4.0000e-01, +3.0000e-01]  # object x position [0]:gotical, [1]: red_gotica 
Y = [-1.5003e-02, +9.9997e-02]  # object y position
x_min =0
x_max =1.5
y_min =-1
y_max =1

rospy.wait_for_service('relocation_srv')
f_check_srv = rospy.ServiceProxy("relocation_srv", reloc_srv.relocate_env_srv)
pub_msg = reloc_srv.relocate_env_srvRequest()
pub_msg.robot_height = robot_height
pub_msg.robot_pose = robot_pose
pub_msg.target_id = target_id
pub_msg.N = N
pub_msg.R = R
pub_msg.H = H
pub_msg.X = X
pub_msg.Y = Y
pub_msg.x_min = x_min
pub_msg.x_max = x_max
pub_msg.y_min = y_min
pub_msg.y_max = y_max
ret = f_check_srv(pub_msg)
accessibility = ret.accessibility
relocate_id= ret.relocate_id
relocate_coordinates = ret.relocate_coordinates

print('Target accessibility (-1=unaccessible, 0=undetected, 1=accessible): %d' % accessibility)
print('Relocate Object %d at (%f, %f)' % (relocate_id, relocate_coordinates[0], relocate_coordinates[1]))
