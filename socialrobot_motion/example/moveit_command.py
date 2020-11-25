#!/usr/bin/env python

import os 
import sys
import rospy
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg
from math import pi
from tf.transformations import quaternion_from_euler

class MoveitGroup(object):
  def __init__(self):
    super(MoveitGroup, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    dir_path = os.path.dirname(os.path.realpath(__file__)) + "/mesh/moveit/"
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()    
    rospy.sleep(2)

    group_left_arm_name = "left_arm"
    group_right_arm_name = "right_arm"
    group_left_hand_name = "left_eef"
    group_right_hand_name = "right_eef"
    group_left_arm = moveit_commander.MoveGroupCommander(group_left_arm_name)
    group_right_arm = moveit_commander.MoveGroupCommander(group_right_arm_name)
    group_left_hand = moveit_commander.MoveGroupCommander(group_left_hand_name)
    group_right_hand = moveit_commander.MoveGroupCommander(group_right_hand_name)
    left_eef_link = group_left_arm.get_end_effector_link()
    right_eef_link = group_right_arm.get_end_effector_link()
    
    self.robot = robot
    self.scene = scene
    self.dir_path = dir_path
    self.group_left_arm = group_left_arm
    self.group_right_arm = group_right_arm
    self.group_left_hand = group_left_hand
    self.group_right_hand = group_right_hand
    self.left_eef_link = left_eef_link
    self.right_eef_link = right_eef_link    
    self.moveit_feasible = False
    
    
  def go_to_pose_goal(self, pos, euler, direction):
    if direction == "left":
      group_arm = self.group_left_arm
    elif direction == "right":
      group_arm = self.group_right_arm
    else:
      print "Fail arm direction.."
      return -1

    try: 
      q = quaternion_from_euler(euler[0],euler[1],euler[2], axes="rzxy")    
    except:
      return -1

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]
    pose_goal.position.x = pos[0]
    pose_goal.position.y = pos[1]
    pose_goal.position.z = pos[2]
    group_arm.set_pose_target(pose_goal)
    
    try:
      self.moveit_feasible = group_arm.go(wait=True)
      group_arm.stop()
      group_arm.clear_pose_targets()
    except:
      return -1
  

  def go_gripper_grasp(self, angle, direction):
    if direction == "left":
      group_hand = self.group_left_hand
    elif direction == "right":
      group_hand = self.group_right_hand
    else:
      print "Fail gripper direction.."
      return -1

    joint_goal = group_hand.get_current_joint_values()
    
    try:
      for i in range(len(joint_goal)):
        joint_goal[i] = angle[i]
      self.moveit_feasible = group_hand.go(joint_goal, wait=True)
      group_hand.stop()
      group_hand.clear_pose_targets()
    except:
      return -1 
    
  
  def add_box(self, object_name, pos, euler, object_size):
    scene = self.scene    
    try: 
      q = quaternion_from_euler(euler[0],euler[1],euler[2], axes="rzxy")    
    except:
      return -1
    
    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.header.frame_id = "base_footprint"
    object_pose.pose.orientation.x = q[0]
    object_pose.pose.orientation.y = q[1]
    object_pose.pose.orientation.z = q[2]
    object_pose.pose.orientation.w = q[3]
    object_pose.pose.position.x = pos[0]
    object_pose.pose.position.y = pos[1]
    object_pose.pose.position.z = pos[2]    
    graspable_object_name = object_name
    size_tuple = (object_size[0], object_size[1], object_size[2])

    try:
      scene.add_box(graspable_object_name, object_pose, size = size_tuple)
    except:
      return -1

  def add_mesh(self, object_name, pos, euler = [0, 0, 0]):
    # check version : pyassimp 4.1.3
    # 1. sudo dpkg --remove --force-depends python-pyassimp
    # 2. sudo -H pip install pyassimp==4.1.3
    rospy.sleep(2)
    scene = self.scene    
    try: 
      q = quaternion_from_euler(euler[0],euler[1],euler[2], axes="rzxy")    
    except:
      return -1
    
    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.header.frame_id = "base_footprint"
    object_pose.pose.orientation.x = q[0]
    object_pose.pose.orientation.y = q[1]
    object_pose.pose.orientation.z = q[2]
    object_pose.pose.orientation.w = q[3]
    object_pose.pose.position.x = pos[0]
    object_pose.pose.position.y = pos[1]
    object_pose.pose.position.z = pos[2]    
    graspable_object_name = object_name
    graspable_object_dirc = self.dir_path + graspable_object_name + ".stl"
    scene.add_mesh(graspable_object_name, object_pose, graspable_object_dirc, size= (0.001,0.001,0.001))
    try:
      scene.add_mesh(graspable_object_name, object_pose, graspable_object_dirc, size= (0.001,0.001,0.001))
    except:
      return -1    

  
  def attach_object(self, graspable_object_name, direction):
    if direction == "left":
      group_arm = self.group_left_arm
      grasping_group = "left_hand"
      eef_link = self.left_eef_link
    elif direction == "right":
      group_arm = self.group_right_arm
      grasping_group = "right_hand"
      eef_link = self.right_eef_link      
    else:
      print "Fail gripper direction.."
      return -1

    robot = self.robot
    scene = self.scene
    
    try:
      touch_links = robot.get_link_names(group=grasping_group)
      scene.attach_box(eef_link, graspable_object_name, touch_links=touch_links)
    except:
      return -1
    

  def detach_object(self, graspable_object_name, direction):
    if direction == "left":
      group_arm = self.group_left_arm
      eef_link = self.left_eef_link
    elif direction == "right":
      group_arm = self.group_right_arm
      eef_link = self.right_eef_link      
    else:
      print "Fail gripper direction.."
      return -1
    
    scene = self.scene
    scene.remove_attached_object(eef_link, name=graspable_object_name)  
    
  
  



def main():    
  moveit_tutorial = MoveitGroup() 
  
  # add box
  x, y, z = input("Input position x,y,z : ")
  object_name = raw_input("object_name : ")
  pos = [x, y, z]
  euler = [0,0,0]
  object_size = [0.1, 0.1, 0.1]
  moveit_tutorial.add_box(object_name, pos, euler, object_size)

  
  # attach object
  dirc = raw_input("gripper direction : ")
  graspable_object_name = raw_input("grasp object : ")
  moveit_tutorial.attach_object(graspable_object_name,dirc)

  # # detach object
  # dirc = raw_input("gripper direction : ")
  # graspable_object_name = raw_input("grasp object : ")
  # moveit_tutorial.detach_object(graspable_object_name,dirc)
    
  if moveit_tutorial.moveit_feasible == True:
    moveit_tutorial.move_manipulator("left")
  else:
    print "There is no joint path"
    moveit_tutorial.moveit_feasible = True
  

    
    

if __name__ == "__main__":  
  rospy.init_node('skku_robot', anonymous=True)
  try:
    main()
  except rospy.ROSInterruptException:
    pass
