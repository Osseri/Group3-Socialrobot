cmake_minimum_required(VERSION 2.8.3)
project(socialrobot_interface)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  diagnostic_msgs 
  vision_msgs
  actionlib_msgs
  trajectory_msgs
  socialrobot_perception_msgs
  socialrobot_task
)

add_message_files(
  FILES
  ActionLibrary.msg
)

add_service_files(
  FILES
  Task.srv
  Perception.srv
  Knowledge.srv
  Actionlib.srv
  System.srv
  Behavior.srv
)

add_action_files(
   FILES
   System.action
)

generate_messages(
    DEPENDENCIES
    std_msgs 
    diagnostic_msgs 
    vision_msgs
    trajectory_msgs
    actionlib_msgs
    socialrobot_perception_msgs
    socialrobot_behavior
    socialrobot_actionlib
    socialrobot_task
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES socialrobot_interface
#  CATKIN_DEPENDS message_generation roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

