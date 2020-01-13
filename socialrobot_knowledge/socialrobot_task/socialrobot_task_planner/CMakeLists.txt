cmake_minimum_required(VERSION 2.8.3)
project(socialrobot_task_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  diagnostic_msgs
  socialrobot_task_msgs
)

add_service_files(
  FILES
  Planning.srv
    
)

generate_messages(
  DEPENDENCIES
  std_msgs
  socialrobot_task_msgs
  diagnostic_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES catkin_planner_PDDL4J
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
