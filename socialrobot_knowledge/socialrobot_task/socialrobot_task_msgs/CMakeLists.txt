cmake_minimum_required(VERSION 2.8.3)
project(socialrobot_task_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
    diagnostic_msgs
  message_generation
  actionlib
)

add_message_files(
    FILES
    Action.msg
    ActionSequence.msg
    Condition.msg
    Fluent.msg
    Predicate.msg
    Problem.msg
)

add_service_files(
    FILES
    GetActionSeq.srv
    RequestFile.srv
    GetProblem.srv
)

add_action_files(
    FILES 
    Dispatch.action 
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    actionlib_msgs
    diagnostic_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES socialrobot_task_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)
