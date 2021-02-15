cmake_minimum_required(VERSION 2.8.3)
project(socialrobot_hardware)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  vision_msgs
  trajectory_msgs
  actionlib
  actionlib_msgs
  eigen_conversions
  nav_msgs
)

add_message_files(
  FILES
  VrepState.msg
  RobotState.msg
  ObjectInfo.msg
)

add_service_files(
  FILES
  VrepGetObstacles.srv
  VrepSetJointTrajectory.srv
  VrepSetSimState.srv
  VrepGetSimState.srv
  SetJointTrajectory.srv
  SetPathTrajectory.srv
  GetRobotState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  vision_msgs
  trajectory_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES socialrobot_ar
  CATKIN_DEPENDS message_runtime rospy std_msgs sensor_msgs vision_msgs geometry_msgs trajectory_msgs nav_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vrep_interface src/vrep_interface.cpp src/interface_controller.cpp src/interface_controller.h)
target_link_libraries(vrep_interface
   ${catkin_LIBRARIES}
 )
