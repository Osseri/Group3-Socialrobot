cmake_minimum_required(VERSION 2.8.3)
project(socialrobot_perception_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  vision_msgs
  trajectory_msgs
)

add_message_files(
  FILES
  Voice.msg
  Object.msg
  Objects.msg
)

add_service_files(
  FILES
  GetObject.srv
  GetObjects.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  vision_msgs
  trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES socialrobot_ar
  CATKIN_DEPENDS message_runtime rospy std_msgs sensor_msgs vision_msgs geometry_msgs trajectory_msgs 
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
