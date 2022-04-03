################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(socialrobot_msgs)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  shape_msgs 
  vision_msgs 
  sensor_msgs 
  geometry_msgs
  message_generation
  socialrobot_actionlib
  diagnostic_msgs
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
   FILES
   Behavior.msg
   Position.msg
   Object.msg
   Objects.msg
   Affordance.msg
   Group.msg
 )

add_service_files(
   FILES
   SetTask.srv
   SetGoalPredicates.srv
   GetObjects.srv
   UpdateObjects.srv
   GetGroups.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  shape_msgs 
  vision_msgs 
  sensor_msgs 
  geometry_msgs
  socialrobot_actionlib
  diagnostic_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS std_msgs shape_msgs vision_msgs sensor_msgs geometry_msgs message_runtime socialrobot_actionlib diagnostic_msgs
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
