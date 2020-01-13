cmake_minimum_required(VERSION 2.8.3)
project(socialrobot_actionlib)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

add_message_files(
  FILES
  Action.msg
)

add_service_files(
  FILES
  GetActionList.srv
  GetPrimitiveActionList.srv
  CheckAction.srv
  GetActionInfo.srv
  GetDomain.srv

)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES socialrobot_ar
  CATKIN_DEPENDS message_runtime rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
