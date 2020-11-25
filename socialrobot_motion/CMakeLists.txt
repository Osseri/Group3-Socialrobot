cmake_minimum_required(VERSION 2.8.3)
project(socialrobot_motion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp moveit_ros_planning_interface moveit_msgs geometry_msgs trajectory_msgs sensor_msgs vision_msgs nav_msgs
  message_generation
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_service_files(FILES
  #ArmPlanning.srv
  MotionPlan.srv)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs trajectory_msgs sensor_msgs vision_msgs nav_msgs
)


catkin_package(
)

#add_executable(planner_node src/planner_node.cpp)

#target_link_libraries(planner_node
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

#add_executable(motionplan_node src/motionplan_node.cpp src/movearm.cpp src/gripper_openclose.cpp)
#add_dependencies(motionplan_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(motionplan_node  ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
