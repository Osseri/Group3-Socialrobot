cmake_minimum_required(VERSION 2.8.3)
project(mobile_manipulator_motion_planner CXX)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  trac_ik_lib
  kdl_parser
)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../rbdl" )
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMake" )
INCLUDE(CMake/FindRBDL.cmake)

find_package (Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)
catkin_package(
  CATKIN_DEPENDS
    kdl_parser
    trac_ik_lib
)

link_directories(${Boost_LIBRARY_DIR})
set(SRCS
    src/Controller.cpp
    src/RRT_planner2.cpp
    src/util.cpp
    src/ROS_bridge.cpp
	  src/NHRRT_planner.cpp
     )

set(HEADS
    include/Controller.h
    include/fwd.h
    include/NodeTree.h
    include/RRT_planner2.h
    include/RRTNode.h
    include/util.h
    include/ROS_bridge.hpp
    include/NHRRT_planner.h
)

add_library(mobile_manipulator_controller 
  ${SRCS} 
  ${HEADS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}
)
add_definitions (-Ddof=7)
add_executable(${PROJECT_NAME} 
  src/Main.cpp 
)

target_link_libraries(${PROJECT_NAME}
  mobile_manipulator_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  rbdl
  trajectory
  )
