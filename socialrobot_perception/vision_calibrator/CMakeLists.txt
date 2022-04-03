cmake_minimum_required(VERSION 3.0.2)
project(vision_calibrator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  cv_bridge
  dynamic_reconfigure
  image_geometry
  image_transport
  image_view
  tf
  marker_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/CheckerboardDetection.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision_calibrator
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ./include/
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(chess_detector src/chess_detector.cpp)
add_dependencies(chess_detector ${PROJECT_NAME}_gencfg) 

## Specify libraries to link a library or executable target against
target_link_libraries(chess_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
)