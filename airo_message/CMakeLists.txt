cmake_minimum_required(VERSION 3.0.2)
project(airo_message)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  mavros_msgs
  message_runtime
  tf
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  TakeoffLandTrigger.msg
  FSMInfo.msg
  Reference.msg
  ReferencePreview.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES airo_trajectory
 CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)