cmake_minimum_required(VERSION 3.0.2)
project(airo_trajectory)
#lala
## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  roslaunch
  geometry_msgs
  mavros_msgs
  message_runtime
  tf
  airo_control
)

find_package(Eigen3 REQUIRED)

# set(acados_include $ENV{HOME}/acados/include)
# set(acados_lib $ENV{HOME}/acados/lib)
# set(quadrotor_model ${PROJECT_SOURCE_DIR}/scripts/c_generated_code)

# add_message_files(
#   FILES
#   TakeoffLandTrigger.msg
#   FSMInfo.msg
#   Reference.msg
# )

# generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES airo_trajectory
 CATKIN_DEPENDS roscpp message_runtime airo_control
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(example_mission_node
    src/example_mission_node.cpp
    src/airo_trajectory_server.cpp)
target_link_libraries(example_mission_node
  ${catkin_LIBRARIES}
)