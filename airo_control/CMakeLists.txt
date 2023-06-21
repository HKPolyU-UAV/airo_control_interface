cmake_minimum_required(VERSION 3.0.2)
project(airo_control)

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
  message_generation
  message_runtime
  tf
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(acados_include $ENV{HOME}/acados/include)
set(acados_lib $ENV{HOME}/acados/lib)
set(quadrotor_model ${PROJECT_SOURCE_DIR}/scripts/c_generated_code)

add_message_files(
  FILES
  TakeoffLandTrigger.msg
  FSMInfo.msg
  Reference.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES airo_control
 CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${quadrotor_model}
  ${acados_include}
  ${acados_include}/blasfeo/include/
  ${acados_include}/hpipm/include/
  ${acados_include}/acados/
  ${acados_include}/qpOASES_e/
)
link_directories(${quadrotor_model})

add_executable(airo_control_node
  src/airo_control_node.cpp
  src/airo_control_fsm.cpp
  src/quadrotor_mpc.cpp
  src/rc_input.cpp
)
target_link_libraries(airo_control_node
  ${quadrotor_model}/libacados_ocp_solver_quadrotor.so
  ${acados_lib}/libacados.so
  ${catkin_LIBRARIES}
)

add_executable(system_identification src/system_identification.cpp)
target_link_libraries(system_identification
  ${catkin_LIBRARIES}
  yaml-cpp
)