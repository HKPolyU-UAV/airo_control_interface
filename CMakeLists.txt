cmake_minimum_required(VERSION 3.0.2)
project(airo_px4)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslaunch
  geometry_msgs
  mavros_msgs
  message_generation
  tf
)

find_package(Eigen3 REQUIRED)

set(acados_include "/home/rocky/acados/include")
set(acados_lib "/home/rocky/acados/lib")
set(quadrotor_model ${PROJECT_SOURCE_DIR}/scripts/c_generated_code)

add_message_files(
  FILES
  TakeoffLandTrigger.msg
  FSMInfo.msg
  MPCReference.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES airo_px4
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

# add_executable(airo_px4_node
#   src/airo_px4_node.cpp
#   src/airo_px4_fsm.cpp
#   src/quadrotor_mpc.cpp
#   src/rc_input.cpp
# )
# target_link_libraries(airo_px4_node
#   ${quadrotor_model}/libacados_ocp_solver_quadrotor.so
#   ${acados_lib}/libacados.so
#   ${catkin_LIBRARIES}
# )

# add_executable(example_mission_node src/example_mission_node.cpp)
# target_link_libraries(example_mission_node
#   ${catkin_LIBRARIES}
# )

# add_executable(system_identification src/system_identification.cpp)
# target_link_libraries(system_identification
#   ${catkin_LIBRARIES}
# )