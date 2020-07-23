cmake_minimum_required(VERSION 2.8.3)
project(o_hardware)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )
set (CMAKE_CXX_FLAGS "-g")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  o_msgs
  controller_manager
  dynamic_reconfigure
  geometry_msgs
  hardware_interface
  nav_msgs
  rosconsole
  roscpp
  rospy
  sensor_msgs
  std_msgs
  transmission_interface
  urdf
)

find_package(Boost REQUIRED COMPONENTS system thread)

## Genrate dynamic reconfiguration files
generate_dynamic_reconfigure_options(
  cfg/motor_controller.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  o_msgs
  dynamic_reconfigure 
  geometry_msgs 
  message_runtime 
  rosconsole 
  roscpp 
  rospy 
  sensor_msgs 
  std_msgs 
  transmission_interface
  urdf
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable (
  motor_controller_node
  src/motor_controller.cpp
  src/roboclaw_controller.cpp
  src/motor_controller_node.cpp
)

add_dependencies(
  motor_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  motor_controller_node
  ${catkin_LIBRARIES} 
  boost_system boost_thread
)

## Install project namespaced headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
