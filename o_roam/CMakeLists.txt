cmake_minimum_required(VERSION 2.8.3)
project(o_roam)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )
set (CMAKE_CXX_FLAGS "-g")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  o_msgs
  geometry_msgs
  nav_msgs
  rosconsole
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
  o_msgs
  geometry_msgs 
  message_runtime
  rosconsole 
  roscpp 
  rospy 
  sensor_msgs 
  std_msgs 
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
#  include
   ${catkin_INCLUDE_DIRS}
)

add_executable (
  roam_node
  src/roam_node.cpp
)

add_dependencies(
  roam_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  roam_node
  ${catkin_LIBRARIES} 
  boost_system boost_thread
)
