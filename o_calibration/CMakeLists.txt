cmake_minimum_required(VERSION 2.8.3)
project(o_calibration)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )
set (CMAKE_CXX_FLAGS "-g")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  o_msgs
  geometry_msgs
  rosconsole
  roscpp
  nav_msgs
  sensor_msgs
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS
  system 
  thread
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  geometry_msgs
  nav_msgs
  o_msgs
  rosconsole 
  roscpp  
  sensor_msgs 
  std_msgs 
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../devel/include/
  ${catkin_INCLUDE_DIRS}
)

add_executable (
  move_in_a_rectangle
  src/move_in_a_rectangle.cpp
)

add_dependencies(
  move_in_a_rectangle
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  move_in_a_rectangle
  ${catkin_LIBRARIES} 
  boost_system boost_thread
)
