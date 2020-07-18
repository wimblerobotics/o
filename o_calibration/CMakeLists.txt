cmake_minimum_required(VERSION 2.8.3)
project(o_calibration)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )
set (CMAKE_CXX_FLAGS "-g")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosconsole
  roscpp
  sensor_msgs
  std_msgs
  tf
#  o_hardware
)

find_package(Boost REQUIRED COMPONENTS
  system 
  thread
)

message(STATUS "##### 1")
#find_package(catkin 
#    REQUIRED COMPONENTS
#    o_hardware)
message(STATUS "#####2 o_hardware_INCLUDE_DIRS: ${o_hardware_INCLUDE_DIRS}")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  geometry_msgs
  o_hardware
  rosconsole 
  roscpp  
  sensor_msgs 
  std_msgs 
  LIBRARIES ${PROJECT_NAME}
)

message(STATUS "##### o_hardware_INCLUDE_DIRS: ${o_hardware_INCLUDE_DIRS}")
message(STATUS "##### o_hardware_INCLUDES: ${o_hardware_INCLUDES}")
message(STATUS "##### CATKIN_PACKAGE_INCLUDE_DESTINATION: ${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
message(STATUS "##### catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
message(STATUS "##### catkin_EXPORTED_TARGETS: ${catkin_EXPORTED_TARGETS}")

message(STATUS "##### CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
message(STATUS "##### CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "##### CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
message(STATUS "##### PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")

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
