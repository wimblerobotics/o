
cmake_minimum_required(VERSION 2.8.3)
project(o_common)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )
set (CMAKE_CXX_FLAGS "-g")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  rosconsole
  roslaunch
  roscpp
  rospy
  std_msgs
)

