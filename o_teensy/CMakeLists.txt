cmake_minimum_required(VERSION 3.0.2)
project(o_teensy)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )
set (CMAKE_CXX_FLAGS "-g")
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rosconsole
  roscpp
  rospy
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS
  system
  thread
)

add_message_files(
  FILES
  MotorCurrent.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    libboost-thread-dev
    message_runtime
    roscpp
    std_msgs
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  teensy_node
  src/teensy_node.cpp
)

add_dependencies(
  teensy_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  teensy_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
