cmake_minimum_required(VERSION 2.8.3)
project(o_darknet)

set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -pthread")
set (CMAKE_CXX_FLAGS "-g")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Path to Darknet project folder.
set(DARKNET_PATH /usr/lib)

# Give a custom name to shared library which is provided by Darkned when 
# the one compiled.
set(DARKNET_LIB_NAME libdarknet)

# Define Darknet library' shared library.
add_library(${DARKNET_LIB_NAME} SHARED IMPORTED)

# Set a path to Darknet library' shared library
set_property(
        TARGET ${DARKNET_LIB_NAME}
        PROPERTY IMPORTED_LOCATION 
        "${DARKNET_PATH}/libdarknet.so"
)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  rospy
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)
#find_package(DARKNET REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
  message_runtime
  rosconsole 
  roscpp 
  rospy 
  std_msgs 
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
#  include
   ${catkin_INCLUDE_DIRS}
)

add_executable (
  darknet_node
  src/darknet.cpp
)

add_dependencies(
  darknet_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  darknet_node
  ${catkin_LIBRARIES} 
  boost_system boost_thread
  ${CUDA_LIBRARIES}
#  /usr/lib/aarch64-linux-gnu/libcuda.so
#  /usr/lib/libdarknet.so
  ${DARKNET_LIB_NAME}
)
