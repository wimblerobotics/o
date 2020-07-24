cmake_minimum_required(VERSION 2.8.3)
project(o_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  RoboClawStatus.msg
  Monitor.msg
)

add_service_files(
  FILES
  ResetEncoders.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)