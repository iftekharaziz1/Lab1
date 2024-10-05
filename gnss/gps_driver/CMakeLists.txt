cmake_minimum_required(VERSION 3.0.2)
project(gps_driver)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
  message_generation
  sensor_msgs
)

add_message_files(
  FILES
  Customgps.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
