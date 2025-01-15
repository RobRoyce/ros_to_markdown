cmake_minimum_required(VERSION 3.0.2)
project(data_processing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  FilteredData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
