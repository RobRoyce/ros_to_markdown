cmake_minimum_required(VERSION 3.0.2)
project(action_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  message_generation
)

add_action_files(
  FILES
  MoveToGoal.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
