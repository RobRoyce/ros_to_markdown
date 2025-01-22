cmake_minimum_required(VERSION 3.0.2)
project(tricky_scenarios)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  std_srvs
  message_generation
)

add_service_files(
  FILES
  GetLatestReadings.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
