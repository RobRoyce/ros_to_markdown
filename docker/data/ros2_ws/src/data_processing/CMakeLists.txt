cmake_minimum_required(VERSION 3.8)
project(data_processing)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FilteredData.msg"
  DEPENDENCIES std_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()