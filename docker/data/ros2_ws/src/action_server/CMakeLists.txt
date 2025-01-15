cmake_minimum_required(VERSION 3.8)
project(action_server)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToGoal.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
