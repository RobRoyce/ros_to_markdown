cmake_minimum_required(VERSION 3.8)
project(environment_integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add Python dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EnvironmentData.msg"
  DEPENDENCIES std_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(DIRECTORY
  ${PROJECT_NAME}
  DESTINATION lib/python3.12/site-packages/
)

# Create lib directory
install(DIRECTORY
  DESTINATION lib/${PROJECT_NAME}
)

# Install msg files
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
