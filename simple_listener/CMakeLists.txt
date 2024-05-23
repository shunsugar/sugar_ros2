cmake_minimum_required(VERSION 3.8)
project(simple_listener)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Add include directories
include_directories(include)

# Create cpp executables
add_executable(simple_listener_string src/simple_listener_string.cpp)
ament_target_dependencies(simple_listener_string rclcpp std_msgs)

# Install cpp executables
install(TARGETS
  simple_listener_string
  DESTINATION lib/${PROJECT_NAME})

ament_package()
