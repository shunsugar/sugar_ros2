cmake_minimum_required(VERSION 3.8)
project(motor_listener)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add include directories
include_directories(include)

# Create cpp executable
add_executable(motor_listener src/motor_listener.cpp)
ament_target_dependencies(motor_listener rclcpp std_msgs geometry_msgs)

# Install cpp executable
install(TARGETS
  motor_listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
