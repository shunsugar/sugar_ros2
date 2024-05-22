cmake_minimum_required(VERSION 3.8)
project(simple_talker)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add include directories
include_directories(include)

# Create cpp executable
add_executable(simple_talker_string src/simple_talker_string.cpp)
ament_target_dependencies(simple_talker_string rclcpp std_msgs)

# Install cpp executable
install(TARGETS
  simple_talker_string
  DESTINATION lib/${PROJECT_NAME})

ament_package()
