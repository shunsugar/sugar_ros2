cmake_minimum_required(VERSION 3.8)
project(line_detector)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add include directories
include_directories(include)

# install
install(DIRECTORY launch src
  DESTINATION share/${PROJECT_NAME}/)

# ---------- cpp ---------- #
# Create cpp executable
add_executable(line_detector src/line_detector.cpp)
ament_target_dependencies(line_detector rclcpp sensor_msgs)

# Install cpp executable
install(TARGETS
  line_detector
  DESTINATION lib/${PROJECT_NAME})

ament_package()
