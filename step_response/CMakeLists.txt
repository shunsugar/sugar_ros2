cmake_minimum_required(VERSION 3.8)
project(step_response)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add include directories
include_directories(include)

# ---------- cpp ---------- #
# Create cpp executable
add_executable(step_response src/step_response.cpp)
ament_target_dependencies(step_response rclcpp geometry_msgs)

# Install cpp executable
install(TARGETS
  step_response
  DESTINATION lib/${PROJECT_NAME})

ament_package()

