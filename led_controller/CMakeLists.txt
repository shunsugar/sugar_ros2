cmake_minimum_required(VERSION 3.8)
project(led_controller)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)

# Add include directories
include_directories(
  include
)

# install
install(DIRECTORY include src
  DESTINATION share/${PROJECT_NAME}/)

# ---------- cpp ---------- #
# Create cpp executable
add_executable(led_controller src/led_controller.cpp)
ament_target_dependencies(led_controller rclcpp std_msgs serial)

# Install cpp executable
install(TARGETS led_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
