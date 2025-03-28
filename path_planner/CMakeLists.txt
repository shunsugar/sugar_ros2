cmake_minimum_required(VERSION 3.8)
project(path_planner)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add include directories
include_directories(
  include
)

# install
install(DIRECTORY include src config
  DESTINATION share/${PROJECT_NAME}/)

# ---------- cpp ---------- #
# Create cpp executable
add_executable(laserscan_to_costmap src/laserscan_to_costmap.cpp)
ament_target_dependencies(laserscan_to_costmap
  rclcpp
  sensor_msgs
  nav_msgs
)

# Install cpp executable
install(TARGETS laserscan_to_costmap
  DESTINATION lib/${PROJECT_NAME})

ament_package()
