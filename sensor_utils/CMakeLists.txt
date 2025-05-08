cmake_minimum_required(VERSION 3.8)
project(sensor_utils)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add include directories
include_directories(
  include
)

# install
install(DIRECTORY config include launch src
  DESTINATION share/${PROJECT_NAME}/)

# ---------- cpp ---------- #
# Create cpp executable
add_executable(imu_to_odom src/imu_to_odom.cpp)
ament_target_dependencies(imu_to_odom rclcpp sensor_msgs nav_msgs geometry_msgs)

# Install cpp executable
install(TARGETS imu_to_odom
  DESTINATION lib/${PROJECT_NAME})

ament_package()
