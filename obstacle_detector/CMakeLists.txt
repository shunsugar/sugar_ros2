cmake_minimum_required(VERSION 3.8)
project(obstacle_detector)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# Add include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Add library directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

# install
install(DIRECTORY include src launch
  DESTINATION share/${PROJECT_NAME}/)

# ---------- cpp ---------- #
# Create cpp executable
add_executable(obstacle_detector src/obstacle_detector.cpp)
ament_target_dependencies(obstacle_detector
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  PCL
)

# Install cpp executable
install(TARGETS obstacle_detector
  DESTINATION lib/${PROJECT_NAME})

ament_package()
