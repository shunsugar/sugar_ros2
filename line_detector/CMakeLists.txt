cmake_minimum_required(VERSION 3.8)
project(line_detector)

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
add_executable(line_detector src/line_detector.cpp)
ament_target_dependencies(line_detector rclcpp sensor_msgs)

# Install cpp executable
install(TARGETS line_detector
  DESTINATION lib/${PROJECT_NAME})

# Create cpp executable
add_executable(white_line_filter src/white_line_filter.cpp)
ament_target_dependencies(white_line_filter
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  PCL
)

# Install cpp executable
install(TARGETS white_line_filter
  DESTINATION lib/${PROJECT_NAME})

# Create cpp executable
add_executable(pcloud_to_lscan src/pcloud_to_lscan.cpp)
ament_target_dependencies(pcloud_to_lscan
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  PCL
)

# Install cpp executable
install(TARGETS pcloud_to_lscan
  DESTINATION lib/${PROJECT_NAME})

ament_package()
