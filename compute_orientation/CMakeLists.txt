cmake_minimum_required(VERSION 3.8)
project(compute_orientation)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add include directories
include_directories(include)

# ---------- cpp ---------- #
# Create cpp executable
add_executable(imu_node_gyro src/imu_node_gyro.cpp)
ament_target_dependencies(imu_node_gyro rclcpp sensor_msgs)

# Install cpp executable
install(TARGETS
  imu_node_gyro
  DESTINATION lib/${PROJECT_NAME})

# Create cpp executable
add_executable(imu_node_gyro_accel src/imu_node_gyro_accel.cpp)
ament_target_dependencies(imu_node_gyro_accel rclcpp sensor_msgs)

# Install cpp executable
install(TARGETS
  imu_node_gyro_accel
  DESTINATION lib/${PROJECT_NAME})

ament_package()
