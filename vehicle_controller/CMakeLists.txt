cmake_minimum_required(VERSION 3.8)
project(vehicle_controller)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# install
install(DIRECTORY include src
  DESTINATION share/${PROJECT_NAME}/)

# ---------- cpp ---------- #
# Create cpp executable
add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster rclcpp nav_msgs tf2_ros)

# Install cpp executable
install(TARGETS odom_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME})

ament_package()
