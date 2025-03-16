cmake_minimum_required(VERSION 3.8)
project(motor_controller)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add include directories
include_directories(include)

# ---------- cpp ---------- #
# Create cpp executable
add_executable(motor_talker src/motor_talker.cpp)
ament_target_dependencies(motor_talker rclcpp geometry_msgs)

# Install cpp executable
install(TARGETS
  motor_talker
  DESTINATION lib/${PROJECT_NAME})

# Create cpp executable
add_executable(teleop_twist_sample src/teleop_twist_sample.cpp)
ament_target_dependencies(teleop_twist_sample rclcpp geometry_msgs)

# Install cpp executable
install(TARGETS
  teleop_twist_sample
  DESTINATION lib/${PROJECT_NAME})

# Create cpp executable
add_executable(motor_listener src/motor_listener.cpp)
ament_target_dependencies(motor_listener rclcpp std_msgs geometry_msgs)

# Install cpp executable
install(TARGETS
  motor_listener
  DESTINATION lib/${PROJECT_NAME})

# Create cpp executable
add_executable(motor_controller src/motor_controller.cpp)
ament_target_dependencies(motor_controller rclcpp std_msgs geometry_msgs)

# Install cpp executable
install(TARGETS
  motor_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
