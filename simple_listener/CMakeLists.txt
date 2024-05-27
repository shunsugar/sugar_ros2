cmake_minimum_required(VERSION 3.8)
project(simple_listener)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add include directories
include_directories(include)

# ---------- cpp ---------- #
# Create cpp executables
add_executable(simple_listener_string src/simple_listener_string.cpp)
ament_target_dependencies(simple_listener_string rclcpp std_msgs)

# Install cpp executables
install(TARGETS
  simple_listener_string
  DESTINATION lib/${PROJECT_NAME})

# Create cpp executables
add_executable(simple_listener_twist src/simple_listener_twist.cpp)
ament_target_dependencies(simple_listener_twist rclcpp geometry_msgs)

# Install cpp executables
install(TARGETS
  simple_listener_twist
  DESTINATION lib/${PROJECT_NAME})

# Create cpp executables
add_executable(simple_listener_laserscan src/simple_listener_laserscan.cpp)
ament_target_dependencies(simple_listener_laserscan rclcpp sensor_msgs)

# Install cpp executables
install(TARGETS
  simple_listener_laserscan
  DESTINATION lib/${PROJECT_NAME})

# Create cpp executables
add_executable(simple_listener_pointcloud2 src/simple_listener_pointcloud2.cpp)
ament_target_dependencies(simple_listener_pointcloud2 rclcpp sensor_msgs)

# Install cpp executables
install(TARGETS
  simple_listener_pointcloud2
  DESTINATION lib/${PROJECT_NAME})

# ---------- python ---------- #
# Install python modules
ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
  simple_listener/simple_listener_string.py
  DESTINATION lib/${PROJECT_NAME})

# ---------- launch ---------- #
# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
