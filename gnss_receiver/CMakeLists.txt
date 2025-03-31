cmake_minimum_required(VERSION 3.8)
project(gnss_receiver)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
#find_package(std_msgs REQUIRED)

# Add include directories
#include_directories(
#  include
#)

# install
#install(DIRECTORY scripts
#  DESTINATION share/${PROJECT_NAME}/)

# Install python modules
ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
  gnss_receiver/lonlat_to_odom.py
  gnss_receiver/combination.py
  gnss_receiver/combination_tf.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
