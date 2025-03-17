cmake_minimum_required(VERSION 3.8)
project(sensor_utils)

# find dependencies
find_package(ament_cmake REQUIRED)

# install
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
