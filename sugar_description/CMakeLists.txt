cmake_minimum_required(VERSION 3.8)
project(sugar_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# install
install(DIRECTORY urdf launch rviz2
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
