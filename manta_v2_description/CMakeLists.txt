cmake_minimum_required(VERSION 3.5)
project(manta_v2_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  rviz
  meshes
  urdf
  ros2_control
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
