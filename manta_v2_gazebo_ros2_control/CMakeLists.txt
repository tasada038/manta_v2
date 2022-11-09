cmake_minimum_required(VERSION 3.5)
project(manta_v2_gazebo_ros2_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

# example executable
add_executable(example_position src/example_position.cpp)
ament_target_dependencies(example_position
  rclcpp
  rclcpp_action
  control_msgs
)

# wing motion executable
add_executable(wing_init src/wing_init.cpp)
ament_target_dependencies(wing_init
  rclcpp
  rclcpp_action
  control_msgs
)
add_executable(wing_motion src/wing_motion.cpp)
ament_target_dependencies(wing_motion
  rclcpp
  rclcpp_action
  control_msgs
)

# tripod motion executable
add_executable(tripod_init src/tripod_init.cpp)
ament_target_dependencies(tripod_init
  rclcpp
  rclcpp_action
  control_msgs
)
add_executable(tripod_motion src/tripod_motion.cpp)
ament_target_dependencies(tripod_motion
  rclcpp
  rclcpp_action
  control_msgs
)
add_executable(tripod_rear src/tripod_rear.cpp)
ament_target_dependencies(tripod_rear
  rclcpp
  rclcpp_action
  control_msgs
)
add_executable(tripod_rotate src/tripod_rotate.cpp)
ament_target_dependencies(tripod_rotate
  rclcpp
  rclcpp_action
  control_msgs
)

# quadruped motion executable
add_executable(quadruped_torotto src/quadruped_torotto.cpp)
ament_target_dependencies(quadruped_torotto
  rclcpp
  rclcpp_action
  control_msgs
)
add_executable(quadruped_torotto_rear src/quadruped_torotto_rear.cpp)
ament_target_dependencies(quadruped_torotto_rear
  rclcpp
  rclcpp_action
  control_msgs
)
add_executable(quadruped_torotto_rotate src/quadruped_torotto_rotate.cpp)
ament_target_dependencies(quadruped_torotto_rotate
  rclcpp
  rclcpp_action
  control_msgs
)
add_executable(quadruped_pace src/quadruped_pace.cpp)
ament_target_dependencies(quadruped_pace
  rclcpp
  rclcpp_action
  control_msgs
)

## Install
install(
  TARGETS 
  example_position
  wing_init
  wing_motion
  tripod_init
  tripod_motion
  tripod_rear
  tripod_rotate
  quadruped_torotto
  quadruped_torotto_rear
  quadruped_torotto_rotate
  quadruped_pace
  DESTINATION 
  lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  launch
  worlds
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
