cmake_minimum_required(VERSION 3.8)
project(industrial_robot_jazzy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(can_msgs REQUIRED)

# Create executable
add_executable(robot_controller
  src/robot_controller.cpp
)

add_executable(odrive_can_node
  src/odrive_can_node.cpp
)

# Specify dependencies
ament_target_dependencies(robot_controller
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

ament_target_dependencies(odrive_can_node
  rclcpp
  can_msgs
)

# Install executable
install(TARGETS robot_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS odrive_can_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and other resources
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()