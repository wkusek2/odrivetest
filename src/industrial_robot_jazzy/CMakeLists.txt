cmake_minimum_required(VERSION 3.8)
project(industrial_robot_jazzy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)

# Create executable
add_executable(robot_controller 
  src/robot_controller.cpp
)

# Specify dependencies
ament_target_dependencies(robot_controller 
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs
  trajectory_msgs
)

# Install executable
install(TARGETS robot_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python nodes
install(PROGRAMS scripts/odrive_can.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/odrive_rviz_interface.py
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
