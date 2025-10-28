cmake_minimum_required(VERSION 3.5)
project(open_manipulator_lab)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)            # Replaces roscpp
find_package(rclpy REQUIRED)             # Replaces rospy
find_package(std_msgs REQUIRED)          # Replaces std_msgs
find_package(geometry_msgs REQUIRED)     # Replaces geometry_msgs

find_package(Boost REQUIRED COMPONENTS system)

ament_export_dependencies(
    rclcpp
    rclpy
    std_msgs
    geometry_msgs
    Boost
)

# Install Python executables
install(PROGRAMS
  scripts/orlab_lv.py
  DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
include_directories(
  include
  # ROS 2 uses the generated 'INCLUDE_DIRS' variable from the required packages
  # which are implicitly included after the package is found.
)