cmake_minimum_required(VERSION 3.5)
project(laser_rahcm_teleop)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)  
find_package(ignition-common3 REQUIRED)

# ------------ Teleop Manager --------------- #
add_executable(TeleopManager src/TeleopManager.cc)
ament_target_dependencies(TeleopManager rclcpp sensor_msgs std_msgs)

# ------------ Teleop Aquatic mode  --------------- #
add_executable(ThrusterController src/ThrusterController.cc)
ament_target_dependencies(ThrusterController rclcpp sensor_msgs geometry_msgs)

# ------------ Teleop Ground mode  --------------- #
add_executable(WhellController src/WhellController.cc)
ament_target_dependencies(WhellController rclcpp sensor_msgs geometry_msgs)

# ------------ Wheel Plugin --------------- #
add_library(usv_gazebo_wheel_plugin SHARED
  src/usv_gazebo_wheel_plugin.cc
  include/laser_rahcm_teleop/usv_gazebo_wheel_plugin.hh
)
target_include_directories(usv_gazebo_wheel_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(usv_gazebo_wheel_plugin
  rclcpp
  std_msgs
  sensor_msgs
  gazebo_ros
  gazebo_dev
)

install(TARGETS 
  TeleopManager
  ThrusterController
  WhellController
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  usv_gazebo_wheel_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()