cmake_minimum_required(VERSION 3.5)
project(rc_teleop)

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

add_executable(rc_teleop src/rc_teleop.cpp)
ament_target_dependencies(rc_teleop rclcpp sensor_msgs tf2_geometry_msgs)

add_executable(wheel_controller src/wheel_controller.cpp)
ament_target_dependencies(wheel_controller rclcpp sensor_msgs geometry_msgs)

# Instalação de arquivos
install(TARGETS
  rc_teleop
  wheel_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
