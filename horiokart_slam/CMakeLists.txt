################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(horiokart_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(slam_toolbox REQUIRED)

include_directories(include)

add_library(posegraph_adapter STATIC
  src/posegraph_serializer/service_adapter.cpp
)
ament_target_dependencies(posegraph_adapter rclcpp nlohmann_json)
ament_target_dependencies(posegraph_adapter slam_toolbox)

add_executable(posegraph_service_adapter_node src/posegraph_serializer/service_adapter.cpp)
target_link_libraries(posegraph_service_adapter_node posegraph_adapter)
ament_target_dependencies(posegraph_service_adapter_node rclcpp nlohmann_json)
ament_target_dependencies(posegraph_service_adapter_node slam_toolbox)

################################################################################
# Install
################################################################################
install(DIRECTORY launch params rviz
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# for python nodes
################################################################################
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/actual_path_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
