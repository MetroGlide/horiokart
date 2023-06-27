################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(horiokart_navigation)

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

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/waypoint_navigator.cpp"
)

set(DEPENDENCIES
  "rclcpp"
  "rcutils"

  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"

  "rclcpp_action"
  "rclcpp_lifecycle"
  "rclcpp_components"

  "nav2_core"
  "nav2_common"
  "nav2_msgs"
  "nav2_util"

  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(WAYPOINT_NAV_EXECUTABLE_NAME "waypoint_navigator_node")
add_executable(${WAYPOINT_NAV_EXECUTABLE_NAME} src/waypoint_navigator.cpp)
target_link_libraries(${WAYPOINT_NAV_EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${WAYPOINT_NAV_EXECUTABLE_NAME} ${DEPENDENCIES})


################################################################################
# Install
################################################################################
install(DIRECTORY launch params rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${WAYPOINT_NAV_EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# for python nodes
################################################################################
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/waypoint_editor_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/waypoints_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_package()
