################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(horiokart_drivers)

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
  "src/base/serial_communicator.cpp"

  "src/devices/motor_driver.cpp"
  "src/sensors/wheel_odometry.cpp"

  "src/motor_driver_node.cpp"
  "src/wheel_odometry_node.cpp"
)

set(DEPENDENCIES
  "rclcpp"
  "rcutils"

  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"

  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(WHEEL_ODOMETRY_EXECUTABLE_NAME "wheel_odometry_node")
add_executable(${WHEEL_ODOMETRY_EXECUTABLE_NAME} src/wheel_odometry_node.cpp)
target_link_libraries(${WHEEL_ODOMETRY_EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${WHEEL_ODOMETRY_EXECUTABLE_NAME} ${DEPENDENCIES})

set(MOTOR_DRIVER_EXECUTABLE_NAME "motor_driver_node")
add_executable(${MOTOR_DRIVER_EXECUTABLE_NAME} src/motor_driver_node.cpp)
target_link_libraries(${MOTOR_DRIVER_EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${MOTOR_DRIVER_EXECUTABLE_NAME} ${DEPENDENCIES})

################################################################################
# Install
################################################################################
install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${WHEEL_ODOMETRY_EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${MOTOR_DRIVER_EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# for python nodes
################################################################################
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/odometry_tf_broadcaster_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/pc_resource_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_package()
