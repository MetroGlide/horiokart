cmake_minimum_required(VERSION 3.5)
project(mg_diagnostics)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/diagnostics_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
