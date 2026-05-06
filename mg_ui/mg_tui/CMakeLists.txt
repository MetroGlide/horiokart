cmake_minimum_required(VERSION 3.5)
project(mg_tui)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(mg_tui)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/tui_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
