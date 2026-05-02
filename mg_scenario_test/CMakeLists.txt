cmake_minimum_required(VERSION 3.5)
project(mg_scenario_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/scenario_test_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch scenarios
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
