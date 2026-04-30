cmake_minimum_required(VERSION 3.5)
project(mg_waypoint_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch behavior_trees rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_sequencer_node.py
  scripts/waypoint_editor_node.py
  scripts/migrate_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fsm_transitions test/test_fsm_transitions.py)
endif()

ament_package()
