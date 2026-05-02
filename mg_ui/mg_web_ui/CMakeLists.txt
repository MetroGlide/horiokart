cmake_minimum_required(VERSION 3.5)
project(mg_web_ui)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/frontend/dist)
  install(DIRECTORY frontend/dist
    DESTINATION share/${PROJECT_NAME}/frontend
  )
endif()

install(PROGRAMS
  scripts/http_server_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
