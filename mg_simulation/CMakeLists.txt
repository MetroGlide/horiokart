cmake_minimum_required(VERSION 3.8)
project(mg_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
