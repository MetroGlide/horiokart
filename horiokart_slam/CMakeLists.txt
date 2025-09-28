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
find_package(Eigen3 REQUIRED)

include_directories(include)

# Export package include dir for consumers/IDE and make it explicit for targets
ament_export_include_directories(include)

add_library(posegraph_adapter STATIC
  src/posegraph_serializer/service_adapter.cpp
)
ament_target_dependencies(posegraph_adapter rclcpp nlohmann_json)
ament_target_dependencies(posegraph_adapter slam_toolbox)

add_executable(posegraph_service_adapter_node src/posegraph_serializer/service_adapter.cpp)
target_link_libraries(posegraph_service_adapter_node posegraph_adapter)
ament_target_dependencies(posegraph_service_adapter_node rclcpp nlohmann_json)
ament_target_dependencies(posegraph_service_adapter_node slam_toolbox)

# Optional KartoAdapter: builds only when explicitly enabled. Stub provided to allow builds without Karto.
option(BUILD_KARTO_ADAPTER "Build KartoAdapter (requires Karto SDK)" ON)
if(BUILD_KARTO_ADAPTER)
  # Try common locations; search for the header with the 'karto_sdk/' prefix so the
  # returned include dir is the parent include directory (e.g. /opt/ros/humble/include)
  find_path(KARTO_INCLUDE_DIR NAMES karto_sdk/Mapper.h karto_sdk/Karto.h PATHS /opt/ros/humble/include /usr/include)
  find_library(KARTO_LIB NAMES kartoSlamToolbox karto libkartoSlamToolbox PATHS /opt/ros/humble/lib /usr/lib)
  # Boost.Serialization is required by the Karto SDK (used in its headers/templates)
  find_package(Boost REQUIRED COMPONENTS serialization)
  # TBB is required by Karto for parallel runtimes
  find_package(TBB REQUIRED)

  if(NOT KARTO_INCLUDE_DIR OR NOT KARTO_LIB)
    message(FATAL_ERROR "BUILD_KARTO_ADAPTER requested but Karto SDK not found. Please install Karto headers and library under /opt/ros/humble or disable BUILD_KARTO_ADAPTER.")
  endif()

  add_library(karto_adapter STATIC src/posegraph_serializer/karto_adapter.cpp)
  target_compile_definitions(karto_adapter PRIVATE BUILD_KARTO_ADAPTER=1)
  # If find_path returned the 'karto_sdk' subdir, use its parent so includes like
  # <karto_sdk/Mapper.h> work when the include path is the parent (/opt/ros/humble/include)
  get_filename_component(KARTO_INCLUDE_PARENT ${KARTO_INCLUDE_DIR} DIRECTORY)
  # Ensure our package include/ is available to the target so headers like
  # <horiokart_slam/karto_adapter.hpp> resolve correctly in IDEs/lints.
  if(EXISTS "${KARTO_INCLUDE_PARENT}/karto_sdk/Mapper.h")
    target_include_directories(karto_adapter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${KARTO_INCLUDE_PARENT} ${EIGEN3_INCLUDE_DIR})
  else()
    target_include_directories(karto_adapter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${KARTO_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})
  endif()
  # Link Karto, Boost.Serialization and TBB into the adapter (plain signature)
  target_link_libraries(karto_adapter ${KARTO_LIB} ${Boost_LIBRARIES} TBB::tbb)
  ament_target_dependencies(karto_adapter rclcpp nlohmann_json)
  
  add_executable(karto_adapter_test src/posegraph_serializer/karto_adapter_test.cpp)
  # Link test executable against the adapter plus required DSOs
  target_link_libraries(karto_adapter_test karto_adapter ${KARTO_LIB} ${Boost_LIBRARIES} TBB::tbb)
  ament_target_dependencies(karto_adapter_test rclcpp nlohmann_json)
endif()

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
