cmake_minimum_required(VERSION 3.8)
project(horiokart_depth_camera_costmap)

set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "$ORIGIN")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pcl_conversions QUIET)
find_package(tf2_eigen QUIET)
find_package(ament_cmake_gtest QUIET)
find_package(PCL QUIET COMPONENTS common kdtree io)
find_package(Eigen3 QUIET)

include_directories(
  include
)

# Core library (ROS-independent processing)
add_library(horiokart_depth_camera_core
  src/core_processor.cpp
)
target_include_directories(horiokart_depth_camera_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(Eigen3_FOUND)
  # use plain signature to avoid mixing keyword/plain forms across CMake
  target_link_libraries(horiokart_depth_camera_core Eigen3::Eigen)
endif()
if(PCL_FOUND)
  target_include_directories(horiokart_depth_camera_core PUBLIC ${PCL_INCLUDE_DIRS})
  target_link_libraries(horiokart_depth_camera_core ${PCL_LIBRARIES})
endif()

# depth camera costmap layer plugin (optional ROS-dependent layer)
add_library(depth_camera_costmap_layer SHARED
  src/depth_camera_costmap_layer.cpp
)
target_include_directories(depth_camera_costmap_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(depth_camera_costmap_layer
  rclcpp sensor_msgs nav_msgs visualization_msgs tf2_ros pluginlib nav2_costmap_2d
)
if(pcl_conversions_FOUND)
  ament_target_dependencies(depth_camera_costmap_layer pcl_conversions)
endif()
if(tf2_eigen_FOUND)
  ament_target_dependencies(depth_camera_costmap_layer tf2_eigen)
endif()

# link core library (use plain signature)
target_link_libraries(depth_camera_costmap_layer horiokart_depth_camera_core)
if(PCL_FOUND)
  target_link_libraries(depth_camera_costmap_layer ${PCL_LIBRARIES})
endif()

pluginlib_export_plugin_description_file(nav2_costmap_2d plugin.xml)

install(TARGETS depth_camera_costmap_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS horiokart_depth_camera_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# nodes
add_executable(depth_camera_processor_node src/depth_camera_processor_node.cpp)
ament_target_dependencies(depth_camera_processor_node
  rclcpp sensor_msgs nav_msgs visualization_msgs tf2_ros
)
if(pcl_conversions_FOUND)
  ament_target_dependencies(depth_camera_processor_node pcl_conversions)
endif()
if(tf2_eigen_FOUND)
  ament_target_dependencies(depth_camera_processor_node tf2_eigen)
endif()

target_include_directories(depth_camera_processor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# link core with plain signature
target_link_libraries(depth_camera_processor_node horiokart_depth_camera_core)
if(PCL_FOUND)
  target_link_libraries(depth_camera_processor_node ${PCL_LIBRARIES})
endif()

install(TARGETS depth_camera_processor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(costmap_adapter_node src/costmap_adapter_node.cpp)
ament_target_dependencies(costmap_adapter_node
  rclcpp nav_msgs
)
install(TARGETS costmap_adapter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Tests
if(AMENT_CMAKE_GTEST_FOUND OR ament_cmake_gtest_FOUND)
  ament_add_gtest(test_core_processor
    test/test_core_processor.cpp
  )
  if(TARGET test_core_processor)
    target_include_directories(test_core_processor PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_core_processor horiokart_depth_camera_core)
  endif()
endif()

set_target_properties(depth_camera_costmap_layer PROPERTIES
  BUILD_WITH_INSTALL_RPATH FALSE
  BUILD_RPATH "$ORIGIN/../lib;$ORIGIN"
  INSTALL_RPATH "$ORIGIN"
)

ament_package()
