cmake_minimum_required(VERSION 3.8)
project(horiokart_depth_camera_costmap)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(depth_camera_costmap_layer SHARED
  src/depth_camera_costmap_layer.cpp
  src/point_cloud_processor.cpp
  src/traversability_evaluator.cpp
  src/obstacle_clusterer.cpp
  src/parameter_manager.cpp
)

target_include_directories(depth_camera_costmap_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(depth_camera_costmap_layer
  rclcpp sensor_msgs nav_msgs visualization_msgs tf2_ros pluginlib PCL nav2_costmap_2d pcl_conversions tf2_eigen
)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugin.xml)

install(TARGETS depth_camera_costmap_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
