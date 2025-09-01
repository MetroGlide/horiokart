cmake_minimum_required(VERSION 3.8)
project(test_depth_camera_costmap_layer)

find_package(ament_cmake_gtest REQUIRED)
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
  ../include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

ament_add_gtest(test_depth_camera_costmap_layer
  test_depth_camera_costmap_layer.cpp
)
target_link_libraries(test_depth_camera_costmap_layer
  depth_camera_costmap_layer
)
ament_target_dependencies(test_depth_camera_costmap_layer
  rclcpp sensor_msgs nav_msgs visualization_msgs tf2_ros pluginlib PCL Eigen3 nav2_costmap_2d pcl_conversions tf2_eigen
)

ament_package()
