#pragma once

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>
#include <vector>

#include "horiokart_obstacle_detector_3d/core/grid_height_map.hpp"
// need ColorInfo
#include "horiokart_obstacle_detector_3d/core/cloud_processor.hpp"

namespace obstacle_detector
{
struct PointXYZ;
struct Cluster;
}  // namespace obstacle_detector

namespace obstacle_detector_node_helpers
{
void publishConfidenceCloud(
  const obstacle_detector::GridHeightMap & grid,
  const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_in_tf,
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_confidence_cloud,
  double roi_x_min, double roi_y_min, double grid_cell_size);

void publishObstacleCloudAndScan(
  const std::vector<obstacle_detector::PointXYZ> & obstacle_pts,
  const std::unordered_map<std::string, obstacle_detector::ColorInfo> & point_meta,
  const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_in_tf,
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_cloud,
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan, bool use_rgb,
  bool use_intensity, const std::function<std::string(double, double, double)> & make_key,
  const std::function<std::vector<double>(const std::string &, const std::vector<double> &)> &
    get_param_double_vec);

}  // namespace obstacle_detector_node_helpers
