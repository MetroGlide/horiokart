// Lightweight point extractor moved out of obstacle_detector_node
#pragma once

#include <tf2/time.h>

#include <Eigen/Dense>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>
#include <vector>

#include "horiokart_obstacle_detector_3d/core/grid_height_map.hpp"
// Need ColorInfo from core
#include "horiokart_obstacle_detector_3d/core/cloud_processor.hpp"

namespace obstacle_detector
{
struct PointXYZ;
}

namespace obstacle_detector_node_helpers
{
using ColorInfo = obstacle_detector::ColorInfo;

// Extract points and per-point metadata from a transformed PointCloud2.
// The function mirrors the behavior of the previous member function.
void extractPointsAndMeta(
  const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_in_tf,
  std::vector<obstacle_detector::PointXYZ> & pts,
  std::unordered_map<std::string, obstacle_detector::PointXYZ> & dummy_meta,
  std::unordered_map<std::string, ColorInfo> & point_meta, bool & has_rgb_field,
  bool & has_intensity_field, double roi_x_min, double roi_x_max, double roi_y_min,
  double roi_y_max, double roi_z_min, double roi_z_max, bool use_rgb, bool use_intensity,
  double intensity_distance_ref, double intensity_distance_power,
  bool intensity_compensate_distance, bool intensity_compensate_angle,
  double intensity_angle_min_dot,
  const std::function<Eigen::Vector3d(const tf2::TimePoint &)> & getSensorForward,
  const std::function<geometry_msgs::msg::TransformStamped(const tf2::TimePoint &)> &
    getSensorTransform,
  const std::function<std::string(double, double, double)> & make_key,
  const std::function<std::vector<double>(const std::string &, const std::vector<double> &)> &
    get_param_double_vec);

// ColorInfo is defined in core and reused here
}  // namespace obstacle_detector_node_helpers
