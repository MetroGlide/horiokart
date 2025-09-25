#pragma once

#include <Eigen/Dense>

#include "horiokart_obstacle_detector_3d/core/types.hpp"

namespace obstacle_detector
{
// Pure function: apply distance and angle compensation to a normalized intensity in [0,1].
// Parameters:
// - intensity: input normalized intensity [0,1]
// - point: point coordinates (in target frame)
// - sensor_origin: sensor origin in target frame
// - sensor_forward: unit forward vector of sensor in target frame
// - distance_ref: reference distance for inverse scaling
// - distance_power: exponent for distance compensation
// - compensate_distance: enable/disable distance compensation
// - compensate_angle: enable/disable angle compensation
// - angle_min_dot: minimum dot used to scale angle compensation
// Returns compensated intensity in [0,1]
double compensateIntensity(
  double intensity, const PointXYZ & point, const Eigen::Vector3d & sensor_origin,
  const Eigen::Vector3d & sensor_forward, double distance_ref, double distance_power,
  bool compensate_distance, bool compensate_angle, double angle_min_dot);

}  // namespace obstacle_detector
