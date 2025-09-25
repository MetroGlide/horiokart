#pragma once

#include <vector>

#include "horiokart_obstacle_detector_3d/core/types.hpp"

namespace obstacle_detector
{
// Simple headless ScanProjector: projects a set of PointXYZ into angle buckets
// and returns range array (range == inf when no observation).
class ScanProjector
{
public:
  ScanProjector(double angle_min, double angle_max, double angle_increment, double range_max);

  // Project points (in base frame) and return ranges (size = N_buckets)
  std::vector<float> project(const std::vector<PointXYZ> & points) const;

private:
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double range_max_;
  int buckets_;
};

}  // namespace obstacle_detector
