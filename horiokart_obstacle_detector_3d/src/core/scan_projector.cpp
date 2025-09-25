#include "horiokart_obstacle_detector_3d/core/scan_projector.hpp"

#include <cmath>
#include <limits>

namespace obstacle_detector
{
ScanProjector::ScanProjector(
  double angle_min, double angle_max, double angle_increment, double range_max)
: angle_min_(angle_min),
  angle_max_(angle_max),
  angle_increment_(angle_increment),
  range_max_(range_max)
{
  buckets_ = std::max(1, static_cast<int>(std::ceil((angle_max_ - angle_min_) / angle_increment_)));
}

std::vector<float> ScanProjector::project(const std::vector<PointXYZ> & points) const
{
  std::vector<float> ranges(buckets_, std::numeric_limits<float>::infinity());
  for (const auto & p : points) {
    double r = std::hypot(p.x, p.y);
    if (r <= 0.0 || r > range_max_) {
      continue;
    }
    double ang = std::atan2(p.y, p.x);
    if (ang < angle_min_ || ang > angle_max_) {
      continue;
    }
    int k = static_cast<int>(std::floor((ang - angle_min_) / angle_increment_));
    if (k < 0 || k >= buckets_) {
      continue;
    }
    if (r < ranges[k]) {
      ranges[k] = static_cast<float>(r);
    }
  }
  return ranges;
}

}  // namespace obstacle_detector
