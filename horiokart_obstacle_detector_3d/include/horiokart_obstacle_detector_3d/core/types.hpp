#pragma once

namespace obstacle_detector
{
struct PointXYZ
{
  float x;
  float y;
  float z;
};

struct GridIndex
{
  int ix;
  int iy;
};

struct GridCell
{
  bool has_observation = false;
  double height_median = 0.0;
  double height_mean = 0.0;
  double height_variance = 0.0;
  int obs_count = 0;
  double confidence = 0.0;
  double last_observed_time = 0.0;  // seconds (monotonic)
  bool is_ground = false;
  double ground_ema = 0.0;  // exponential moving average of ground score (0..1)
};

}  // namespace obstacle_detector
