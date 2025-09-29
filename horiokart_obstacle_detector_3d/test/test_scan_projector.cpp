#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "horiokart_obstacle_detector_3d/core/scan_projector.hpp"
#include "horiokart_obstacle_detector_3d/core/types.hpp"

using obstacle_detector::PointXYZ;
using obstacle_detector::ScanProjector;

TEST(ScanProjector, BasicProjection)
{
  ScanProjector sp(-1.57, 1.57, 0.1, 10.0);
  std::vector<PointXYZ> pts;
  // point straight ahead (0 rad)
  pts.push_back(PointXYZ{1.0f, 0.0f, 0.0f});
  // left 45deg
  pts.push_back(PointXYZ{0.7f, 0.7f, 0.0f});
  // out of range
  pts.push_back(PointXYZ{20.0f, 0.0f, 0.0f});

  auto ranges = sp.project(pts);
  // find bucket for 0 rad
  int idx0 = static_cast<int>(std::floor((0.0 - (-1.57)) / 0.1));
  ASSERT_LT(idx0, static_cast<int>(ranges.size()));
  EXPECT_NEAR(ranges[idx0], 1.0f, 1e-6);
  // left 45deg ~ 0.785 rad
  int idx45 = static_cast<int>(std::floor((0.785398 - (-1.57)) / 0.1));
  ASSERT_LT(idx45, static_cast<int>(ranges.size()));
  EXPECT_NEAR(ranges[idx45], std::hypot(0.7f, 0.7f), 1e-6);
}
