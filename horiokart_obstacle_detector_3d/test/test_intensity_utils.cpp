#include <gtest/gtest.h>

#include "horiokart_obstacle_detector_3d/core/intensity_utils.hpp"

using obstacle_detector::compensateIntensity;
using obstacle_detector::PointXYZ;

TEST(IntensityUtils, DistanceOnly)
{
  PointXYZ p{1.0f, 0.0f, 0.0f};
  Eigen::Vector3d origin(0.0, 0.0, 0.0);
  Eigen::Vector3d fwd(1.0, 0.0, 0.0);
  double in = 1.0;
  double out = compensateIntensity(in, p, origin, fwd, 1.0, 2.0, true, false, 0.2);
  // at distance 1 and ref 1, factor==1 => out ~ in
  EXPECT_NEAR(out, 1.0, 1e-6);
  PointXYZ p2{2.0f, 0.0f, 0.0f};
  double out2 = compensateIntensity(in, p2, origin, fwd, 1.0, 2.0, true, false, 0.2);
  // at distance 2, factor=4 => intensity should be scaled down
  EXPECT_LT(out2, out);
}

TEST(IntensityUtils, AngleOnly)
{
  PointXYZ p{1.0f, 0.0f, 0.0f};
  Eigen::Vector3d origin(0.0, 0.0, 0.0);
  Eigen::Vector3d fwd(0.0, 1.0, 0.0);  // perpendicular
  double in = 1.0;
  double out = compensateIntensity(in, p, origin, fwd, 1.0, 2.0, false, true, 0.5);
  // dot = 0 -> scale to 0
  EXPECT_NEAR(out, 0.0, 1e-6);
}
