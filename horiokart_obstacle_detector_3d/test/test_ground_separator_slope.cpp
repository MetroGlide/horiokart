#include <gtest/gtest.h>

#include "horiokart_obstacle_detector_3d/core/ground_separator.hpp"

using namespace obstacle_detector;

TEST(GroundSeparatorSlope, SlopeBehavior)
{
  GroundSeparator g;
  g.setParameters(0.5, 0.1);
  g.setSlopeThresholdDeg(10.0);

  // low slope, low variance -> ground
  EXPECT_TRUE(g.isGround(0.2, 0.05, 5.0));
  // high slope -> not ground
  EXPECT_FALSE(g.isGround(0.2, 0.05, 12.0));
  // high variance -> not ground
  EXPECT_FALSE(g.isGround(0.2, 0.2, 5.0));
}
