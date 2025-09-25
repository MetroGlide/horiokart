#include <gtest/gtest.h>

#include "horiokart_obstacle_detector_3d/core/ground_separator.hpp"

using namespace obstacle_detector;

TEST(GroundSeparatorTest, Basic)
{
  GroundSeparator gs;
  gs.setParameters(0.2, 0.05);

  // low height, low variance -> ground
  EXPECT_TRUE(gs.isGround(0.1, 0.01));

  // high height -> not ground
  EXPECT_FALSE(gs.isGround(0.5, 0.01));

  // high variance -> not ground
  EXPECT_FALSE(gs.isGround(0.1, 0.1));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
