#include <gtest/gtest.h>

#include "horiokart_obstacle_detector_3d/core/ground_separator.hpp"

using namespace obstacle_detector;

TEST(GroundSeparatorExtra, ScoreMonotonicity) {
  GroundSeparator gs;
  gs.setSlopeThresholdDeg(20.0);
  gs.setParameters(0.3, 0.05);
  gs.setScoreWeights(0.5, 0.4, 0.1);

  double s_low = gs.computeGroundScore(1.0, 0.001, 1.0);
  double s_mid = gs.computeGroundScore(5.0, 0.01, 0.8);
  double s_high = gs.computeGroundScore(15.0, 0.04, 0.5);

  EXPECT_GE(s_low, s_mid);
  EXPECT_GE(s_mid, s_high);
  EXPECT_GE(s_low, 0.0);
  EXPECT_LE(s_low, 1.0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
