#include <gtest/gtest.h>

#include "horiokart_obstacle_detector_3d/core/grid_height_map.hpp"

using namespace obstacle_detector;

// Test EMA temporal fusion and observation timeout behavior
TEST(GridHeightMapMore, EMAAndTimeout)
{
  GridHeightMap g(0.0, 1.0, 0.0, 1.0, 0.5);  // 2x2 grid with cell_size 0.5
  g.setParameters(2, 1, 2.0, 0.5, 1.0, 0.5, 0.5, 0.2);

  // Frame 1: observe cell (0,0) with height 1.0 (two points)
  g.reset();
  g.accumulatePoint(PointXYZ{0.1f, 0.1f, 1.0f});
  g.accumulatePoint(PointXYZ{0.2f, 0.2f, 1.0f});
  g.finalizeFrame(0.0);

  GridCell c;
  ASSERT_TRUE(g.getCell(0, 0, c));
  EXPECT_NEAR(c.height_median, 1.0, 1e-6);
  double conf1 = c.confidence;

  // Frame 2: new observation lower (0.0). EMA should blend
  g.reset();
  g.accumulatePoint(PointXYZ{0.1f, 0.1f, 0.0f});
  g.accumulatePoint(PointXYZ{0.2f, 0.2f, 0.0f});
  g.finalizeFrame(0.1);
  ASSERT_TRUE(g.getCell(0, 0, c));
  // height should be between 1.0 and 0.0 due to temporal_alpha_height=0.5
  EXPECT_GT(c.height_median, 0.0);
  EXPECT_LT(c.height_median, 1.0);
  double conf2 = c.confidence;
  EXPECT_GT(conf2, 0.0);

  // Wait beyond observation_timeout (0.2) and finalize with no observations
  g.finalizeFrame(1.0);
  ASSERT_TRUE(g.getCell(0, 0, c));
  // should have expired observations
  EXPECT_FALSE(c.has_observation);
  EXPECT_EQ(c.confidence, 0.0);
}
