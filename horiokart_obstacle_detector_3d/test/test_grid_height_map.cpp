#include <gtest/gtest.h>

#include "horiokart_obstacle_detector_3d/core/grid_height_map.hpp"

using obstacle_detector::GridCell;
using obstacle_detector::GridHeightMap;
using obstacle_detector::PointXYZ;

TEST(GridHeightMapTest, MedianAndInterpolation)
{
  // grid covers x:[0,0.2), y:[0,0.2) with cell_size 0.1 => 2x2 grid
  GridHeightMap grid(0.0, 0.2, 0.0, 0.2, 0.1);
  grid.setParameters(3, 1, 2.0, 0.6, 0.5, 0.3, 0.4, 0.5);

  // Add points to cell (0,0)
  PointXYZ p1{0.02f, 0.02f, 0.0f};
  PointXYZ p2{0.03f, 0.01f, 0.01f};
  PointXYZ p3{0.04f, 0.03f, -0.01f};
  grid.accumulatePoint(p1);
  grid.accumulatePoint(p2);
  grid.accumulatePoint(p3);

  // cell (1,1) remains empty and should be interpolated from neighbors if small
  // hole
  double now = 1.0;
  grid.finalizeFrame(now);

  GridCell c00, c11;
  ASSERT_TRUE(grid.getCell(0, 0, c00));
  EXPECT_TRUE(c00.has_observation);
  EXPECT_NEAR(c00.height_median, 0.0, 1e-6);
  EXPECT_GE(c00.confidence, 1.0);

  // Check interpolated cell
  ASSERT_TRUE(grid.getCell(1, 1, c11));
  // since neighbors exist, interpolated cell should have has_observation true
  // and confidence > 0
  EXPECT_TRUE(c11.has_observation);
  EXPECT_GT(c11.confidence, 0.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
