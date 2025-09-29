#include <gtest/gtest.h>

#include "horiokart_obstacle_detector_3d/core/grid_height_map.hpp"

using obstacle_detector::GridCell;
using obstacle_detector::GridHeightMap;
using obstacle_detector::PointXYZ;

// Verify small hole is interpolated while a large hole is not
TEST(GridHeightMapHole, SmallVsLargeHole)
{
  // Build a 5x1 grid: x [0..0.5) cell_size=0.1, y single row
  GridHeightMap g(0.0, 0.5, 0.0, 0.1, 0.1);
  // parameters: neighborhood, interpolation params: keep defaults
  // set max_interp_area_m2 small so a 2-cell hole is considered "large" and not
  // interpolated one cell area = 0.1 * 0.1 = 0.01 m2; set max_interp_area_m2
  // between 0.01 and 0.02
  g.setParameters(3, 3, 2.0, 0.6, 0.015, 0.3, 0.4, 0.5);

  // Fill cells 0 and 2 (leaving cell 1 as small hole), and leave cells 3,4 as a
  // large hole
  g.accumulatePoint(PointXYZ{0.05f, 0.05f, 0.0f});  // cell 0
  g.accumulatePoint(PointXYZ{0.25f, 0.05f, 0.0f});  // cell 2
  // no points in cell 1 -> should be interpolated (small hole)

  // cells 3 and 4 remain empty -> large connected hole spanning two cells

  g.finalizeFrame(0.1);

  GridCell c0, c1, c4;
  ASSERT_TRUE(g.getCell(0, 0, c0));
  ASSERT_TRUE(g.getCell(1, 0, c1));
  ASSERT_TRUE(g.getCell(4, 0, c4));

  // cell 0 observed
  EXPECT_TRUE(c0.has_observation);
  // cell 1 small hole should be interpolated
  EXPECT_TRUE(c1.has_observation);
  EXPECT_GT(c1.confidence, 0.0);
  // cell 4 (far end) large hole should remain unobserved (no interpolation)
  EXPECT_FALSE(c4.has_observation);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
