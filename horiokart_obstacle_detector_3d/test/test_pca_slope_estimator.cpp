#include <gtest/gtest.h>

#include "horiokart_obstacle_detector_3d/core/grid_height_map.hpp"
#include "horiokart_obstacle_detector_3d/core/pca_slope_estimator.hpp"

using obstacle_detector::computePcaSlopesAndNormals;
using obstacle_detector::GridHeightMap;
using obstacle_detector::PointXYZ;

TEST(PcaSlopeEstimator, PlaneHorizontal) {
  // create points on z=0 plane within roi
  std::vector<PointXYZ> pts;
  for (double x = 0.5; x <= 1.5; x += 0.1) {
    for (double y = -0.5; y <= 0.5; y += 0.1) {
      pts.push_back(
          PointXYZ{static_cast<float>(x), static_cast<float>(y), 0.0f});
    }
  }
  GridHeightMap grid(0.0, 2.0, -1.0, 1.0, 0.1);
  // finalize empty grid to ensure rows/cols etc are consistent
  grid.setParameters(3, 1, 2.0, 0.6, 0.5, 0.3, 0.4, 0.5);
  grid.finalizeFrame(0.0);
  auto res = computePcaSlopesAndNormals(pts, grid, 0.2, 5, 0.1, 0.0, -1.0);
  // Expect many cells have near-zero slope
  int near_zero = 0;
  for (const auto &kv : res) {
    double slope = kv.second.first;
    if (slope < 5.0) {
      near_zero++;
    }
  }
  EXPECT_GT(near_zero, 0);
}
