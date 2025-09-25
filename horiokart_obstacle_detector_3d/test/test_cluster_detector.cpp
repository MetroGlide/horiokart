#include <gtest/gtest.h>

#include "horiokart_obstacle_detector_3d/core/cluster_detector.hpp"

using namespace obstacle_detector;

// Helper to generate a cluster of points around center
static void make_cluster(
  std::vector<PointXYZ> & out, double cx, double cy, double cz, int n, double spread)
{
  for (int i = 0; i < n; ++i) {
    double rx = ((rand() / (double)RAND_MAX) - 0.5) * spread;
    double ry = ((rand() / (double)RAND_MAX) - 0.5) * spread;
    double rz = ((rand() / (double)RAND_MAX) - 0.5) * spread;
    out.push_back(PointXYZ{
      static_cast<float>(cx + rx), static_cast<float>(cy + ry), static_cast<float>(cz + rz)});
  }
}

TEST(ClusterDetectorTest, ThreeClusters)
{
  std::vector<PointXYZ> pts;
  // seed rand for reproducibility
  srand(12345);
  make_cluster(pts, 0.0, 0.0, 0.0, 50, 0.1);
  make_cluster(pts, 2.0, 0.0, 0.0, 40, 0.1);
  make_cluster(pts, 0.0, 2.0, 0.0, 30, 0.1);

  ClusterDetector det;
  det.setParameters(0.3, 10, 1000);  // tolerance 0.3
  auto clusters = det.extractClusters(pts);
  // expect 3 clusters
  EXPECT_EQ(clusters.size(), 3u);

  // check centroids are near expected locations (unordered)
  std::vector<PointXYZ> expected = {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}};
  for (auto & c : clusters) {
    bool matched = false;
    for (auto & e : expected) {
      double dx = c.centroid.x - e.x;
      double dy = c.centroid.y - e.y;
      double dz = c.centroid.z - e.z;
      double d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < 0.5 * 0.5) {
        matched = true;
        break;
      }
    }
    EXPECT_TRUE(matched);
  }
}
