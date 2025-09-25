#include <gtest/gtest.h>

#include <cmath>

#include "horiokart_obstacle_detector_3d/core/cluster_detector.hpp"

using namespace obstacle_detector;

TEST(ClusterDetectorExpand, ExpandToOriginal)
{
  // create dense original cloud cluster
  std::vector<PointXYZ> original;
  const double PI = std::acos(-1.0);
  for (int i = 0; i < 200; ++i) {
    double ang = (i / 200.0) * 2.0 * PI;
    original.push_back(PointXYZ{
      static_cast<float>(0.5 * std::cos(ang)), static_cast<float>(0.5 * std::sin(ang)), 0.0f});
  }
  // detector will internally downsample when leaf size is set; pass the full original cloud

  ClusterDetector det;
  det.setParameters(0.2, 5, 10000);
  det.setDownsampleLeafSize(0.05);
  det.setExpandToOriginalCloud(true);

  // run on downsampled input but expect expanded clusters to contain many original points
  auto clusters = det.extractClusters(original);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_GT(clusters[0].points.size(), 50u);  // expanded should recover many points
}
