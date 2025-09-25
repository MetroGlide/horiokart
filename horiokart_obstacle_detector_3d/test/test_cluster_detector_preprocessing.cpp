#include <gtest/gtest.h>

#include "horiokart_obstacle_detector_3d/core/cluster_detector.hpp"

using namespace obstacle_detector;

TEST(ClusterDetectorPreprocessing, RadiusOutlierRemoval)
{
  // create a cluster at origin and a few isolated noise points far away
  std::vector<PointXYZ> pts;
  for (int i = 0; i < 50; ++i) {
    pts.push_back(PointXYZ{
      static_cast<float>(0.0 + 0.01 * (rand() / (double)RAND_MAX)),
      static_cast<float>(0.0 + 0.01 * (rand() / (double)RAND_MAX)),
      static_cast<float>(0.0 + 0.01 * (rand() / (double)RAND_MAX))});
  }
  // add noise
  pts.push_back(PointXYZ{5.0f, 5.0f, 5.0f});
  pts.push_back(PointXYZ{6.0f, 5.0f, 5.0f});

  ClusterDetector det;
  det.setParameters(0.2, 10, 1000);
  det.setOutlierRadius(0.05);
  det.setOutlierMinNeighbors(2);
  det.setDownsampleLeafSize(0.0);

  auto clusters = det.extractClusters(pts);
  // Expect 1 cluster (noise removed)
  EXPECT_EQ(clusters.size(), 1u);
}
