#pragma once

#include <vector>

#include "horiokart_obstacle_detector_3d/core/types.hpp"

namespace obstacle_detector
{
struct Cluster
{
  std::vector<PointXYZ> points;
  PointXYZ centroid;
  double volume;
  int id;
};

class ClusterDetector
{
public:
  ClusterDetector() = default;

  void setParameters(double cluster_tolerance, int min_cluster_size, int max_cluster_size);

  /// Optional: enable voxel grid downsampling by specifying a leaf size (meters).
  /// Set to 0.0 to disable (default).
  void setDownsampleLeafSize(double leaf_size);
  // Radius outlier removal (optional). radius=0 disables.
  void setOutlierRadius(double radius);
  void setOutlierMinNeighbors(int min_neighbors);
  // PassThrough filter on Z (optional). Disabled if enable=false
  void setPassThroughZ(bool enable, double z_min = -1.0, double z_max = 1.0);
  // If true, after clustering on downsampled cloud, expand clusters by mapping back to original
  // cloud
  void setExpandToOriginalCloud(bool enable);

  // Minimal API: extract clusters from a vector of points (headless)
  std::vector<Cluster> extractClusters(const std::vector<PointXYZ> & points) const;

private:
  double cluster_tolerance_ = 0.1;
  int min_cluster_size_ = 30;
  int max_cluster_size_ = 1000000;
  double voxel_leaf_size_ = 0.0;
  // outlier removal
  double outlier_radius_ = 0.0;
  int outlier_min_neighbors_ = 1;
  // passthrough z filter
  bool passthrough_z_enable_ = false;
  double passthrough_z_min_ = -1.0;
  double passthrough_z_max_ = 1.0;
  // mapping back
  bool expand_to_original_cloud_ = false;
};

}  // namespace obstacle_detector
