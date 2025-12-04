#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace depth_postprocess
{

struct PostprocessParams
{
  double voxel_leaf_size = 0.05;
  bool use_statistical_outlier_removal = true;
  int sor_mean_k = 50;
  double sor_std_mul = 1.0;
};

class DepthPostprocessor
{
public:
  DepthPostprocessor();
  explicit DepthPostprocessor(const PostprocessParams & params);

  void setParams(const PostprocessParams & params);
  PostprocessParams getParams() const;

  // Process input pointcloud and write to output. Returns true on success.
  bool process(const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output);

private:
  PostprocessParams params_;
};

}  // namespace depth_postprocess
