#include "depth_postprocess/depth_postprocessor.hpp"

#include <pcl/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <exception>
#include <memory>

namespace depth_postprocess
{

DepthPostprocessor::DepthPostprocessor() : params_() {}

DepthPostprocessor::DepthPostprocessor(const PostprocessParams & params) : params_(params) {}

void DepthPostprocessor::setParams(const PostprocessParams & params) { params_ = params; }
PostprocessParams DepthPostprocessor::getParams() const { return params_; }

bool DepthPostprocessor::process(
  const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output)
{
  using PointT = pcl::PointXYZRGB;

  pcl::PointCloud<PointT> cloud;
  try {
    pcl::fromROSMsg(input, cloud);
  } catch (const std::exception & e) {
    std::cerr << "Exception during point cloud conversion: " << e.what() << std::endl;
    return false;
  }

  auto cloud_ptr = std::make_shared<pcl::PointCloud<PointT>>(cloud);
  auto filtered = std::make_shared<pcl::PointCloud<PointT>>();

  // Statistical outlier removal (optional)
  if (params_.use_statistical_outlier_removal) {
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_ptr);
    sor.setMeanK(params_.sor_mean_k);
    sor.setStddevMulThresh(params_.sor_std_mul);
    sor.filter(*filtered);
  } else {
    *filtered = *cloud_ptr;
  }

  // VoxelGrid downsample
  auto downsampled = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(filtered);
  vg.setLeafSize(
    static_cast<float>(params_.voxel_leaf_size), static_cast<float>(params_.voxel_leaf_size),
    static_cast<float>(params_.voxel_leaf_size));
  vg.filter(*downsampled);

  pcl::PCLPointCloud2 pcl2;
  pcl::toPCLPointCloud2(*downsampled, pcl2);
  pcl_conversions::fromPCL(pcl2, output);
  output.header = input.header;

  return true;
}

}  // namespace depth_postprocess
