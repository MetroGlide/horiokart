#include "obstacle_detection/obstacle_detector.hpp"

#include <cmath>
#include <limits>
#include <iostream>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

namespace obstacle_detection
{

ObstacleDetector::ObstacleDetector() : params_()
{
  cloud_voxeled_ = std::make_shared<PCLCloud>();
  cloud_cropped_ = std::make_shared<PCLCloud>();
  cloud_obstacle_ = std::make_shared<PCLCloud>();
  cloud_clustered_ = std::make_shared<PCLCloud>();
}

ObstacleDetector::ObstacleDetector(const ObstacleDetectionParams & params) : params_(params)
{
  cloud_voxeled_ = std::make_shared<PCLCloud>();
  cloud_cropped_ = std::make_shared<PCLCloud>();
  cloud_obstacle_ = std::make_shared<PCLCloud>();
  cloud_clustered_ = std::make_shared<PCLCloud>();
}

void ObstacleDetector::setParams(const ObstacleDetectionParams & params) { params_ = params; }
ObstacleDetectionParams ObstacleDetector::getParams() const { return params_; }

bool ObstacleDetector::process(const PCLCloud & input, PCLCloud & output_obstacle,
                               std::vector<pcl::PointIndices> & output_clusters)
{
  if (input.empty()) {
    return false;
  }

  // PCLオブジェクトの使い回し (clear only)
  cloud_voxeled_->clear();
  cloud_cropped_->clear();
  cloud_obstacle_->clear();
  cloud_clustered_->clear();

  downsample(input, *cloud_voxeled_);
  cropROI(*cloud_voxeled_, *cloud_cropped_);
  extractObstaclesByDeltaZ(*cloud_cropped_, *cloud_obstacle_);
  removeOutliers(*cloud_obstacle_, *cloud_clustered_);
  extractClusters(*cloud_clustered_, output_obstacle, output_clusters);

  return true;
}

void ObstacleDetector::downsample(const PCLCloud & in, PCLCloud & out)
{
  if (in.empty()) return;
  pcl::VoxelGrid<PointT> vg;
  auto in_ptr = std::make_shared<PCLCloud>(in);
  vg.setInputCloud(in_ptr);
  vg.setLeafSize(params_.voxel_leaf_size, params_.voxel_leaf_size, params_.voxel_leaf_size);
  vg.filter(out);
}

void ObstacleDetector::cropROI(const PCLCloud & in, PCLCloud & out)
{
  if (in.empty()) return;
  pcl::CropBox<PointT> crop;
  auto in_ptr = std::make_shared<PCLCloud>(in);
  crop.setInputCloud(in_ptr);
  crop.setMin(Eigen::Vector4f(params_.cropbox_x_min, params_.cropbox_y_min, params_.cropbox_z_min, 1.0));
  crop.setMax(Eigen::Vector4f(params_.cropbox_x_max, params_.cropbox_y_max, params_.cropbox_z_max, 1.0));
  crop.filter(out);
}

void ObstacleDetector::extractObstaclesByDeltaZ(const PCLCloud & in, PCLCloud & out)
{
  if (in.empty()) return;

  int grid_width = static_cast<int>(std::ceil((params_.cropbox_x_max - params_.cropbox_x_min) / params_.grid_size));
  int grid_height = static_cast<int>(std::ceil((params_.cropbox_y_max - params_.cropbox_y_min) / params_.grid_size));

  if (grid_width <= 0 || grid_height <= 0) return;

  // メンバ配列を .assign() でリセット（毎フレーム new しない）
  grid_.assign(grid_width * grid_height, GridCell{});

  for (const auto & p : in.points) {
    if (p.x < params_.cropbox_x_min || p.x >= params_.cropbox_x_max ||
        p.y < params_.cropbox_y_min || p.y >= params_.cropbox_y_max) {
      continue;
    }

    int ix = static_cast<int>((p.x - params_.cropbox_x_min) / params_.grid_size);
    int iy = static_cast<int>((p.y - params_.cropbox_y_min) / params_.grid_size);

    if (ix < 0 || ix >= grid_width || iy < 0 || iy >= grid_height) {
      continue;
    }

    int idx = ix + iy * grid_width;
    grid_[idx].has_point = true;
    grid_[idx].z_min = std::min(grid_[idx].z_min, p.z);
    grid_[idx].z_max = std::max(grid_[idx].z_max, p.z);
  }

  out.header = in.header;
  for (const auto & p : in.points) {
    if (p.x < params_.cropbox_x_min || p.x >= params_.cropbox_x_max ||
        p.y < params_.cropbox_y_min || p.y >= params_.cropbox_y_max) {
      continue;
    }

    int ix = static_cast<int>((p.x - params_.cropbox_x_min) / params_.grid_size);
    int iy = static_cast<int>((p.y - params_.cropbox_y_min) / params_.grid_size);

    if (ix < 0 || ix >= grid_width || iy < 0 || iy >= grid_height) {
      continue;
    }

    int idx = ix + iy * grid_width;
    if (grid_[idx].has_point && (grid_[idx].z_max - grid_[idx].z_min) > params_.delta_z_threshold) {
      out.points.push_back(p);
    }
  }

  out.width = out.points.size();
  out.height = 1;
  out.is_dense = true;
}

void ObstacleDetector::removeOutliers(const PCLCloud & in, PCLCloud & out)
{
  if (in.empty()) return;
  pcl::RadiusOutlierRemoval<PointT> ror;
  auto in_ptr = std::make_shared<PCLCloud>(in);
  ror.setInputCloud(in_ptr);
  ror.setRadiusSearch(params_.ror_radius_search);
  ror.setMinNeighborsInRadius(params_.ror_min_neighbors);
  ror.filter(out);
}

void ObstacleDetector::extractClusters(const PCLCloud & in,
                                       PCLCloud & out,
                                       std::vector<pcl::PointIndices> & clusters)
{
  if (in.empty()) return;
  
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  auto in_ptr = std::make_shared<PCLCloud>(in);
  tree->setInputCloud(in_ptr);

  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(params_.cluster_tolerance);
  ec.setMinClusterSize(params_.min_cluster_size);
  ec.setMaxClusterSize(params_.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(in_ptr);
  ec.extract(clusters);

  out.header = in.header;
  for (const auto & cluster : clusters) {
    for (const auto & idx : cluster.indices) {
      out.points.push_back(in.points[idx]);
    }
  }
  out.width = out.points.size();
  out.height = 1;
  out.is_dense = true;
}

}  // namespace obstacle_detection
