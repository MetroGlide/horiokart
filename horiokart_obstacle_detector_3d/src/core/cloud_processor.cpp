// SPDX-License-Identifier: Apache-2.0

#include "horiokart_obstacle_detector_3d/core/cloud_processor.hpp"

#include <algorithm>
#include <limits>

#include "horiokart_obstacle_detector_3d/core/pca_slope_estimator.hpp"

namespace obstacle_detector
{
CloudProcessor::CloudProcessor(const CloudProcessorConfig & cfg) : cfg_(cfg) {}

CloudProcessorResult CloudProcessor::process(
  const std::vector<PointXYZ> & pts, const std::unordered_map<std::string, ColorInfo> & point_meta,
  ClusterDetector & cluster_detector, GroundSeparator & ground_separator) const
{
  CloudProcessorResult res;

  // 1) build grid
  res.grid = std::make_unique<GridHeightMap>(
    cfg_.roi_x_min, cfg_.roi_x_max, cfg_.roi_y_min, cfg_.roi_y_max, cfg_.grid_cell_size);
  // parameters for grid are still left to caller to set if desired
  for (const auto & p : pts) {
    res.grid->accumulatePoint(p);
  }
  // finalize frame with dummy timestamp 0 (caller can set temporal params earlier)
  res.grid->finalizeFrame(0.0);

  // 2) PCA slopes if requested
  if (cfg_.slope_method == "pca") {
    res.pca_results = obstacle_detector::computePcaSlopesAndNormals(
      pts, *res.grid, cfg_.pca_radius_m, cfg_.pca_min_points, cfg_.grid_cell_size, cfg_.roi_x_min,
      cfg_.roi_y_min);
  }

  // 3) classify points into ground / non-ground using ground_separator
  for (const auto & p : pts) {
    int ix = static_cast<int>(std::floor((p.x - cfg_.roi_x_min) / cfg_.grid_cell_size));
    int iy = static_cast<int>(std::floor((p.y - cfg_.roi_y_min) / cfg_.grid_cell_size));
    GridCell cell;
    double slope_deg = 0.0;
    if (cfg_.slope_method == "pca") {
      int idx = ix * res.grid->cols() + iy;
      auto itp = res.pca_results.find(idx);
      if (itp != res.pca_results.end()) {
        slope_deg = itp->second.first;
      } else {
        res.grid->getCellSlopeDeg(ix, iy, slope_deg);
      }
    } else {
      res.grid->getCellSlopeDeg(ix, iy, slope_deg);
    }
    if (res.grid->getCell(ix, iy, cell) && cell.has_observation) {
      auto itmeta = point_meta.find(
        std::to_string(p.x) + "," + std::to_string(p.y) + "," + std::to_string(p.z));
      if (itmeta != point_meta.end() && itmeta->second.has_intensity) {
        double in = itmeta->second.intensity;
        cell.confidence = std::min(1.0, cell.confidence * (1.0 - 0.2) + in * 0.2);
      }
      double dz = std::abs(p.z - cell.height_median);
      double score =
        ground_separator.computeGroundScore(slope_deg, cell.height_variance, cell.confidence);
      double alpha = ground_separator.getEmaAlpha();
      cell.ground_ema = alpha * score + (1.0 - alpha) * cell.ground_ema;
      double high_th = ground_separator.getHighThreshold();
      double low_th = ground_separator.getLowThreshold();
      bool is_ground_by_score =
        (cell.ground_ema >= high_th) || (cell.ground_ema > low_th && cell.is_ground);
      if (dz <= cfg_.ground_max_distance && is_ground_by_score) {
        res.ground_points.push_back(p);
        continue;
      }
    }
    res.non_ground_points.push_back(p);
  }

  // 4) clustering & obstacle selection
  // adjust cluster detector downsample based on pts count
  if (cfg_.dynamic_leaf && pts.size() > 0) {
    double scale =
      std::sqrt(static_cast<double>(pts.size()) / static_cast<double>(cfg_.target_points));
    double new_leaf = cfg_.voxel_leaf_size * scale;
    new_leaf = std::max(0.005, std::min(0.2, new_leaf));
    cluster_detector.setDownsampleLeafSize(new_leaf);
  }
  res.clusters = cluster_detector.extractClusters(res.non_ground_points);
  for (const auto & c : res.clusters) {
    double zmin = std::numeric_limits<double>::max(), zmax = -std::numeric_limits<double>::max();
    for (const auto & p : c.points) {
      zmin = std::min(zmin, (double)p.z);
      zmax = std::max(zmax, (double)p.z);
    }
    double height = zmax - zmin;
    if (height < cfg_.min_obstacle_height) continue;
    if (c.volume < cfg_.min_obstacle_volume) continue;
    res.obstacle_points.insert(res.obstacle_points.end(), c.points.begin(), c.points.end());
  }

  return res;
}

}  // namespace obstacle_detector
