#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <vector>

#include "horiokart_obstacle_detector_3d/core/grid_height_map.hpp"
#include "horiokart_obstacle_detector_3d/core/types.hpp"

namespace obstacle_detector
{
// Compute PCA-based slope (degrees) and normal per grid cell.
// Returns a map from linearized cell index (ix*cols + iy) to pair(slope_deg, normal)
std::unordered_map<int, std::pair<double, Eigen::Vector3d>> computePcaSlopesAndNormals(
  const std::vector<PointXYZ> & pts, const GridHeightMap & grid, double pca_radius_m,
  int pca_min_points, double grid_cell_size, double roi_x_min, double roi_y_min);

}  // namespace obstacle_detector
