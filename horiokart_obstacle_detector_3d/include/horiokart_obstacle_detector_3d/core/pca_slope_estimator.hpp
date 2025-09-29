
#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <utility>
#include <vector>

#include "horiokart_obstacle_detector_3d/core/grid_height_map.hpp"
#include "horiokart_obstacle_detector_3d/core/types.hpp"

namespace obstacle_detector
{
// PCA ベースの傾斜（度）と法線をグリッドセルごとに計算します。
// 返り値は線形化されたセルインデックス (ix*cols + iy) をキーとする map です。
std::unordered_map<int, std::pair<double, Eigen::Vector3d>> computePcaSlopesAndNormals(
  const std::vector<PointXYZ> & pts, const GridHeightMap & grid, double pca_radius_m,
  int pca_min_points, double grid_cell_size, double roi_x_min, double roi_y_min);

}  // namespace obstacle_detector
