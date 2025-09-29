#pragma once

#include <memory>
#include <vector>

#include "horiokart_obstacle_detector_3d/core/grid_height_map.hpp"

namespace obstacle_detector_node_helpers
{
std::unique_ptr<obstacle_detector::GridHeightMap> buildGridFromPoints(
  const std::vector<obstacle_detector::PointXYZ> & pts, double roi_x_min, double roi_x_max,
  double roi_y_min, double roi_y_max, double grid_cell_size,
  int min_obs_per_cell_for_confident_median, int radius_interp_cells, double interp_power_p,
  double interp_alpha, double max_interp_area_m2, double temporal_alpha_height,
  double temporal_alpha_conf, double observation_timeout);

}  // namespace obstacle_detector_node_helpers
