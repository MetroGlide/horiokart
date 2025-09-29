#include "horiokart_obstacle_detector_3d/node_helpers/grid_utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace obstacle_detector_node_helpers
{
std::unique_ptr<obstacle_detector::GridHeightMap> buildGridFromPoints(
  const std::vector<obstacle_detector::PointXYZ> & pts, double roi_x_min, double roi_x_max,
  double roi_y_min, double roi_y_max, double grid_cell_size,
  int min_obs_per_cell_for_confident_median, int radius_interp_cells, double interp_power_p,
  double interp_alpha, double max_interp_area_m2, double temporal_alpha_height,
  double temporal_alpha_conf, double observation_timeout)
{
  auto grid = std::make_unique<obstacle_detector::GridHeightMap>(
    roi_x_min, roi_x_max, roi_y_min, roi_y_max, grid_cell_size);
  grid->setParameters(
    min_obs_per_cell_for_confident_median, radius_interp_cells, interp_power_p, interp_alpha,
    max_interp_area_m2, temporal_alpha_height, temporal_alpha_conf, observation_timeout);
  for (const auto & p : pts) {
    grid->accumulatePoint(p);
  }
  grid->finalizeFrame(rclcpp::Clock().now().seconds());
  return grid;
}

}  // namespace obstacle_detector_node_helpers
