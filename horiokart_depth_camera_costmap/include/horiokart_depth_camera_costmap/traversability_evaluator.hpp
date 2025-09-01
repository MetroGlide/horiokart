#pragma once
#include <Eigen/Dense>
#include <map>
#include <utility>
#include "horiokart_depth_camera_costmap/point_cloud_processor.hpp"

class TraversabilityEvaluator
{
public:
    TraversabilityEvaluator(float max_slope_angle_deg, float max_step_height_m, float z_variance_threshold,
                            float normal_angle_threshold_deg, int cost_traversable, int cost_semi_traversable,
                            int cost_obstacle, int cost_lethal);
    std::map<std::pair<int, int>, int> evaluate(const std::map<std::pair<int, int>, GridCellFeature> &features);

private:
    float max_slope_angle_deg_;
    float max_step_height_m_;
    float z_variance_threshold_;
    float normal_angle_threshold_deg_;
    int cost_traversable_;
    int cost_semi_traversable_;
    int cost_obstacle_;
    int cost_lethal_;
};
