#include "horiokart_depth_camera_costmap/traversability_evaluator.hpp"
#include <cmath>

TraversabilityEvaluator::TraversabilityEvaluator(float max_slope_angle_deg, float max_step_height_m, float z_variance_threshold,
                                                 float normal_angle_threshold_deg, int cost_traversable, int cost_semi_traversable,
                                                 int cost_obstacle, int cost_lethal)
    : max_slope_angle_deg_(max_slope_angle_deg), max_step_height_m_(max_step_height_m),
      z_variance_threshold_(z_variance_threshold), normal_angle_threshold_deg_(normal_angle_threshold_deg),
      cost_traversable_(cost_traversable), cost_semi_traversable_(cost_semi_traversable),
      cost_obstacle_(cost_obstacle), cost_lethal_(cost_lethal) {}

std::map<std::pair<int, int>, int> TraversabilityEvaluator::evaluate(const std::map<std::pair<int, int>, GridCellFeature> &features)
{
    std::map<std::pair<int, int>, int> cost_map;
    for (const auto &kv : features)
    {
        const auto &feat = kv.second;
        // 安全のため正規化された法線を利用
        Eigen::Vector3f n = feat.mean_normal;
        if (n.norm() == 0)
            n = Eigen::Vector3f(0, 0, 1);
        else
            n.normalize();
        float nz = std::min(1.0f, std::max(-1.0f, n.z()));
        float normal_angle = std::acos(nz) * 180.0f / static_cast<float>(M_PI);
        // 路面判定
        if (feat.z_variance < z_variance_threshold_ && normal_angle < max_slope_angle_deg_)
        {
            cost_map[kv.first] = cost_traversable_;
            continue;
        }
        // 段差判定
        if (feat.z_max - feat.z_min > max_step_height_m_)
        {
            cost_map[kv.first] = cost_obstacle_;
            continue;
        }
        // 変曲点や急傾斜判定
        if (normal_angle < max_slope_angle_deg_ + normal_angle_threshold_deg_)
        {
            cost_map[kv.first] = cost_semi_traversable_;
            continue;
        }
        cost_map[kv.first] = cost_lethal_;
    }
    return cost_map;
}
