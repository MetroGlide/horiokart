#include "horiokart_depth_camera_costmap/parameter_manager.hpp"

ParameterManager::ParameterManager(rclcpp_lifecycle::LifecycleNode *node) : node_(node) {}

DepthCameraCostmapParams ParameterManager::getParams()
{
    DepthCameraCostmapParams params;
    params.max_slope_angle_deg = node_->declare_parameter<double>("max_slope_angle_deg", 20.0);
    params.max_step_height_m = node_->declare_parameter<double>("max_step_height_m", 0.10);
    params.grid_resolution_m = node_->declare_parameter<double>("grid_resolution_m", 0.10);
    params.z_variance_threshold = node_->declare_parameter<double>("z_variance_threshold", 0.005);
    params.normal_angle_threshold_deg = node_->declare_parameter<double>("normal_angle_threshold_deg", 10.0);
    params.cost_traversable = node_->declare_parameter<int>("cost_traversable", 10);
    params.cost_semi_traversable = node_->declare_parameter<int>("cost_semi_traversable", 80);
    params.cost_obstacle = node_->declare_parameter<int>("cost_obstacle", 200);
    params.cost_lethal = node_->declare_parameter<int>("cost_lethal", 255);
    params.voxel_leaf_size_m = node_->declare_parameter<double>("voxel_leaf_size_m", 0.05);
    params.sor_mean_k = node_->declare_parameter<int>("sor_mean_k", 50);
    params.sor_stddev_mul_thresh = node_->declare_parameter<double>("sor_stddev_mul_thresh", 1.0);
    params.cluster_distance_threshold_m = node_->declare_parameter<double>("cluster_distance_threshold_m", 0.20);
    params.cluster_min_points = node_->declare_parameter<int>("cluster_min_points", 10);
    return params;
}
