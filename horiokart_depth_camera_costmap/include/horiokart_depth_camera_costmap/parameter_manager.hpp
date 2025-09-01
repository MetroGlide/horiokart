#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

struct DepthCameraCostmapParams
{
    double max_slope_angle_deg;
    double max_step_height_m;
    double grid_resolution_m;
    double z_variance_threshold;
    double normal_angle_threshold_deg;
    int cost_traversable;
    int cost_semi_traversable;
    int cost_obstacle;
    int cost_lethal;
    double voxel_leaf_size_m;
    int sor_mean_k;
    double sor_stddev_mul_thresh;
    double cluster_distance_threshold_m;
    int cluster_min_points;
};

class ParameterManager
{
public:
    ParameterManager(rclcpp_lifecycle::LifecycleNode *node);
    DepthCameraCostmapParams getParams();

private:
    rclcpp_lifecycle::LifecycleNode *node_;
};
