#include "horiokart_depth_camera_costmap/parameter_manager.hpp"

namespace horiokart_depth_camera_costmap
{

    ParameterManager::ParameterManager(rclcpp::Node *node) : node_(node), lifecycle_node_(nullptr) {}

    ParameterManager::ParameterManager(rclcpp_lifecycle::LifecycleNode *lifecycle_node)
        : node_(nullptr), lifecycle_node_(lifecycle_node)
    {
    }

    DepthCameraCostmapParams ParameterManager::getParams()
    {
        DepthCameraCostmapParams params;

        if (!node_ && !lifecycle_node_)
            return params;

        auto declare_and_get = [&](auto *n)
        {
            n->declare_parameter("grid_resolution_m", params.grid_resolution_m);
            n->declare_parameter("max_step_height_m", params.max_step_height_m);
            n->declare_parameter("max_normal_angle_deg", params.max_normal_angle_deg);
            n->declare_parameter("normal_angle_threshold_deg", params.normal_angle_threshold_deg);
            n->declare_parameter("variance_threshold", params.variance_threshold);
            n->declare_parameter("cluster_distance_threshold_m", params.cluster_distance_threshold_m);
            n->declare_parameter("cluster_min_points", params.cluster_min_points);
            n->declare_parameter("conditional_overwrite", params.conditional_overwrite);

            n->declare_parameter("voxel_leaf_size_m", params.voxel_leaf_size_m);
            n->declare_parameter("sor_mean_k", params.sor_mean_k);
            n->declare_parameter("sor_stddev_mul_thresh", params.sor_stddev_mul_thresh);
            n->declare_parameter("max_slope_angle_deg", params.max_slope_angle_deg);
            n->declare_parameter("z_variance_threshold", params.z_variance_threshold);

            n->declare_parameter("cost_traversable", params.cost_traversable);
            n->declare_parameter("cost_semi_traversable", params.cost_semi_traversable);
            n->declare_parameter("cost_obstacle", params.cost_obstacle);
            n->declare_parameter("cost_lethal", params.cost_lethal);

            n->get_parameter("grid_resolution_m", params.grid_resolution_m);
            n->get_parameter("max_step_height_m", params.max_step_height_m);
            n->get_parameter("max_normal_angle_deg", params.max_normal_angle_deg);
            n->get_parameter("normal_angle_threshold_deg", params.normal_angle_threshold_deg);
            n->get_parameter("variance_threshold", params.variance_threshold);
            n->get_parameter("cluster_distance_threshold_m", params.cluster_distance_threshold_m);
            n->get_parameter("cluster_min_points", params.cluster_min_points);
            n->get_parameter("conditional_overwrite", params.conditional_overwrite);

            n->get_parameter("voxel_leaf_size_m", params.voxel_leaf_size_m);
            n->get_parameter("sor_mean_k", params.sor_mean_k);
            n->get_parameter("sor_stddev_mul_thresh", params.sor_stddev_mul_thresh);
            n->get_parameter("max_slope_angle_deg", params.max_slope_angle_deg);
            n->get_parameter("z_variance_threshold", params.z_variance_threshold);

            n->get_parameter("cost_traversable", params.cost_traversable);
            n->get_parameter("cost_semi_traversable", params.cost_semi_traversable);
            n->get_parameter("cost_obstacle", params.cost_obstacle);
            n->get_parameter("cost_lethal", params.cost_lethal);
        };

        if (node_)
        {
            declare_and_get(node_);
        }
        else if (lifecycle_node_)
        {
            declare_and_get(lifecycle_node_);
        }

        return params;
    }

} // namespace horiokart_depth_camera_costmap
