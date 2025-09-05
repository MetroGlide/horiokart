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
            // Declare parameters with default values
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

            // new topic/frame params
            n->declare_parameter("pointcloud_topic", params.pointcloud_topic);
            n->declare_parameter("marker_topic", params.marker_topic);
            n->declare_parameter("target_frame", params.target_frame);
            n->declare_parameter("pointcloud_queue_size", params.pointcloud_queue_size);
            n->declare_parameter("marker_queue_size", params.marker_queue_size);
            n->declare_parameter("tf_lookup_timeout_ms", params.tf_lookup_timeout_ms);
            n->declare_parameter("tf_retry_count", params.tf_retry_count);
            n->declare_parameter("tf_retry_backoff_ms", params.tf_retry_backoff_ms);

            // Get parameters and assign to struct
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

            // get new topic/frame params
            n->get_parameter("pointcloud_topic", params.pointcloud_topic);
            n->get_parameter("marker_topic", params.marker_topic);
            n->get_parameter("target_frame", params.target_frame);
            n->get_parameter("pointcloud_queue_size", params.pointcloud_queue_size);
            n->get_parameter("marker_queue_size", params.marker_queue_size);
            n->get_parameter("tf_lookup_timeout_ms", params.tf_lookup_timeout_ms);
            n->get_parameter("tf_retry_count", params.tf_retry_count);
            n->get_parameter("tf_retry_backoff_ms", params.tf_retry_backoff_ms);
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

// ファイル: parameter_manager.cpp
// 概要: ノード（通常の rclcpp::Node または LifecycleNode）からパラメータを宣言し取得するユーティリティ。
//       getParams() はすべてのパラメータを宣言し（存在しない場合デフォルトを設定）、その値を構造体に格納して返します。
