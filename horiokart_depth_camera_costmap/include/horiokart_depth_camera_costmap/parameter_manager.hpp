#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace horiokart_depth_camera_costmap
{

    struct DepthCameraCostmapParams
    {
        float grid_resolution_m = 0.05f;
        float max_step_height_m = 0.1f;
        float max_normal_angle_deg = 30.0f;
        float normal_angle_threshold_deg = 10.0f;
        float variance_threshold = 0.02f;
        float cluster_distance_threshold_m = 0.2f;
        int cluster_min_points = 3;
        bool conditional_overwrite = true;

        // Additional params used by implementation
        float voxel_leaf_size_m = 0.02f;
        int sor_mean_k = 50;
        float sor_stddev_mul_thresh = 1.0f;
        float max_slope_angle_deg = 30.0f;
        float z_variance_threshold = 0.02f;

        int cost_traversable = 0;
        int cost_semi_traversable = 50;
        int cost_obstacle = 150;
        int cost_lethal = 255;

        // Topic / frame parameters
        std::string pointcloud_topic = "pointcloud";
        std::string marker_topic = "costmap_markers";
        std::string target_frame = "base_link";

        // Queue sizes and TF timeout
        int pointcloud_queue_size = 10;
        int marker_queue_size = 10;
        int tf_lookup_timeout_ms = 100;
        // TF retry policy
        int tf_retry_count = 3;       // number of attempts to try TF lookup
        int tf_retry_backoff_ms = 50; // base backoff in ms (exponential)

        // ... add other params as needed
    };

    class ParameterManager
    {
    public:
        ParameterManager(rclcpp::Node *node);
        ParameterManager(rclcpp_lifecycle::LifecycleNode *lifecycle_node);

        DepthCameraCostmapParams getParams();

    private:
        rclcpp::Node *node_ = nullptr;
        rclcpp_lifecycle::LifecycleNode *lifecycle_node_ = nullptr;
    };

} // namespace horiokart_depth_camera_costmap
