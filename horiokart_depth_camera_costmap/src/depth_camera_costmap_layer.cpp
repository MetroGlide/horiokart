#include "horiokart_depth_camera_costmap/depth_camera_costmap_layer.hpp"
#include "horiokart_depth_camera_costmap/point_cloud_processor.hpp"
#include "horiokart_depth_camera_costmap/traversability_evaluator.hpp"
#include "horiokart_depth_camera_costmap/obstacle_clusterer.hpp"
#include "horiokart_depth_camera_costmap/parameter_manager.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cmath>

namespace horiokart_depth_camera_costmap
{

    DepthCameraCostmapLayer::DepthCameraCostmapLayer() {}

    void DepthCameraCostmapLayer::reset()
    {
        // Reset internal maps
        cost_map_.clear();
        clusters_.clear();
    }

    void DepthCameraCostmapLayer::onInitialize()
    {
        auto node = node_.lock();
        if (!node)
            return;
        pointcloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", 10,
            std::bind(&DepthCameraCostmapLayer::pointCloudCallback, this, std::placeholders::_1));
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("costmap_markers", 10);
    }

    void DepthCameraCostmapLayer::pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto node = node_.lock();
        if (!node)
            return;
        try
        {
            // Use rclcpp::Node* for ParameterManager
            ParameterManager param_mgr(node.get());
            auto params = param_mgr.getParams();
            PointCloudProcessor pc_proc;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*msg, *pcl_cloud);
            auto cloud_ds = pc_proc.downsample(pcl_cloud, static_cast<float>(params.voxel_leaf_size_m));
            auto cloud_filtered = pc_proc.removeOutliers(cloud_ds, params.sor_mean_k, params.sor_stddev_mul_thresh);
            geometry_msgs::msg::TransformStamped tf;
            try
            {
                tf = tf_buffer_->lookupTransform("base_link", msg->header.frame_id, msg->header.stamp);
            }
            catch (const std::exception &e)
            {
                auto node = node_.lock();
                if (node)
                {
                    RCLCPP_WARN(node->get_logger(), "TF取得失敗: %s", e.what());
                }
                return;
            }
            Eigen::Affine3f tf_eigen = tf2::transformToEigen(tf.transform).cast<float>();
            auto cloud_trans = pc_proc.transform(cloud_filtered, tf_eigen);
            auto grid_features = pc_proc.computeGridFeatures(cloud_trans, static_cast<float>(params.grid_resolution_m));
            TraversabilityEvaluator evaluator(static_cast<float>(params.max_normal_angle_deg), static_cast<float>(params.max_step_height_m), static_cast<float>(params.z_variance_threshold),
                                              static_cast<float>(params.normal_angle_threshold_deg), params.cost_traversable, params.cost_semi_traversable,
                                              params.cost_obstacle, params.cost_lethal);
            cost_map_ = evaluator.evaluate(grid_features);
            ObstacleClusterer clusterer(params.cluster_distance_threshold_m, params.cluster_min_points, params.grid_resolution_m);
            clusters_ = clusterer.cluster(cost_map_, params.cost_obstacle);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "点群処理例外: %s", e.what());
        }
    }

    void DepthCameraCostmapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                               double *min_x, double *min_y, double *max_x, double *max_y)
    {
        // store robot pose for use in updateCosts when converting local grid indices to world coords
        last_robot_x_ = robot_x;
        last_robot_y_ = robot_y;
        last_robot_yaw_ = robot_yaw;

        // conservative bounds around robot
        *min_x = robot_x - 5.0;
        *min_y = robot_y - 5.0;
        *max_x = robot_x + 5.0;
        *max_y = robot_y + 5.0;
    }

    void DepthCameraCostmapLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                              int min_i, int min_j, int max_i, int max_j)
    {
        auto node = node_.lock();
        if (!node)
            return;
        // fetch params once
        ParameterManager param_mgr(node.get());
        auto params = param_mgr.getParams();

        // Iterate through stored cost_map_ whose keys are grid cell indices in base_link frame
        for (const auto &kv : cost_map_)
        {
            int ix = kv.first.first;
            int iy = kv.first.second;
            int cost = kv.second;

            // convert cell index to local (base_link) coordinates (cell center)
            float cell_x = (static_cast<float>(ix) + 0.5f) * static_cast<float>(params.grid_resolution_m);
            float cell_y = (static_cast<float>(iy) + 0.5f) * static_cast<float>(params.grid_resolution_m);

            // rotate by robot yaw and translate by robot world pose to get world coordinates
            float cy = std::cos(static_cast<float>(last_robot_yaw_));
            float sy = std::sin(static_cast<float>(last_robot_yaw_));
            float world_x = static_cast<float>(last_robot_x_) + cy * cell_x - sy * cell_y;
            float world_y = static_cast<float>(last_robot_y_) + sy * cell_x + cy * cell_y;

            unsigned int mx, my;
            if (!master_grid.worldToMap(world_x, world_y, mx, my))
                continue; // outside master grid

            // check update window (map indices)
            if (static_cast<int>(mx) < min_i || static_cast<int>(mx) >= max_i || static_cast<int>(my) < min_j || static_cast<int>(my) >= max_j)
                continue;

            // clamp cost to valid range
            int clamped = std::min(255, std::max(0, cost));
            if (params.conditional_overwrite)
            {
                unsigned char existing = master_grid.getCost(mx, my);
                // overwrite only if depth camera suggests a lower cost (e.g., correct LIDAR false positives)
                if (clamped < static_cast<int>(existing))
                {
                    master_grid.setCost(mx, my, static_cast<unsigned char>(clamped));
                }
            }
            else
            {
                master_grid.setCost(mx, my, static_cast<unsigned char>(clamped));
            }
        }

        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        for (const auto &cluster : clusters_)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "base_link";
            m.header.stamp = node->now();
            m.ns = "depth_camera_clusters";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;
            // centroid is in cell coordinates; convert to meters using grid resolution from parameters
            float gx = cluster.centroid.x() * static_cast<float>(params.grid_resolution_m);
            float gy = cluster.centroid.y() * static_cast<float>(params.grid_resolution_m);
            m.pose.position.x = gx;
            m.pose.position.y = gy;
            m.pose.position.z = 0.5; // arbitrary height for visualization
            m.scale.x = params.grid_resolution_m * 1.0f * static_cast<float>(cluster.cells.size());
            m.scale.y = params.grid_resolution_m * 1.0f * static_cast<float>(cluster.cells.size());
            m.scale.z = 1.0;
            if (cluster.type == "wall")
            {
                m.color.r = 1.0;
                m.color.g = 0.0;
                m.color.b = 0.0;
                m.color.a = 0.8;
            }
            else if (cluster.type == "rock")
            {
                m.color.r = 0.5;
                m.color.g = 0.5;
                m.color.b = 0.2;
                m.color.a = 0.8;
            }
            else
            {
                m.color.r = 0.2;
                m.color.g = 0.2;
                m.color.b = 0.2;
                m.color.a = 0.6;
            }
            marker_array.markers.push_back(m);
        }
        marker_pub_->publish(marker_array);
    }

    PLUGINLIB_EXPORT_CLASS(horiokart_depth_camera_costmap::DepthCameraCostmapLayer, nav2_costmap_2d::Layer)

} // namespace horiokart_depth_camera_costmap
