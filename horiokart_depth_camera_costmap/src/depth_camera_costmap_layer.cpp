// ファイル: depth_camera_costmap_layer.cpp
// 概要: DepthCameraCostmapLayer は Nav2 のコストマップレイヤとして動作します。
//       深度カメラからの点群を受信し、前処理（ダウンサンプリング、外れ値除去、座標変換）を行い、
//       グリッド特徴量を算出してTraversabilityEvaluatorで通過可能性コストを評価します。
//       高コストセルは ObstacleClusterer によりクラスタ化され、可視化用のマーカーが配信されます。
//       updateCosts では master costmap に対して条件付き（または強制）上書きを行います。

#include "horiokart_depth_camera_costmap/depth_camera_costmap_layer.hpp"
#include "horiokart_depth_camera_costmap/point_cloud_processor.hpp"
#include "horiokart_depth_camera_costmap/traversability_evaluator.hpp"
#include "horiokart_depth_camera_costmap/obstacle_clusterer.hpp"
#include "horiokart_depth_camera_costmap/parameter_manager.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cmath>

namespace horiokart_depth_camera_costmap
{

    DepthCameraCostmapLayer::DepthCameraCostmapLayer() {}

    // reset: 内部に保持しているコストマップやクラスタ情報をクリアします。
    void DepthCameraCostmapLayer::reset()
    {
        // Reset internal maps
        cost_map_.clear();
        clusters_.clear();
    }

    // onInitialize: ノードハンドルからパラメータを読み取り、点群購読やTFリスナ、マーカーパブリッシャを初期化します。
    void DepthCameraCostmapLayer::onInitialize()
    {
        auto node = node_.lock();
        if (!node)
            return;

        // read parameters to configure topics/frames
        ParameterManager param_mgr(node.get());
        auto params = param_mgr.getParams();

        pointcloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            params.pointcloud_topic, params.pointcloud_queue_size,
            std::bind(&DepthCameraCostmapLayer::pointCloudCallback, this, std::placeholders::_1));
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(params.marker_topic, params.marker_queue_size);
    }

    // pointCloudCallback: 点群を受け取った際のメイン処理パイプライン。
    // 1) パラメータ取得
    // 2) 点群のダウンサンプリング、外れ値除去
    // 3) TF を取得して点群を target_frame に変換（リトライ・バックオフあり）
    // 4) グリッド特徴量を算出
    // 5) TraversabilityEvaluator によりコストマップを生成
    // 6) ObstacleClusterer により障害物クラスタを作成
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
            bool tf_ok = false;
            rclcpp::Duration timeout = rclcpp::Duration::from_seconds(static_cast<double>(params.tf_lookup_timeout_ms) / 1000.0);
            for (int attempt = 0; attempt < params.tf_retry_count && !tf_ok; ++attempt)
            {
                try
                {
                    if (tf_buffer_->canTransform(params.target_frame, msg->header.frame_id, msg->header.stamp, timeout))
                    {
                        tf = tf_buffer_->lookupTransform(params.target_frame, msg->header.frame_id, msg->header.stamp);
                        tf_ok = true;
                        break;
                    }
                    if (tf_buffer_->canTransform(params.target_frame, msg->header.frame_id, rclcpp::Time(0), timeout))
                    {
                        tf = tf_buffer_->lookupTransform(params.target_frame, msg->header.frame_id, rclcpp::Time(0));
                        tf_ok = true;
                        RCLCPP_WARN(node->get_logger(), "TF lookup with message time not available; fell back to latest transform (attempt %d).", attempt + 1);
                        break;
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_DEBUG(node->get_logger(), "TF lookup attempt %d failed: %s", attempt + 1, e.what());
                }

                // exponential backoff
                if (attempt + 1 < params.tf_retry_count)
                {
                    int backoff = params.tf_retry_backoff_ms * (1 << attempt);
                    rclcpp::sleep_for(std::chrono::milliseconds(backoff));
                }
            }
            if (!tf_ok)
            {
                RCLCPP_WARN(node->get_logger(), "TF not available for transform from %s to %s after %d attempts", msg->header.frame_id.c_str(), params.target_frame.c_str(), params.tf_retry_count);
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

    // updateBounds: コストマップ更新領域を robot の周辺に保守的に設定します。
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

    // updateCosts: master_grid に対して内部で保持している cost_map_ を反映します。
    // セル座標 -> ロボットローカル -> ワールド座標 に変換し、マスターグリッドの対応セルを更新します。
    // conditional_overwrite が有効な場合は既存の値より低いコストのみ上書きします。
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
            m.header.frame_id = params.target_frame;
            m.header.stamp = node->now();
            m.ns = "depth_camera_clusters";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;
            // centroid is in cell coordinates; convert to meters using grid resolution from parameters
            float gx = cluster.centroid.x() * static_cast<float>(params.grid_resolution_m);
            float gy = cluster.centroid.y() * static_cast<float>(params.grid_resolution_m);
            // convert local base_link coords to world using last_robot pose
            float cy = std::cos(static_cast<float>(last_robot_yaw_));
            float sy = std::sin(static_cast<float>(last_robot_yaw_));
            float world_gx = static_cast<float>(last_robot_x_) + cy * gx - sy * gy;
            float world_gy = static_cast<float>(last_robot_y_) + sy * gx + cy * gy;
            m.pose.position.x = world_gx;
            m.pose.position.y = world_gy;
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
