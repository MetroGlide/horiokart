// ファイル: depth_camera_costmap_layer.cpp
// 概要: DepthCameraCostmapLayer は Nav2 のコストマップレイヤとして動作します。
//       深度カメラからの点群を受信し、前処理（ダウンサンプリング、外れ値除去、座標変換）を行い、
//       グリッド特徴量を算出してTraversabilityEvaluatorで通過可能性コストを評価します。
//       高コストセルは ObstacleClusterer によりクラスタ化され、可視化用のマーカーが配信されます。
//       updateCosts では master costmap に対して条件付き（または強制）上書きを行います。

#include "horiokart_depth_camera_costmap/depth_camera_costmap_layer.hpp"
#include "horiokart_depth_camera_costmap/parameter_manager.hpp"
#include "horiokart_depth_camera_costmap/core_processor.hpp"
#include "horiokart_depth_camera_costmap/core_types.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

namespace horiokart::depth_camera_costmap
{

    DepthCameraCostmapLayer::DepthCameraCostmapLayer() {}

    // reset: 内部に保持しているコストマップやクラスタ情報をクリアします。
    void DepthCameraCostmapLayer::reset()
    {
        // Reset internal maps
        // GridCostMap stores costs in .costs
        cost_map_ = GridCostMap();
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

            // convert PointCloud2 to core Point3D vector
            std::vector<Point3D> pts;
            pts.reserve(static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height));
            sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
            for (size_t i = 0; i < static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height); ++i, ++it_x, ++it_y, ++it_z)
            {
                float x = *it_x;
                float y = *it_y;
                float z = *it_z;
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                    continue;
                pts.push_back(Point3D{x, y, z});
            }

            // TF lookup and transform points to target_frame if requested
            geometry_msgs::msg::TransformStamped tfst;
            bool tf_ok = false;
            if (!params.target_frame.empty())
            {
                rclcpp::Duration timeout = rclcpp::Duration::from_seconds(static_cast<double>(params.tf_lookup_timeout_ms) / 1000.0);
                for (int attempt = 0; attempt < params.tf_retry_count && !tf_ok; ++attempt)
                {
                    try
                    {
                        if (tf_buffer_->canTransform(params.target_frame, msg->header.frame_id, msg->header.stamp, timeout))
                        {
                            tfst = tf_buffer_->lookupTransform(params.target_frame, msg->header.frame_id, msg->header.stamp);
                            tf_ok = true;
                            break;
                        }
                        if (tf_buffer_->canTransform(params.target_frame, msg->header.frame_id, rclcpp::Time(0), timeout))
                        {
                            tfst = tf_buffer_->lookupTransform(params.target_frame, msg->header.frame_id, rclcpp::Time(0));
                            tf_ok = true;
                            break;
                        }
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_DEBUG(node->get_logger(), "TF lookup attempt %d failed: %s", attempt + 1, e.what());
                    }
                    if (attempt + 1 < params.tf_retry_count)
                        rclcpp::sleep_for(std::chrono::milliseconds(params.tf_retry_backoff_ms * (1 << attempt)));
                }
            }
            // if target_frame empty or TF not available, we keep points in input frame
            if (tf_ok)
            {
                Eigen::Affine3d T = tf2::transformToEigen(tfst.transform);
                for (auto &p : pts)
                {
                    Eigen::Vector3d v(p.x, p.y, p.z);
                    Eigen::Vector3d vt = T * v;
                    p.x = static_cast<float>(vt.x());
                    p.y = static_cast<float>(vt.y());
                    p.z = static_cast<float>(vt.z());
                }
            }

            // map params to core CoreParams
            CoreParams core_params;
            core_params.grid_resolution = params.grid_resolution;
            core_params.voxel_size = params.voxel_size;
            core_params.sor_mean_k = params.sor_mean_k;
            core_params.sor_stddev_mul_thresh = params.sor_stddev_mul_thresh;
            core_params.normal_k = params.normal_k;
            core_params.normal_angle_threshold_deg = params.normal_angle_threshold_deg;
            core_params.z_variance_threshold = params.z_variance_threshold;
            core_params.cost_lethal = static_cast<std::uint8_t>(params.cost_lethal);

            try
            {
                // call core processing
                auto core_grid = processPointCloud(pts, core_params);
                cost_map_ = core_grid;

                // clustering
                ClusterParams cparams;
                cparams.cluster_tolerance = params.cluster_distance_threshold_m;
                cparams.min_cluster_size = static_cast<std::size_t>(params.cluster_min_points);
                cparams.merge_distance = 0.1f;
                clusters_ = clusterCostMap(cost_map_, cparams);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node->get_logger(), "core processing failed: %s", e.what());
            }
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
        // store robot pose for use in updateCosts when converting local cell coords to world coords
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
    void DepthCameraCostmapLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                              int min_i, int min_j, int max_i, int max_j)
    {
        auto node = node_.lock();
        if (!node)
            return;
        // fetch params once
        ParameterManager param_mgr(node.get());
        auto params = param_mgr.getParams();

        const double res = (cost_map_.resolution > 0.0f) ? cost_map_.resolution : params.grid_resolution;

        // iterate over dense grid cells
        for (uint32_t iy = 0; iy < cost_map_.height; ++iy)
        {
            for (uint32_t ix = 0; ix < cost_map_.width; ++ix)
            {
                const GridCellFeature &cell = cost_map_.cells[cost_map_.index(ix, iy)];
                if (cell.cost == 255)
                    continue; // unknown
                int cost = static_cast<int>(cell.cost);

                // local cell center in layer frame
                float cell_x = cost_map_.origin.x + (static_cast<float>(ix) + 0.5f) * static_cast<float>(res);
                float cell_y = cost_map_.origin.y + (static_cast<float>(iy) + 0.5f) * static_cast<float>(res);

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
        }

        // publish cluster markers
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

            // cluster.centroid is in layer-local coordinates; convert to world
            float cx = cluster.centroid.x;
            float cy0 = cluster.centroid.y;
            float cy_sin = std::sin(static_cast<float>(last_robot_yaw_));
            float cy_cos = std::cos(static_cast<float>(last_robot_yaw_));
            float world_gx = static_cast<float>(last_robot_x_) + cy_cos * cx - cy_sin * cy0;
            float world_gy = static_cast<float>(last_robot_y_) + cy_sin * cx + cy_cos * cy0;

            m.pose.position.x = world_gx;
            m.pose.position.y = world_gy;
            m.pose.position.z = cluster.centroid.z;
            m.scale.x = static_cast<float>(res) * 1.0f * static_cast<float>(cluster.cells.size());
            m.scale.y = static_cast<float>(res) * 1.0f * static_cast<float>(cluster.cells.size());
            m.scale.z = 1.0;

            // set color based on ObstacleType
            if (cluster.type == ObstacleType::WALL)
            {
                m.color.r = 1.0;
                m.color.g = 0.0;
                m.color.b = 0.0;
                m.color.a = 0.8;
            }
            else if (cluster.type == ObstacleType::POINT)
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

    PLUGINLIB_EXPORT_CLASS(horiokart::depth_camera_costmap::DepthCameraCostmapLayer, nav2_costmap_2d::Layer)

} // namespace horiokart::depth_camera_costmap
