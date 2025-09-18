#pragma once

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <map>
#include <vector>
#include "horiokart_depth_camera_costmap/parameter_manager.hpp"
#include "horiokart_depth_camera_costmap/core_types.hpp"

namespace horiokart_depth_camera_costmap
{

    // alias to core namespace
    namespace core = ::horiokart::depth_camera_costmap;

    class DepthCameraCostmapLayer : public nav2_costmap_2d::Layer
    {
    public:
        DepthCameraCostmapLayer();
        virtual void onInitialize() override;
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double *min_x, double *min_y, double *max_x, double *max_y) override;
        virtual void updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                 int min_i, int min_j, int max_i, int max_j) override;
        virtual bool isClearable() override { return false; }
        void reset() override;

    private:
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        // Use core types for stored state
        horiokart::depth_camera_costmap::GridCostMap cost_map_;
        std::vector<horiokart::depth_camera_costmap::ObstacleCluster> clusters_;

        // store last robot pose received in updateBounds so updateCosts can convert local cell coords to world coords
        double last_robot_x_ = 0.0;
        double last_robot_y_ = 0.0;
        double last_robot_yaw_ = 0.0;
    };

} // namespace horiokart_depth_camera_costmap

// Note: PLUGINLIB_EXPORT_CLASS must be placed in a single .cpp implementation file, not in headers.
