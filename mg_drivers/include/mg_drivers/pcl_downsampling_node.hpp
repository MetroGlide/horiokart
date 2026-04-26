#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// C++
#include <memory>
#include <string>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

namespace horiokart_drivers
{
    class PclDownsamplingNode : public rclcpp::Node
    {
    public:
        explicit PclDownsamplingNode(
            rclcpp::NodeOptions options);
        virtual ~PclDownsamplingNode(){};

    private:
        void init_ros_params();

        void prepare_ros_communications();
        void publish();

        void downsampling();

        std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pcl_cloud_;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pcl_cloud_downsampled_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

        rclcpp::TimerBase::SharedPtr publish_timer_;

        std::string frame_id_of_pcl_;
        int publish_rate_;
        double downsampling_leaf_size_;
    };
} // namespace horiokart_drivers