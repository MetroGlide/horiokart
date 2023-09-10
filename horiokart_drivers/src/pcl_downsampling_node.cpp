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

        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud_;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud_downsampled_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

        rclcpp::TimerBase::SharedPtr publish_timer_;

        std::string frame_id_of_pcl_;
        std::string child_frame_id_of_pcl_;
        std::string device_name_;
        int publish_rate_;
        double downsampling_leaf_size_;
    };

    PclDownsamplingNode::PclDownsamplingNode(
        rclcpp::NodeOptions options) : Node("pcl_downsampling_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "pcl_downsampling_node has been created.");

        init_ros_params();

        prepare_ros_communications();
    }

    void PclDownsamplingNode::init_ros_params()
    {
        // Declare parameters
        this->declare_parameter<std::string>("pcl.frame_id");
        this->declare_parameter<std::string>("pcl.child_frame_id");
        this->declare_parameter<int>("pcl.publish_rate");
        this->declare_parameter<double>("pcl.downsampling_leaf_size");

        // Get parameters
        this->get_parameter_or<std::string>(
            "pcl.frame_id",
            frame_id_of_pcl_,
            std::string("base_link"));

        this->get_parameter_or<std::string>(
            "pcl.child_frame_id",
            child_frame_id_of_pcl_,
            std::string("base_link"));

        this->get_parameter_or<int>(
            "pcl.publish_rate",
            publish_rate_,
            20);

        this->get_parameter_or<double>(
            "pcl.downsampling_leaf_size",
            downsampling_leaf_size_,
            0.1);
    }

    void PclDownsamplingNode::prepare_ros_communications()
    {
        // Prepare ROS communications
        pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points/raw",
            rclcpp::QoS(1),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                pcl_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                pcl::fromROSMsg(*msg, *pcl_cloud_);
            });

        pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "points/downsampled",
            rclcpp::QoS(1));

        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / publish_rate_),
            std::bind(&PclDownsamplingNode::publish, this));
    }

    void PclDownsamplingNode::publish()
    {
        if (pcl_cloud_ == nullptr)
        {
            return;
        }

        downsampling();

        sensor_msgs::msg::PointCloud2 pcl_msg;
        pcl::toROSMsg(*pcl_cloud_downsampled_, pcl_msg);
        pcl_msg.header.frame_id = frame_id_of_pcl_;
        pcl_msg.header.stamp = this->now();
        pcl_pub_->publish(pcl_msg);
    }

    void PclDownsamplingNode::downsampling()
    {
        pcl_cloud_downsampled_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(pcl_cloud_);
        sor.setLeafSize(downsampling_leaf_size_, downsampling_leaf_size_, downsampling_leaf_size_);
        sor.filter(*pcl_cloud_downsampled_);
    }
} // namespace horiokart_drivers