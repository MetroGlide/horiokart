#include "horiokart_drivers/pcl_downsampling_node.hpp"

using horiokart_drivers::PclDownsamplingNode;

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
    this->declare_parameter<int>("pcl.publish_rate");
    this->declare_parameter<double>("pcl.downsampling_leaf_size");

    // Get parameters
    this->get_parameter_or<int>(
        "pcl.publish_rate",
        publish_rate_,
        5);

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
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            pcl_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
            pcl::fromROSMsg(*msg, *pcl_cloud_);
            frame_id_of_pcl_ = msg->header.frame_id;
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
    pcl_cloud_downsampled_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(pcl_cloud_);
    sor.setLeafSize(downsampling_leaf_size_, downsampling_leaf_size_, downsampling_leaf_size_);
    sor.filter(*pcl_cloud_downsampled_);
}