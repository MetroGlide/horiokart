#include <memory>
#include <string>

#include "depth_postprocess/depth_postprocessor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

class DepthPostprocessNode : public rclcpp::Node
{
public:
  DepthPostprocessNode() : Node("depth_postprocess_node")
  {
    // Voxel grid leaf size (meters): edge length of each voxel cube in meters.
    this->declare_parameter<double>("voxel_leaf_size", 0.05);

    // Enable statistical outlier removal filter (boolean).
    this->declare_parameter<bool>("use_statistical_outlier_removal", true);

    // Mean K for StatisticalOutlierRemoval: number of nearest neighbors to use (integer).
    this->declare_parameter<int>("sor_mean_k", 50);

    // Standard deviation multiplier threshold for outlier removal (unitless).
    this->declare_parameter<double>("sor_std_mul", 1.0);

    // Use local (relative) topic names so they can be remapped from launch files.
    // Remap example in launch: ("points", "/camera/camera/depth/color/points")
    const std::string in_topic = "points";
    const std::string out_topic = "points_filtered";

    depth_postprocess::PostprocessParams p;
    p.voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
    p.use_statistical_outlier_removal =
      this->get_parameter("use_statistical_outlier_removal").as_bool();
    p.sor_mean_k = this->get_parameter("sor_mean_k").as_int();
    p.sor_std_mul = this->get_parameter("sor_std_mul").as_double();

    processor_ = std::make_unique<depth_postprocess::DepthPostprocessor>(p);

    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, qos);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic, qos, std::bind(&DepthPostprocessNode::pointcloudCallback, this, _1));
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 out;
    if (processor_->process(*msg, out)) {
      pub_->publish(out);
    } else {
      RCLCPP_WARN(this->get_logger(), "DepthPostprocessor failed to process incoming cloud");
    }
  }

  std::unique_ptr<depth_postprocess::DepthPostprocessor> processor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthPostprocessNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
