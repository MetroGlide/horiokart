#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class CostmapAdapterNode : public rclcpp::Node
{
public:
  CostmapAdapterNode() : Node("costmap_adapter_node")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/depth_costmap/occupancy_grid", 10,
        std::bind(&CostmapAdapterNode::onOccupancyGrid, this, std::placeholders::_1));
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/depth_costmap/for_nav2", 10);
    RCLCPP_INFO(this->get_logger(), "CostmapAdapterNode started");
  }

private:
  void onOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // For now, simply forward the occupancy grid to a topic Nav2 can subscribe to.
    // In future, perform diffing and conditional overwrite against master costmap.
    auto out = *msg;
    publisher_->publish(out);
    RCLCPP_DEBUG(this->get_logger(), "Forwarded occupancy grid (seq=%u)", msg->header.stamp.sec);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
