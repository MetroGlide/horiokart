#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "horiokart_obstacle_detector_3d/obstacle_detector_node.hpp"

using std::chrono_literals::operator"ms";

TEST(PublishEmptyScan, PublishesWhenEnabled) {
  // Initialize rclcpp in test context
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  // Create node with parameter publish_empty_scan = true
  auto node = std::make_shared<ObstacleDetectorNode>();

  // set parameter explicitly
  node->set_parameter(rclcpp::Parameter("publish_empty_scan", true));

  // subscribe to scan topic
  std::atomic<bool> received{false};
  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "test_obstacle_scan", 10,
      [&](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        (void)msg;
        received = true;
      });

  // Instead of spinning the global executor, spin this node a few cycles
  rclcpp::WallRate rate(10);
  for (int i = 0; i < 5 && !received; ++i) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  // cleanup
  rclcpp::shutdown();
  EXPECT_TRUE(received);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
