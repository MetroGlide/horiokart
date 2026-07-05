#ifndef MG_NAVIGATION__IS_COLLISION_DETECTED_HPP_
#define MG_NAVIGATION__IS_COLLISION_DETECTED_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_msgs/msg/collision_detector_state.hpp"

namespace mg_navigation
{

class IsCollisionDetected : public BT::ConditionNode
{
public:
  IsCollisionDetected(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsCollisionDetected() = delete;

  ~IsCollisionDetected() override;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("polygon_index", 0, "Index of the polygon in CollisionDetectorState detections array")
    };
  }

private:
  void stateCallback(const nav2_msgs::msg::CollisionDetectorState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav2_msgs::msg::CollisionDetectorState>::SharedPtr sub_;
  nav2_msgs::msg::CollisionDetectorState::SharedPtr last_state_;
  int polygon_index_;
};

}  // namespace mg_navigation

#endif  // MG_NAVIGATION__IS_COLLISION_DETECTED_HPP_
