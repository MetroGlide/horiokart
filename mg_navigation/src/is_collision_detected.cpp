#include "mg_navigation/is_collision_detected.hpp"
#include "nav2_util/node_utils.hpp"

namespace mg_navigation
{

IsCollisionDetected::IsCollisionDetected(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  polygon_index_(0)
{
  node_ = conf.blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("polygon_index", polygon_index_);

  sub_ = node_->create_subscription<nav2_msgs::msg::CollisionDetectorState>(
    "/collision_detector_state", rclcpp::SystemDefaultsQoS(),
    std::bind(&IsCollisionDetected::stateCallback, this, std::placeholders::_1));
}

IsCollisionDetected::~IsCollisionDetected()
{
}

void IsCollisionDetected::stateCallback(const nav2_msgs::msg::CollisionDetectorState::SharedPtr msg)
{
  last_state_ = msg;
}

BT::NodeStatus IsCollisionDetected::tick()
{
  if (!last_state_) {
    return BT::NodeStatus::FAILURE;
  }

  if (polygon_index_ < 0 || static_cast<size_t>(polygon_index_) >= last_state_->detections.size()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(), 1000,
      "IsCollisionDetected: polygon_index %d is out of bounds (size %zu)",
      polygon_index_, last_state_->detections.size());
    return BT::NodeStatus::FAILURE;
  }

  if (last_state_->detections[polygon_index_]) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace mg_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mg_navigation::IsCollisionDetected>("IsCollisionDetected");
}
