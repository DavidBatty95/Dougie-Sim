#include "unitree_social_costmap/replan_requested.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "behaviortree_cpp_v3/bt_factory.h"

namespace unitree_social_costmap
{
ReplanRequested::ReplanRequested(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  getInput("minimum_interval", minimum_interval_);
  getInput("fallback_interval", fallback_interval_);
  if (!(minimum_interval_ > 0.0 && fallback_interval_ >= minimum_interval_)) {
    throw std::invalid_argument("Invalid dynamic replanning intervals");
  }
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(group_, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions options;
  options.callback_group = group_;
  subscription_ = node_->create_subscription<std_msgs::msg::UInt64>(
    "/navigation/replan_request", rclcpp::QoS(1),
    [this](std_msgs::msg::UInt64::SharedPtr msg) {receive(msg->data);}, options);
  RCLCPP_INFO(node_->get_logger(), "Dynamic route replanning active: minimum %.2fs, fallback %.2fs",
    minimum_interval_, fallback_interval_);
}

BT::PortsList ReplanRequested::providedPorts()
{
  return {BT::InputPort<nav_msgs::msg::Path>("path"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<double>("minimum_interval", 0.2, "Minimum seconds between completed-route updates"),
    BT::InputPort<double>("fallback_interval", 0.75, "Periodic refresh if no event arrives")};
}

void ReplanRequested::receive(uint64_t sequence)
{
  latest_request_ = std::max(latest_request_, sequence);
}

BT::NodeStatus ReplanRequested::tick()
{
  executor_.spin_some(std::chrono::milliseconds(2));
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped goal;
  getInput("path", path);
  getInput("goal", goal);
  if (!have_goal_ || goal.header.frame_id != previous_goal_.header.frame_id || goal.pose != previous_goal_.pose) {
    goal_dirty_ = true;
    previous_goal_ = goal;
    have_goal_ = true;
  }
  const auto now = std::chrono::steady_clock::now();
  if (path != previous_path_ && !path.poses.empty()) {
    previous_path_ = path;
    if (in_flight_) {served_request_ = in_flight_request_;}
    in_flight_ = false;
    last_plan_ = now;
  }
  bool missing = path.poses.empty() || path.header.frame_id != goal.header.frame_id;
  if (!missing) {
    const auto & end = path.poses.back().pose.position;
    missing = std::hypot(end.x - goal.pose.position.x, end.y - goal.pose.position.y) > 0.30;
  }
  const double elapsed = std::chrono::duration<double>(now - last_plan_).count();
  if (in_flight_) {return BT::NodeStatus::SUCCESS;}
  if (goal_dirty_ || missing || (elapsed >= minimum_interval_ &&
    (latest_request_ > served_request_ || elapsed >= fallback_interval_)))
  {
    in_flight_ = true;
    in_flight_request_ = latest_request_;
    goal_dirty_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace unitree_social_costmap

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<unitree_social_costmap::ReplanRequested>("ReplanRequested");
}
