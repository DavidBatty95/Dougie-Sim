#pragma once

#include <chrono>
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace unitree_social_costmap
{
// A request stays latched until ComputePathToPose replaces the blackboard path.
// This keeps ReactiveFallback from halting an in-flight planning action.
class ReplanRequested : public BT::ConditionNode
{
public:
  ReplanRequested(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void receive(uint64_t sequence);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr subscription_;
  nav_msgs::msg::Path previous_path_;
  geometry_msgs::msg::PoseStamped previous_goal_;
  bool have_goal_{false}, goal_dirty_{false};
  uint64_t latest_request_{0}, served_request_{0}, in_flight_request_{0};
  bool in_flight_{false};
  std::chrono::steady_clock::time_point last_plan_{std::chrono::steady_clock::now()};
  double minimum_interval_{0.2}, fallback_interval_{0.75};
};
}  // namespace unitree_social_costmap
