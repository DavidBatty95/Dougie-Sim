#include <gtest/gtest.h>
#include <thread>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "unitree_social_costmap/replan_requested.hpp"

TEST(ReplanRequested, LatchesInFlightAndPreservesNewerRequests)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("replan_condition_test");
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("node", node);
  config.input_ports = {{"path", "{path}"}, {"goal", "{goal}"},
    {"minimum_interval", "0.001"}, {"fallback_interval", "60.0"}};
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 2.0;
  config.blackboard->set("path", path);
  config.blackboard->set("goal", goal);
  unitree_social_costmap::ReplanRequested condition("request", config);
  EXPECT_EQ(condition.tick(), BT::NodeStatus::SUCCESS);
  condition.receive(10);
  EXPECT_EQ(condition.tick(), BT::NodeStatus::SUCCESS);
  path.header.frame_id = "map";
  path.header.stamp.sec = 1;
  path.poses.push_back(goal);
  config.blackboard->set("path", path);
  EXPECT_EQ(condition.tick(), BT::NodeStatus::FAILURE);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  EXPECT_EQ(condition.tick(), BT::NodeStatus::SUCCESS);  // Request 10 was not lost.
  condition.receive(11);
  EXPECT_EQ(condition.tick(), BT::NodeStatus::SUCCESS);  // Never halt the running action.
  path.header.stamp.sec = 2;
  config.blackboard->set("path", path);
  EXPECT_EQ(condition.tick(), BT::NodeStatus::FAILURE);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  EXPECT_EQ(condition.tick(), BT::NodeStatus::SUCCESS);  // Request 11 needs its own update.
  path.header.stamp.sec = 3;
  config.blackboard->set("path", path);
  EXPECT_EQ(condition.tick(), BT::NodeStatus::FAILURE);
  condition.receive(10);  // Reordered/duplicate requests cannot retrigger.
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  EXPECT_EQ(condition.tick(), BT::NodeStatus::FAILURE);
  goal.pose.position.x = 2.1;  // Even changes smaller than endpoint tolerance replan.
  config.blackboard->set("goal", goal);
  EXPECT_EQ(condition.tick(), BT::NodeStatus::SUCCESS);
  rclcpp::shutdown();
}

TEST(ReplanRequested, RecoveryDelayUsesMillisecondsRatherThanIntegerSecondWait)
{
  BT::BehaviorTreeFactory factory;
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<Delay delay_msec='20'><AlwaysSuccess/></Delay></BehaviorTree></root>");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}

TEST(ReplanRequested, PeriodicFallbackWorksWithoutAnEventPublisher)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("replan_fallback_test");
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("node", node);
  config.input_ports = {{"path", "{path}"}, {"goal", "{goal}"},
    {"minimum_interval", "0.001"}, {"fallback_interval", "0.02"}};
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 2.;
  config.blackboard->set("path", path);
  config.blackboard->set("goal", goal);
  unitree_social_costmap::ReplanRequested condition("fallback", config);
  EXPECT_EQ(condition.tick(), BT::NodeStatus::SUCCESS);
  path.header.frame_id = "map";
  path.header.stamp.sec = 1;
  path.poses.push_back(goal);
  config.blackboard->set("path", path);
  EXPECT_EQ(condition.tick(), BT::NodeStatus::FAILURE);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(condition.tick(), BT::NodeStatus::SUCCESS);
  rclcpp::shutdown();
}
