#include <gtest/gtest.h>
#include <algorithm>
#include "unitree_social_costmap/people_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

class TestPeopleLayer : public unitree_social_costmap::PeopleLayer
{
public:
  using PeopleLayer::receive;
};

class StaticPointLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override {enabled_ = current_ = true;}
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(double, double, double, double * x0, double * y0, double * x1, double * y1) override
  {
    *x0 = std::min(*x0, 0.9); *y0 = std::min(*y0, -0.1);
    *x1 = std::max(*x1, 1.1); *y1 = std::max(*y1, 0.1);
  }
  void updateCosts(nav2_costmap_2d::Costmap2D & grid, int, int, int, int) override
  {
    unsigned int x, y;
    if (grid.worldToMap(1.0, 0.0, x, y)) {grid.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);}
  }
};

TEST(PeopleLayer, MovementEmptyMessagesAndExpiryClearPreviousFootprints)
{
  rclcpp::init(0, nullptr);
  auto options = rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("people.enabled", true)});
  auto node = std::make_shared<nav2_util::LifecycleNode>("people_layer_test", "", options);
  tf2_ros::Buffer tf(node->get_clock());
  nav2_costmap_2d::LayeredCostmap map("world", false, false);
  map.resizeMap(100, 100, 0.1, -5.0, -5.0);
  auto layer = std::make_shared<TestPeopleLayer>();
  map.addPlugin(layer);
  layer->initialize(&map, "people", &tf, node, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  auto message = std::make_shared<unitree_social_navigation_msgs::msg::TrackedPersons>();
  message->header.frame_id = "world";
  message->header.stamp = node->now();
  unitree_social_navigation_msgs::msg::TrackedPerson person;
  person.pose.pose.position.x = 1.0;
  person.body_radius = 0.4;
  person.confidence = 1.0;
  person.track_state = person.TRACK_STATE_CONFIRMED;
  message->persons.push_back(person);
  layer->receive(message);
  map.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(map.getCostmap()->getCost(60, 50), nav2_costmap_2d::LETHAL_OBSTACLE);
  auto moved = std::make_shared<unitree_social_navigation_msgs::msg::TrackedPersons>(*message);
  moved->persons[0].pose.pose.position.x = 2.0;
  layer->receive(moved);
  map.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(map.getCostmap()->getCost(60, 50), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(map.getCostmap()->getCost(70, 50), nav2_costmap_2d::LETHAL_OBSTACLE);
  auto empty = std::make_shared<unitree_social_navigation_msgs::msg::TrackedPersons>(*moved);
  empty->persons.clear();
  layer->receive(empty);
  map.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(map.getCostmap()->getCost(70, 50), nav2_costmap_2d::FREE_SPACE);
  EXPECT_TRUE(layer->isCurrent());
  layer->receive(moved);
  map.updateMap(0.0, 0.0, 0.0);
  auto stale = std::make_shared<unitree_social_navigation_msgs::msg::TrackedPersons>(*moved);
  stale->header.stamp = node->now() - rclcpp::Duration::from_seconds(2.0);
  layer->receive(stale);
  map.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(map.getCostmap()->getCost(70, 50), nav2_costmap_2d::FREE_SPACE);
  EXPECT_FALSE(layer->isCurrent());
  rclcpp::shutdown();
}

TEST(PeopleLayer, ClearingPersonPreservesUnderlyingStaticObstacle)
{
  rclcpp::init(0, nullptr);
  auto options = rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("people.enabled", true)});
  auto node = std::make_shared<nav2_util::LifecycleNode>("people_static_test", "", options);
  tf2_ros::Buffer tf(node->get_clock());
  nav2_costmap_2d::LayeredCostmap map("world", false, false);
  map.resizeMap(100, 100, 0.1, -5.0, -5.0);
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto fixed = std::make_shared<StaticPointLayer>();
  map.addPlugin(fixed);
  fixed->initialize(&map, "fixed", &tf, node, group);
  auto layer = std::make_shared<TestPeopleLayer>();
  map.addPlugin(layer);
  layer->initialize(&map, "people", &tf, node, group);
  auto message = std::make_shared<unitree_social_navigation_msgs::msg::TrackedPersons>();
  message->header.frame_id = "world";
  message->header.stamp = node->now();
  unitree_social_navigation_msgs::msg::TrackedPerson person;
  person.pose.pose.position.x = 1.0;
  person.body_radius = 0.4;
  person.confidence = 1.0;
  person.track_state = person.TRACK_STATE_CONFIRMED;
  message->persons.push_back(person);
  layer->receive(message);
  map.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(map.getCostmap()->getCost(63, 50), nav2_costmap_2d::LETHAL_OBSTACLE);
  layer->reset();
  map.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(map.getCostmap()->getCost(60, 50), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(map.getCostmap()->getCost(63, 50), nav2_costmap_2d::FREE_SPACE);
  rclcpp::shutdown();
}

TEST(PeopleLayer, ForecastIsSoftAndClearsOnDirectionChangeWithoutErasingStaticMap)
{
  rclcpp::init(0, nullptr);
  auto options = rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("people.enabled", true), rclcpp::Parameter("people.prediction_horizon", 1.2)});
  auto node = std::make_shared<nav2_util::LifecycleNode>("people_prediction_test", "", options);
  tf2_ros::Buffer tf(node->get_clock());
  nav2_costmap_2d::LayeredCostmap map("odom", false, false);
  map.resizeMap(100, 100, 0.1, -5.0, -5.0);
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto fixed = std::make_shared<StaticPointLayer>();
  map.addPlugin(fixed);
  fixed->initialize(&map, "fixed", &tf, node, group);
  auto layer = std::make_shared<TestPeopleLayer>();
  map.addPlugin(layer);
  layer->initialize(&map, "people", &tf, node, group);
  auto message = std::make_shared<unitree_social_navigation_msgs::msg::TrackedPersons>();
  message->header.frame_id = "odom";
  message->header.stamp = node->now();
  unitree_social_navigation_msgs::msg::TrackedPerson person;
  person.body_radius = 0.1;
  person.confidence = 1.0;
  person.track_state = person.TRACK_STATE_CONFIRMED;
  person.velocity.twist.linear.x = 1.0;
  message->persons.push_back(person);
  layer->receive(message);
  map.updateMap(0., 0., 0.);
  EXPECT_EQ(map.getCostmap()->getCost(50, 50), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(map.getCostmap()->getCost(58, 50), nav2_costmap_2d::FREE_SPACE);
  EXPECT_LT(map.getCostmap()->getCost(58, 50), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(map.getCostmap()->getCost(60, 50), nav2_costmap_2d::LETHAL_OBSTACLE);
  auto turned = std::make_shared<unitree_social_navigation_msgs::msg::TrackedPersons>(*message);
  turned->persons[0].velocity.twist.linear.x = 0.0;
  turned->persons[0].velocity.twist.linear.y = 1.0;
  layer->receive(turned);
  map.updateMap(0., 0., 0.);
  EXPECT_EQ(map.getCostmap()->getCost(58, 50), nav2_costmap_2d::FREE_SPACE);
  EXPECT_GT(map.getCostmap()->getCost(50, 58), nav2_costmap_2d::FREE_SPACE);
  layer->reset();
  map.updateMap(0., 0., 0.);
  EXPECT_EQ(map.getCostmap()->getCost(50, 58), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(map.getCostmap()->getCost(60, 50), nav2_costmap_2d::LETHAL_OBSTACLE);
  rclcpp::shutdown();
}

TEST(PeopleLayer, ForecastDoesNotDeclareUnknownSpaceFree)
{
  rclcpp::init(0, nullptr);
  auto options = rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("people.enabled", true), rclcpp::Parameter("people.prediction_horizon", 1.2)});
  auto node = std::make_shared<nav2_util::LifecycleNode>("people_unknown_test", "", options);
  tf2_ros::Buffer tf(node->get_clock());
  nav2_costmap_2d::LayeredCostmap map("odom", false, true);
  map.resizeMap(100, 100, .1, -5., -5.);
  auto layer = std::make_shared<TestPeopleLayer>();
  map.addPlugin(layer);
  layer->initialize(&map, "people", &tf, node, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  auto message = std::make_shared<unitree_social_navigation_msgs::msg::TrackedPersons>();
  message->header.frame_id = "odom";
  message->header.stamp = node->now();
  unitree_social_navigation_msgs::msg::TrackedPerson person;
  person.body_radius = .1;
  person.confidence = 1.;
  person.track_state = person.TRACK_STATE_CONFIRMED;
  person.velocity.twist.linear.x = 1.;
  message->persons.push_back(person);
  layer->receive(message);
  map.updateMap(0., 0., 0.);
  EXPECT_EQ(map.getCostmap()->getCost(58, 50), nav2_costmap_2d::NO_INFORMATION);
  rclcpp::shutdown();
}
