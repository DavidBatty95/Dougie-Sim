#pragma once

#include <limits>
#include <mutex>
#include <vector>
#include "nav2_costmap_2d/layer.hpp"
#include "unitree_social_navigation_msgs/msg/tracked_persons.hpp"

namespace unitree_social_costmap
{
class PeopleLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;
  void reset() override;
  bool isClearable() override {return true;}

protected:
  using People = unitree_social_navigation_msgs::msg::TrackedPersons;
  void receive(People::SharedPtr message);

private:
  struct Disc {double x; double y; double radius; unsigned char cost{254};};
  std::mutex mutex_;
  People::SharedPtr latest_;
  rclcpp::Subscription<People>::SharedPtr subscription_;
  std::vector<Disc> snapshot_;
  double timeout_{0.6};
  double margin_{0.1};
  double prediction_horizon_{0.0};
  double prediction_step_{0.3};
  int prediction_cost_{200};
  double last_min_x_{std::numeric_limits<double>::infinity()};
  double last_min_y_{std::numeric_limits<double>::infinity()};
  double last_max_x_{-std::numeric_limits<double>::infinity()};
  double last_max_y_{-std::numeric_limits<double>::infinity()};
};
}  // namespace unitree_social_costmap
