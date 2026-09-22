#include "unitree_social_costmap/people_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"

namespace unitree_social_costmap
{
void PeopleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("People layer requires a lifecycle node");}
  declareParameter("enabled", rclcpp::ParameterValue(false));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/tracked_persons")));
  declareParameter("timeout", rclcpp::ParameterValue(0.6));
  declareParameter("margin", rclcpp::ParameterValue(0.1));
  declareParameter("prediction_horizon", rclcpp::ParameterValue(0.0));
  declareParameter("prediction_step", rclcpp::ParameterValue(0.3));
  declareParameter("prediction_cost", rclcpp::ParameterValue(200));
  std::string topic;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic);
  node->get_parameter(name_ + ".timeout", timeout_);
  node->get_parameter(name_ + ".margin", margin_);
  node->get_parameter(name_ + ".prediction_horizon", prediction_horizon_);
  node->get_parameter(name_ + ".prediction_step", prediction_step_);
  node->get_parameter(name_ + ".prediction_cost", prediction_cost_);
  if (!std::isfinite(timeout_) || !std::isfinite(margin_) || timeout_ <= 0.0 || margin_ < 0.0) {
    throw std::invalid_argument("Invalid people-layer timeout/margin");
  }
  if (!std::isfinite(prediction_horizon_) || !std::isfinite(prediction_step_) ||
    prediction_horizon_ < 0.0 || prediction_horizon_ > 3.0 || prediction_step_ < 0.1 ||
    prediction_cost_ < 1 || prediction_cost_ >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    throw std::invalid_argument("Invalid people-layer prediction parameters");
  }
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<People>(topic, rclcpp::QoS(1),
    [this](People::SharedPtr message) {receive(std::move(message));}, options);
  current_ = !enabled_;
}

void PeopleLayer::receive(People::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = std::move(message);
}

void PeopleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.reset();
  // Keep the old bounds: the next update must remove the previous footprints.
  current_ = !enabled_;
}

void PeopleLayer::updateBounds(double, double, double, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  *min_x = std::min(*min_x, last_min_x_);
  *min_y = std::min(*min_y, last_min_y_);
  *max_x = std::max(*max_x, last_max_x_);
  *max_y = std::max(*max_y, last_max_y_);
  last_min_x_ = last_min_y_ = std::numeric_limits<double>::infinity();
  last_max_x_ = last_max_y_ = -std::numeric_limits<double>::infinity();
  snapshot_.clear();
  current_ = !enabled_;
  if (!enabled_) {return;}
  People::SharedPtr people;
  {std::lock_guard<std::mutex> lock(mutex_); people = latest_;}
  if (!people) {return;}
  const double age = (clock_->now() - rclcpp::Time(people->header.stamp)).seconds();
  if (age < 0.0 || age > timeout_) {return;}
  double tx = 0.0, ty = 0.0, c = 1.0, s = 0.0;
  if (people->header.frame_id != layered_costmap_->getGlobalFrameID()) {
    try {
      auto transform = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), people->header.frame_id, tf2::TimePointZero);
      const auto & q = transform.transform.rotation;
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      tx = transform.transform.translation.x;
      ty = transform.transform.translation.y;
      c = std::cos(yaw); s = std::sin(yaw);
    } catch (const tf2::TransformException &) {return;}
  }
  for (const auto & person : people->persons) {
    const bool confirmed = person.track_state == person.TRACK_STATE_CONFIRMED;
    const double observation_age = person.observation_age.sec + person.observation_age.nanosec * 1e-9;
    const bool recent_lost = prediction_horizon_ > 0.0 && person.track_state == person.TRACK_STATE_LOST && observation_age + age <= timeout_;
    if ((!confirmed && !recent_lost) || !std::isfinite(person.confidence) || person.confidence < 0.5) {continue;}
    const double x = person.pose.pose.position.x + age * person.velocity.twist.linear.x;
    const double y = person.pose.pose.position.y + age * person.velocity.twist.linear.y;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(person.body_radius)) {continue;}
    const double vx = person.velocity.twist.linear.x, vy = person.velocity.twist.linear.y;
    const double horizon = std::hypot(vx, vy) >= 0.1 ? prediction_horizon_ : 0.0;
    for (double offset = 0.0; offset <= horizon + 1e-6; offset += prediction_step_) {
      const double px = x + offset * vx, py = y + offset * vy;
      const auto cost = offset == 0.0 && confirmed ? nav2_costmap_2d::LETHAL_OBSTACLE :
        // Decay slowly enough that the forecast adds information beyond the
        // inflation halo of the current body, while remaining traversable.
        static_cast<unsigned char>(prediction_cost_ * std::exp(-0.35 * offset));
      Disc disc{tx + c * px - s * py, ty + s * px + c * py,
        std::clamp<double>(person.body_radius, 0.1, 1.0) + margin_, cost};
      snapshot_.push_back(disc);
      last_min_x_ = std::min(last_min_x_, disc.x - disc.radius);
      last_min_y_ = std::min(last_min_y_, disc.y - disc.radius);
      last_max_x_ = std::max(last_max_x_, disc.x + disc.radius);
      last_max_y_ = std::max(last_max_y_, disc.y + disc.radius);
    }
  }
  *min_x = std::min(*min_x, last_min_x_);
  *min_y = std::min(*min_y, last_min_y_);
  *max_x = std::max(*max_x, last_max_x_);
  *max_y = std::max(*max_y, last_max_y_);
  current_ = true;
}

void PeopleLayer::updateCosts(nav2_costmap_2d::Costmap2D & grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {return;}
  // LayeredCostmap resets the dirty rectangle before restoring each layer.
  // Repaint only current people, so empty messages and movement clear ghosts
  // without erasing static obstacles supplied by the underlying static layer.
  for (const auto & disc : snapshot_) {
    int x0, y0, x1, y1;
    grid.worldToMapEnforceBounds(disc.x - disc.radius, disc.y - disc.radius, x0, y0);
    grid.worldToMapEnforceBounds(disc.x + disc.radius, disc.y + disc.radius, x1, y1);
    for (int y = std::max(min_j, y0); y < std::min(max_j, y1 + 1); ++y) {
      for (int x = std::max(min_i, x0); x < std::min(max_i, x1 + 1); ++x) {
        double wx, wy;
        grid.mapToWorld(x, y, wx, wy);
        if (std::hypot(wx - disc.x, wy - disc.y) <= disc.radius) {
          const auto previous = grid.getCost(x, y);
          if (disc.cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
            (previous != nav2_costmap_2d::NO_INFORMATION && disc.cost > previous))
          {
            grid.setCost(x, y, disc.cost);
          }
        }
      }
    }
  }
}
}  // namespace unitree_social_costmap

PLUGINLIB_EXPORT_CLASS(unitree_social_costmap::PeopleLayer, nav2_costmap_2d::Layer)
