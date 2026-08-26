#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <g1_msgs/action/approach_object.hpp>
#include <g1_msgs/action/retreat.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stdexcept>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <utility>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_locomotion/approach_planner.hpp"

namespace g1_locomotion
{
using ApproachObject = g1_msgs::action::ApproachObject;
using Retreat = g1_msgs::action::Retreat;
using ApproachHandle = rclcpp_action::ServerGoalHandle<ApproachObject>;
using RetreatHandle = rclcpp_action::ServerGoalHandle<Retreat>;
constexpr double kMaxRetreatDistanceM = 2.0;

namespace
{
double wrap(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}
auto deadlineIn(double seconds)
{
  return std::chrono::steady_clock::now() +
         std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(seconds));
}
}  // namespace

class BaseApproachNode : public rclcpp::Node
{
public:
  BaseApproachNode()
  : Node("g1_base_approach"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    cmd_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    object_timeout_ms_ = declare_parameter<double>("object_timeout_ms", 1500.0);
    GaitLimits gait_defaults;
    gait_.min_speed_x_mps = declare_parameter<double>("min_speed_x_mps", gait_defaults.min_speed_x_mps);
    gait_.min_speed_y_mps = declare_parameter<double>("min_speed_y_mps", gait_defaults.min_speed_y_mps);
    gait_.max_speed_x_mps = declare_parameter<double>("max_speed_x_mps", gait_defaults.max_speed_x_mps);
    gait_.max_speed_y_mps = declare_parameter<double>("max_speed_y_mps", gait_defaults.max_speed_y_mps);
    gait_.max_yaw_rate_rps = declare_parameter<double>("max_yaw_rate_rps", gait_defaults.max_yaw_rate_rps);
    gait_.speed_per_m = declare_parameter<double>("speed_per_m", gait_defaults.speed_per_m);
    gait_.yaw_rate_per_rad = declare_parameter<double>("yaw_rate_per_rad", gait_defaults.yaw_rate_per_rad);
    retreat_speed_mps_ = declare_parameter<double>("retreat_speed_mps", 0.30);
    settle_s_ = declare_parameter<double>("settle_s", 1.0);
    cmd_rate_hz_ = declare_parameter<double>("cmd_rate_hz", 20.0);
    ApproachLimits defaults;
    limits_.target_x_m = declare_parameter<double>("target_x_m", defaults.target_x_m);
    limits_.target_y_m = declare_parameter<double>("target_y_m", defaults.target_y_m);
    limits_.forward_tolerance_m = declare_parameter<double>("forward_tolerance_m", defaults.forward_tolerance_m);
    limits_.lateral_tolerance_m = declare_parameter<double>("lateral_tolerance_m", defaults.lateral_tolerance_m);
    limits_.min_forward_m = declare_parameter<double>("min_forward_m", defaults.min_forward_m);
    limits_.heading_tolerance_rad = declare_parameter<double>("heading_tolerance_rad", defaults.heading_tolerance_rad);
    standoff_ids_ = declare_parameter<std::vector<std::string>>(
      "standoff_object_ids", std::vector<std::string>{});
    standoff_target_x_ = declare_parameter<std::vector<double>>(
      "standoff_target_x_m", std::vector<double>{});
    lookup_grace_s_ = declare_parameter<double>("lookup_grace_s", 3.0);
    default_timeout_s_ = declare_parameter<double>("default_timeout_s", 900.0);
    if (standoff_ids_.size() != standoff_target_x_.size()) {
      throw std::runtime_error("standoff object IDs and target distances must have equal lengths");
    }
    if (!limitsAreUsable(limits_) || !gaitLimitsAreUsable(gait_)) {
      throw std::runtime_error("configured approach or gait limits are unusable");
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 1);
    objects_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      "objects", rclcpp::SensorDataQoS(),
      [this](vision_msgs::msg::Detection3DArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(objects_mutex_); objects_ = std::move(msg);
      });
    approach_server_ = rclcpp_action::create_server<ApproachObject>(
      this, "~/approach_object",
      [this](const auto &, const auto &) {
        return acquire() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
               rclcpp_action::GoalResponse::REJECT;
      },
      [](const auto &) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<ApproachHandle> handle) {
        std::thread([this, handle]() {runGuarded([&]() {runApproach(handle);}, handle);}).detach();
      });
    retreat_server_ = rclcpp_action::create_server<Retreat>(
      this, "~/retreat",
      [this](const auto &, const Retreat::Goal::ConstSharedPtr goal) {
        if (goal->distance_m <= 0.0 || goal->distance_m > kMaxRetreatDistanceM) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return acquire() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
               rclcpp_action::GoalResponse::REJECT;
      },
      [](const auto &) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<RetreatHandle> handle) {
        std::thread([this, handle]() {runGuarded([&]() {runRetreat(handle);}, handle);}).detach();
      });
    RCLCPP_INFO(get_logger(), "base approach ready, publishing on %s", cmd_topic_.c_str());
  }

  ~BaseApproachNode() override
  {
    stopping_ = true;
    while (goals_running_ > 0) {std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    if (cmd_pub_) {publish(0.0, 0.0, 0.0);}
  }

private:
  bool acquire()
  {
    bool expected = false;
    return busy_.compare_exchange_strong(expected, true);
  }

  template<typename ActionT, typename Body>
  void runGuarded(Body body, const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
  {
    ++goals_running_;
    try {body();} catch (const std::exception & error) {
      publish(0.0, 0.0, 0.0);
      auto result = std::make_shared<typename ActionT::Result>();
      result->success = false;
      result->message = std::string("aborted on an internal error: ") + error.what();
      if (handle->is_executing() || handle->is_canceling()) {handle->abort(result);}
    }
    busy_ = false;
    --goals_running_;
  }

  std::chrono::duration<double> tickPeriod() const
  {
    return std::chrono::duration<double>(1.0 / std::max(1.0, cmd_rate_hz_));
  }
  void publish(double vx, double vy, double yaw)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = vx; msg.linear.y = vy; msg.angular.z = yaw; cmd_pub_->publish(msg);
  }
  void settle()
  {
    const auto until = deadlineIn(settle_s_);
    while (rclcpp::ok() && !stopping_ && std::chrono::steady_clock::now() < until) {
      publish(0.0, 0.0, 0.0); std::this_thread::sleep_for(tickPeriod());
    }
  }

  std::optional<geometry_msgs::msg::PoseStamped> basePose()
  {
    try {
      const auto tf = tf_buffer_.lookupTransform(
        fixed_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
      geometry_msgs::msg::PoseStamped pose;
      pose.header = tf.header;
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = tf.transform.translation.z;
      pose.pose.orientation = tf.transform.rotation;
      return pose;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no base pose: %s", error.what());
      return std::nullopt;
    }
  }

  std::optional<geometry_msgs::msg::PointStamped> objectInBase(const std::string & id)
  {
    vision_msgs::msg::Detection3DArray::SharedPtr snapshot;
    {std::lock_guard<std::mutex> lock(objects_mutex_); snapshot = objects_;}
    if (!snapshot || (now() - rclcpp::Time(snapshot->header.stamp)).seconds() * 1e3 > object_timeout_ms_) {
      return std::nullopt;
    }
    for (const auto & detection : snapshot->detections) {
      if (detection.results.empty() || detection.results.front().hypothesis.class_id != id) {continue;}
      geometry_msgs::msg::PointStamped point;
      point.header = snapshot->header;
      point.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      point.point = detection.results.front().pose.pose.position;
      try {return tf_buffer_.transform(point, base_frame_, tf2::durationFromSec(0.5));}
      catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "object transform failed: %s", error.what());
        return std::nullopt;
      }
    }
    return std::nullopt;
  }

  ApproachLimits limitsFor(const ApproachObject::Goal & goal) const
  {
    auto limits = limits_;
    if (goal.arm == ApproachObject::Goal::ARM_LEFT) {limits.target_y_m = -limits.target_y_m;}
    for (std::size_t i = 0; i < standoff_ids_.size(); ++i) {
      if (standoff_ids_[i] == goal.object_id) {limits.target_x_m = standoff_target_x_[i]; break;}
    }
    return limits;
  }

  void runApproach(const std::shared_ptr<ApproachHandle> handle)
  {
    const auto goal = handle->get_goal();
    auto result = std::make_shared<ApproachObject::Result>();
    auto feedback = std::make_shared<ApproachObject::Feedback>();
    if (goal->arm != ApproachObject::Goal::ARM_LEFT && goal->arm != ApproachObject::Goal::ARM_RIGHT) {
      result->message = "locating: arm must be 'left' or 'right'"; handle->abort(result); return;
    }
    const auto limits = limitsFor(*goal);
    const auto start = basePose();
    if (!start) {result->message = "locating: no base pose"; handle->abort(result); return;}
    const double working_yaw = goal->use_current_heading ?
      tf2::getYaw(start->pose.orientation) : goal->working_yaw;
    const double timeout = goal->timeout_s > 0.0 ? goal->timeout_s : default_timeout_s_;
    const auto deadline = deadlineIn(timeout);
    auto last_measured = std::chrono::steady_clock::now();
    int tick = 0;
    const int feedback_every = std::max(1, static_cast<int>(cmd_rate_hz_ / 2.0));
    while (rclcpp::ok() && !stopping_) {
      if (handle->is_canceling()) {
        publish(0, 0, 0); result->message = "closing: cancelled"; handle->canceled(result); return;
      }
      if (std::chrono::steady_clock::now() > deadline) {
        publish(0, 0, 0); result->message = "closing: timed out"; handle->abort(result); return;
      }
      const auto object = objectInBase(goal->object_id);
      const auto here = basePose();
      if (!object || !here) {
        publish(0, 0, 0);
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_measured).count() > lookup_grace_s_) {
          result->message = "locating: no fresh object or base pose"; handle->abort(result); return;
        }
        std::this_thread::sleep_for(tickPeriod()); continue;
      }
      last_measured = std::chrono::steady_clock::now();
      const auto command = planApproach(
        object->point.x, object->point.y,
        wrap(working_yaw - tf2::getYaw(here->pose.orientation)), limits, gait_);
      result->final_x_m = object->point.x; result->final_y_m = object->point.y;
      feedback->forward_error_m = command.forward_error_m;
      feedback->lateral_error_m = command.lateral_error_m;
      if (command.state == ApproachState::kOvershot || command.state == ApproachState::kInvalid) {
        publish(0, 0, 0); result->message = "closing: approach cannot continue"; handle->abort(result); return;
      }
      if (command.state == ApproachState::kArrived) {
        feedback->phase = ApproachObject::Feedback::PHASE_VERIFYING;
        handle->publish_feedback(feedback); settle();
        const auto settled = objectInBase(goal->object_id);
        if (!settled) {continue;}
        const auto verdict = planApproach(settled->point.x, settled->point.y, 0.0, limits, gait_);
        result->final_x_m = settled->point.x; result->final_y_m = settled->point.y;
        if (verdict.state != ApproachState::kArrived) {continue;}
        result->success = true; result->message = "object is in the reach window";
        handle->succeed(result); return;
      }
      if (tick++ % feedback_every == 0) {
        feedback->phase = ApproachObject::Feedback::PHASE_CLOSING; handle->publish_feedback(feedback);
      }
      publish(command.vx_mps, command.vy_mps, command.yaw_rate_rps);
      std::this_thread::sleep_for(tickPeriod());
    }
    publish(0, 0, 0); result->message = "closing: shutting down"; handle->abort(result);
  }

  void runRetreat(const std::shared_ptr<RetreatHandle> handle)
  {
    const auto goal = handle->get_goal();
    auto result = std::make_shared<Retreat::Result>();
    auto feedback = std::make_shared<Retreat::Feedback>();
    const auto start = basePose();
    if (!start) {result->message = "backing_off: no base pose"; handle->abort(result); return;}
    feedback->phase = Retreat::Feedback::PHASE_BACKING_OFF; handle->publish_feedback(feedback);
    const double timeout = goal->timeout_s > 0.0 ? goal->timeout_s : default_timeout_s_;
    const auto deadline = deadlineIn(timeout);
    auto blind_deadline = deadlineIn(lookup_grace_s_);
    double travelled = 0.0;
    int tick = 0;
    const int feedback_every = std::max(1, static_cast<int>(cmd_rate_hz_ / 2.0));
    while (rclcpp::ok() && !stopping_ && travelled < goal->distance_m &&
      std::chrono::steady_clock::now() < deadline)
    {
      if (handle->is_canceling()) {
        publish(0, 0, 0); result->travelled_m = travelled;
        result->message = "backing_off: cancelled"; handle->canceled(result); return;
      }
      if (const auto here = basePose()) {
        travelled = std::hypot(
          here->pose.position.x - start->pose.position.x,
          here->pose.position.y - start->pose.position.y);
        blind_deadline = deadlineIn(lookup_grace_s_);
      } else {
        publish(0, 0, 0);
        if (std::chrono::steady_clock::now() > blind_deadline) {
          result->travelled_m = travelled; result->message = "backing_off: lost the base pose";
          handle->abort(result); return;
        }
        std::this_thread::sleep_for(tickPeriod()); continue;
      }
      publish(-retreat_speed_mps_, 0, 0);
      if (tick++ % feedback_every == 0) {
        feedback->travelled_m = travelled; handle->publish_feedback(feedback);
      }
      std::this_thread::sleep_for(tickPeriod());
    }
    settle();
    if (const auto here = basePose()) {
      travelled = std::hypot(
        here->pose.position.x - start->pose.position.x,
        here->pose.position.y - start->pose.position.y);
    }
    result->travelled_m = travelled;
    result->success = travelled >= goal->distance_m;
    result->message = result->success ? "retreat complete" : "backing_off: distance not reached";
    if (result->success) {handle->succeed(result);} else {handle->abort(result);}
  }

  std::string cmd_topic_, base_frame_, fixed_frame_;
  double object_timeout_ms_{1500.0}, retreat_speed_mps_{0.30}, settle_s_{1.0};
  double cmd_rate_hz_{20.0}, lookup_grace_s_{3.0}, default_timeout_s_{900.0};
  ApproachLimits limits_;
  GaitLimits gait_;
  std::vector<std::string> standoff_ids_;
  std::vector<double> standoff_target_x_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr objects_sub_;
  rclcpp_action::Server<ApproachObject>::SharedPtr approach_server_;
  rclcpp_action::Server<Retreat>::SharedPtr retreat_server_;
  std::mutex objects_mutex_;
  vision_msgs::msg::Detection3DArray::SharedPtr objects_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::atomic<bool> busy_{false}, stopping_{false};
  std::atomic<int> goals_running_{0};
};
}  // namespace g1_locomotion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<g1_locomotion::BaseApproachNode>();
    executor.add_node(node); executor.spin(); rclcpp::shutdown(); return 0;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("g1_base_approach"), "fatal: %s", error.what());
    rclcpp::shutdown(); return 1;
  }
}
