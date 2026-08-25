#ifndef G1_LOCOMOTION__APPROACH_PLANNER_HPP_
#define G1_LOCOMOTION__APPROACH_PLANNER_HPP_

#include <cstdint>

namespace g1_locomotion
{

enum class ApproachState : std::uint8_t
{
  kArrived,
  kClosing,
  kOvershot,
  kInvalid,
};

struct ApproachLimits
{
  double target_x_m = 0.270;
  double target_y_m = -0.220;
  double forward_tolerance_m = 0.050;
  double lateral_tolerance_m = 0.040;
  double min_forward_m = 0.050;
  double heading_tolerance_rad = 0.350;
};

struct GaitLimits
{
  double min_speed_x_mps = 0.20;
  double min_speed_y_mps = 0.25;
  double max_speed_x_mps = 0.40;
  double max_speed_y_mps = 0.35;
  double max_yaw_rate_rps = 0.60;
  double speed_per_m = 1.0;
  double yaw_rate_per_rad = 1.0;
};

struct ApproachCommand
{
  ApproachState state = ApproachState::kInvalid;
  double forward_error_m = 0.0;
  double lateral_error_m = 0.0;
  double vx_mps = 0.0;
  double vy_mps = 0.0;
  double yaw_rate_rps = 0.0;
};

bool limitsAreUsable(const ApproachLimits & limits);
bool gaitLimitsAreUsable(const GaitLimits & gait);
ApproachCommand planApproach(
  double object_x_m, double object_y_m, double heading_error_rad,
  const ApproachLimits & limits, const GaitLimits & gait);

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__APPROACH_PLANNER_HPP_
