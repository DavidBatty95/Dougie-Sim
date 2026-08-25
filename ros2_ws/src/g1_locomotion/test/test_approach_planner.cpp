#include <gmock/gmock.h>

#include "g1_locomotion/approach_planner.hpp"

namespace
{
using g1_locomotion::ApproachLimits;
using g1_locomotion::ApproachState;
using g1_locomotion::GaitLimits;
using g1_locomotion::gaitLimitsAreUsable;
using g1_locomotion::limitsAreUsable;
using g1_locomotion::planApproach;

ApproachLimits defaults() {return ApproachLimits{};}
GaitLimits gait() {return GaitLimits{};}
auto plan(double x, double y, const ApproachLimits & limits)
{
  return planApproach(x, y, 0.0, limits, gait());
}

TEST(ApproachPlanner, ObjectInTheWindowIsArrivedAndCommandsNothing)
{
  const auto command = plan(0.270, -0.220, defaults());
  EXPECT_EQ(command.state, ApproachState::kArrived);
  EXPECT_DOUBLE_EQ(command.vx_mps, 0.0);
  EXPECT_DOUBLE_EQ(command.vy_mps, 0.0);
  EXPECT_DOUBLE_EQ(command.yaw_rate_rps, 0.0);
}

TEST(ApproachPlanner, AnErrorJustOutsideTheWindowStillClearsTheGaitDeadband)
{
  auto limits = defaults();
  const auto command = plan(
    limits.target_x_m + limits.forward_tolerance_m + 0.001, -0.220, limits);
  EXPECT_EQ(command.state, ApproachState::kClosing);
  EXPECT_GE(command.vx_mps, gait().min_speed_x_mps);
}

TEST(ApproachPlanner, ASmallLateralErrorClearsTheLateralFloorWhichIsHigher)
{
  auto limits = defaults();
  const auto command = plan(
    0.270, limits.target_y_m + limits.lateral_tolerance_m + 0.001, limits);
  EXPECT_EQ(command.state, ApproachState::kClosing);
  EXPECT_GE(command.vy_mps, gait().min_speed_y_mps);
}

TEST(ApproachPlanner, ABigErrorIsCappedRatherThanScaledUp)
{
  const auto command = plan(2.0, 1.5, defaults());
  EXPECT_EQ(command.state, ApproachState::kClosing);
  EXPECT_DOUBLE_EQ(command.vx_mps, gait().max_speed_x_mps);
  EXPECT_DOUBLE_EQ(command.vy_mps, gait().max_speed_y_mps);
}

TEST(ApproachPlanner, BothAxesAreDrivenAtOnce)
{
  const auto command = plan(0.600, 0.100, defaults());
  EXPECT_GT(command.vx_mps, 0.0);
  EXPECT_GT(command.vy_mps, 0.0);
}

TEST(ApproachPlanner, VelocitiesPointAtTheError)
{
  EXPECT_GT(plan(0.600, -0.220, defaults()).vx_mps, 0.0);
  EXPECT_LT(plan(0.150, -0.220, defaults()).vx_mps, 0.0);
  EXPECT_GT(plan(0.270, 0.100, defaults()).vy_mps, 0.0);
  EXPECT_LT(plan(0.270, -0.500, defaults()).vy_mps, 0.0);
}

TEST(ApproachPlanner, PastTheWindowIsRecoveredByReversing)
{
  const auto command = plan(0.120, -0.220, defaults());
  EXPECT_EQ(command.state, ApproachState::kClosing);
  EXPECT_LE(command.vx_mps, -gait().min_speed_x_mps);
}

TEST(ApproachPlanner, OnlyTheObjectBeingUnderTheRobotIsTerminal)
{
  const auto limits = defaults();
  EXPECT_EQ(plan(limits.min_forward_m - 0.001, -0.220, limits).state, ApproachState::kOvershot);
  EXPECT_EQ(plan(limits.min_forward_m + 0.001, -0.220, limits).state, ApproachState::kClosing);
}

TEST(ApproachPlanner, HeadingIsHeldWhileClosingButIsNotPartOfArriving)
{
  const auto limits = defaults();
  const double off = limits.heading_tolerance_rad + 0.2;
  EXPECT_EQ(planApproach(0.270, -0.220, off, limits, gait()).state, ApproachState::kArrived);
  EXPECT_DOUBLE_EQ(planApproach(0.270, -0.220, off, limits, gait()).yaw_rate_rps, 0.0);
  EXPECT_GT(planApproach(0.600, -0.220, off, limits, gait()).yaw_rate_rps, 0.0);
  EXPECT_LT(planApproach(0.600, -0.220, -off, limits, gait()).yaw_rate_rps, 0.0);
}

TEST(ApproachPlanner, SmallHeadingErrorsAreLeftAloneRatherThanFloored)
{
  const auto limits = defaults();
  EXPECT_DOUBLE_EQ(
    planApproach(
      0.600, -0.220, limits.heading_tolerance_rad - 0.01, limits, gait()).yaw_rate_rps,
    0.0);
}

TEST(ApproachPlanner, YawIsCappedInBothDirections)
{
  EXPECT_DOUBLE_EQ(
    planApproach(0.600, -0.220, 3.0, defaults(), gait()).yaw_rate_rps,
    gait().max_yaw_rate_rps);
  EXPECT_DOUBLE_EQ(
    planApproach(0.600, -0.220, -3.0, defaults(), gait()).yaw_rate_rps,
    -gait().max_yaw_rate_rps);
}

TEST(ApproachPlanner, ErrorsAreReportedWhateverTheState)
{
  const auto command = plan(0.500, -0.100, defaults());
  EXPECT_NEAR(command.forward_error_m, 0.230, 1e-9);
  EXPECT_NEAR(command.lateral_error_m, 0.120, 1e-9);
}

TEST(ApproachPlanner, UnusableLimitsAreRefusedRatherThanAimedAt)
{
  EXPECT_TRUE(limitsAreUsable(defaults()));
  auto no_window = defaults();
  no_window.forward_tolerance_m = 0.0;
  EXPECT_FALSE(limitsAreUsable(no_window));
  auto floor_too_high = defaults();
  floor_too_high.min_forward_m = floor_too_high.target_x_m;
  EXPECT_FALSE(limitsAreUsable(floor_too_high));
  EXPECT_EQ(plan(0.270, -0.220, no_window).state, ApproachState::kInvalid);
}

TEST(ApproachPlanner, UnusableGaitLimitsAreRefusedRatherThanCommanded)
{
  EXPECT_TRUE(gaitLimitsAreUsable(gait()));
  auto inverted = gait();
  inverted.min_speed_x_mps = 0.9;
  inverted.max_speed_x_mps = 0.4;
  EXPECT_FALSE(gaitLimitsAreUsable(inverted));
  EXPECT_EQ(
    planApproach(0.600, -0.220, 0.0, defaults(), inverted).state,
    ApproachState::kInvalid);
}

TEST(ApproachPlanner, TheLeftArmWindowIsTheRightArmWindowMirrored)
{
  auto left = defaults();
  left.target_y_m = -left.target_y_m;
  EXPECT_EQ(plan(0.270, -0.220, defaults()).state, ApproachState::kArrived);
  EXPECT_EQ(plan(0.270, 0.220, left).state, ApproachState::kArrived);
  EXPECT_EQ(plan(0.270, 0.220, defaults()).state, ApproachState::kClosing);
}
}  // namespace
