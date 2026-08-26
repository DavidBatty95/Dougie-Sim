#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "g1_manipulation/geometry.hpp"

using g1_manipulation::Pose;
using g1_manipulation::Vector3;

TEST(ArmResolution, ResolvesOnlySingleArmSides)
{
  const auto left = g1_manipulation::resolveArm("left", "left_tip", "right_tip");
  ASSERT_TRUE(left);
  EXPECT_EQ(left->group, "left_arm");
  EXPECT_EQ(left->tip_link, "left_tip");
  EXPECT_TRUE(left->is_left);

  const auto right = g1_manipulation::resolveArm("right", "left_tip", "right_tip");
  ASSERT_TRUE(right);
  EXPECT_EQ(right->group, "right_arm");
  EXPECT_EQ(right->tip_link, "right_tip");
  EXPECT_FALSE(right->is_left);

  EXPECT_FALSE(g1_manipulation::resolveArm("both", "left_tip", "right_tip"));
  EXPECT_FALSE(g1_manipulation::resolveArm("left_arm", "left_tip", "right_tip"));
  EXPECT_FALSE(g1_manipulation::resolveArm("left", "", "right_tip"));
}

TEST(Orientation, MirrorsConfiguredRoll)
{
  const std::array<double, 3> rpy{-M_PI_2, 0.0, 0.0};
  const auto right = g1_manipulation::mirroredGraspOrientation(rpy, false);
  const auto left = g1_manipulation::mirroredGraspOrientation(rpy, true);
  EXPECT_NEAR(right.x, -std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(left.x, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(right.w, left.w, 1e-12);
}

TEST(Geometry, AppliesPlanningFrameOffsetApproachAndLift)
{
  Pose object{{0.4, -0.2, 0.8}, {0.0, 0.0, 0.0, 1.0}};
  const auto grasp = g1_manipulation::graspTipPose(
    object, {-0.10, 0.02, 0.03}, {0.0, 0.0, 0.0}, false);
  EXPECT_DOUBLE_EQ(grasp.position.x, 0.3);
  EXPECT_DOUBLE_EQ(grasp.position.y, -0.18);
  EXPECT_DOUBLE_EQ(grasp.position.z, 0.83);
  const auto pregrasp = g1_manipulation::above(grasp, 0.15);
  const auto lift = g1_manipulation::above(grasp, 0.20);
  EXPECT_DOUBLE_EQ(pregrasp.position.z, 0.98);
  EXPECT_DOUBLE_EQ(lift.position.z, 1.03);
  EXPECT_DOUBLE_EQ(pregrasp.position.x, grasp.position.x);
}

TEST(Placement, ComputesSurfaceTopPlusHeldHalfHeight)
{
  EXPECT_DOUBLE_EQ(g1_manipulation::placementCenterHeight(0.7, 0.2, 0.06), 0.83);
  EXPECT_TRUE(std::isnan(g1_manipulation::placementCenterHeight(0.7, 0.0, 0.06)));
  EXPECT_TRUE(std::isnan(g1_manipulation::placementCenterHeight(0.7, 0.2, -0.1)));
}

TEST(Validation, RejectsInvalidInputs)
{
  EXPECT_TRUE(g1_manipulation::validPostureGroup("both_arms"));
  EXPECT_FALSE(g1_manipulation::validPostureGroup("left_hand"));
  EXPECT_FALSE(g1_manipulation::validPostureGroup(""));
  Pose pose{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
  EXPECT_FALSE(g1_manipulation::validPose(pose));
  pose.orientation.w = 1.0;
  EXPECT_TRUE(g1_manipulation::validPose(pose));
  pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(g1_manipulation::validPose(pose));
}
