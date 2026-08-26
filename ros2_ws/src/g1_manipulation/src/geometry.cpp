#include "g1_manipulation/geometry.hpp"

#include <cmath>
#include <limits>

namespace g1_manipulation
{

std::optional<ArmContext> resolveArm(
  const std::string & arm, const std::string & left_tip, const std::string & right_tip)
{
  if (arm == "left" && !left_tip.empty()) {
    return ArmContext{"left", "left_arm", left_tip, true};
  }
  if (arm == "right" && !right_tip.empty()) {
    return ArmContext{"right", "right_arm", right_tip, false};
  }
  return std::nullopt;
}

bool validPostureGroup(const std::string & group)
{
  return group == "left_arm" || group == "right_arm" || group == "both_arms";
}

bool validVector(const Vector3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool validPose(const Pose & pose)
{
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return validVector(pose.position) && std::isfinite(norm) && norm > 1e-12;
}

Quaternion mirroredGraspOrientation(const std::array<double, 3> & rpy, bool is_left)
{
  const double roll = (is_left ? -1.0 : 1.0) * rpy[0];
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(rpy[1] * 0.5);
  const double sp = std::sin(rpy[1] * 0.5);
  const double cy = std::cos(rpy[2] * 0.5);
  const double sy = std::sin(rpy[2] * 0.5);
  return Quaternion{
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy};
}

Pose graspTipPose(
  const Pose & object_pose, const Vector3 & tip_to_object_offset,
  const std::array<double, 3> & grasp_rpy, bool is_left)
{
  Pose result;
  result.position.x = object_pose.position.x + tip_to_object_offset.x;
  result.position.y = object_pose.position.y + tip_to_object_offset.y;
  result.position.z = object_pose.position.z + tip_to_object_offset.z;
  result.orientation = mirroredGraspOrientation(grasp_rpy, is_left);
  return result;
}

Pose above(const Pose & pose, double height)
{
  Pose result = pose;
  result.position.z += height;
  return result;
}

double placementCenterHeight(double surface_center_z, double surface_height, double held_height)
{
  if (!std::isfinite(surface_center_z) || !std::isfinite(surface_height) ||
    !std::isfinite(held_height) || surface_height <= 0.0 || held_height <= 0.0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return surface_center_z + 0.5 * surface_height + 0.5 * held_height;
}

}  // namespace g1_manipulation
