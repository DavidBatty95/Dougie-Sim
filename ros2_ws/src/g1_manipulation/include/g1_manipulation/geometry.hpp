#ifndef G1_MANIPULATION__GEOMETRY_HPP_
#define G1_MANIPULATION__GEOMETRY_HPP_

#include <array>
#include <optional>
#include <string>

namespace g1_manipulation
{

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Pose
{
  Vector3 position;
  Quaternion orientation;
};

struct ArmContext
{
  std::string side;
  std::string group;
  std::string tip_link;
  bool is_left{false};
};

std::optional<ArmContext> resolveArm(
  const std::string & arm, const std::string & left_tip, const std::string & right_tip);
bool validPostureGroup(const std::string & group);
bool validVector(const Vector3 & value);
bool validPose(const Pose & pose);
Quaternion mirroredGraspOrientation(const std::array<double, 3> & rpy, bool is_left);
Pose graspTipPose(
  const Pose & object_pose, const Vector3 & tip_to_object_offset,
  const std::array<double, 3> & grasp_rpy, bool is_left);
Pose above(const Pose & pose, double height);
double placementCenterHeight(double surface_center_z, double surface_height, double held_height);

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__GEOMETRY_HPP_
