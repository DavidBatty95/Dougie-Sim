# G1 robot description

[Workspace map](../README.md) · [MoveIt configuration](../g1_moveit_config/README.md)

This package supplies the ROS G1 robot description and meshes used for TF,
visualization, kinematics and arm planning.

- [`urdf/`](urdf/) contains `g1_29dof_mode_15_brainco_hand.urdf`, the local
  body-and-hand description used by MoveIt and transport kinematics.
- [`meshes/`](meshes/) contains the referenced visual/collision assets.
- [`package.xml`](package.xml) identifies build dependencies and package metadata.

Joint names, limits, axes and frame origins are integration contracts. Changes
must be reconciled with bridge joint order, MoveIt groups/controller mappings,
arm-kinematics tests and any generated MuJoCo hand model. The simulator's MJCF
assets live in `unitree_mujoco`; a URDF edit does not automatically update them.

The [development checks](../../../docs/DEVELOPMENT.md) cover description parsing
and related package tests. Preserve upstream attribution and the
[license inventory](../../../LICENSES.md) when changing assets.
