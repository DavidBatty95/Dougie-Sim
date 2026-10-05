# Geoff-Sim G1 MoveIt Config

[Workspace map](../README.md) · [Robot description](../unitree_g1_description/README.md) · [Manipulation actions](../g1_manipulation/README.md)

This package is the first MoveIt gate for the local Geoff-Sim G1 model. It plans only while the
robot is stationary and uses the checked-in
`unitree_g1_description/urdf/g1_29dof_mode_15_brainco_hand.urdf` description.

## Scope

| Group | Definition | Active joints |
|---|---|---:|
| `left_arm` | `torso_link` to `left_base_link` | 7 |
| `right_arm` | `torso_link` to `right_base_link` | 7 |
| `both_arms` | composed from both arm groups | 14 |

The waist, legs, floating-base surrogate, and BrainCo fingers are not planning groups. The palm
base is the arm tip, but no hand joint belongs to an arm. Hand planning and grasp control are a
later gate.

The model has no virtual joint in the SRDF. This is intentional for stationary manipulation: the
local URDF remains rooted at `base_footprint`, and the arm chains remain rooted at `torso_link` so
MoveIt cannot request waist motion that the arm trajectory controller does not execute. Do not use
this configuration for reaching while walking.

## Planning And Execution

Humble's packaged KDL plugin solves each 7-DoF chain. `both_arms` has no direct kinematics entry
because it is a branched composite group; MoveIt uses the two subgroup solvers. OMPL RRTConnect is
configured for all three groups.

MoveIt sends trajectories to:

```text
/arm_trajectory_controller/follow_joint_trajectory
```

The controller mapping is arms only and follows the local body motor order: all seven left joints,
then all seven right joints. MoveIt does not spawn, activate, or switch the controller. Geoff-Sim
bringup must provide the action and complete `/joint_states` before execution can work.

Launch planning with:

Run inside a sourced ROS container; the robot state and controller prerequisites
above must be satisfied separately.

```bash
ros2 launch g1_moveit_config move_group.launch.py
```

Named targets `home`, `zero`, `tucked`, `ready`, `reach_front`, and `carry` are available on each
single arm and on `both_arms`. Their values are adapted from Grove G1 and constrained to the local
URDF limits; they are starting postures, not substitutes for runtime collision checking.

## Deliberate Omissions

There is no octomap configuration. A future perception gate must name a real local pointcloud topic
and a TF-connected frame before adding one. There are no hand groups or end effectors
in this configuration. Experimental BrainCo contact-bench work exists separately;
it does not establish a validated mobile grasp pipeline. Attached-object lifecycle
is handled by the manipulation server rather than this configuration package.

The allowed-collision matrix contains only direct URDF adjacency where both links have collision
geometry. It includes the non-planning legs and fingers because MoveIt collision-checks the whole
model, but it contains no sampled "never collides" pairs and no cross-arm disable.

## Tests

The static test parses the source URDF and SRDF, reconstructs both chains, checks group composition,
named targets, configured limits, controller order/action, the absence of hand groups, and every
collision disable. It requires no simulator or ROS graph.

```bash
colcon test --packages-select g1_moveit_config
```

## Configuration map and next steps

[`config/`](config/) contains the SRDF, kinematics, limits and controller/planner
settings. [`launch/`](launch/) loads them, and [`test/`](test/) checks their
consistency against the local model. Update these together when changing joint
groups or arm limits. S3–S5 of the [project plan](../../../docs/PROJECT_PLAN.md)
cover contact grasping, collision-aware approaches and coordinated loaded motion.
