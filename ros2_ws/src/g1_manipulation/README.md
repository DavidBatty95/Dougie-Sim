# g1_manipulation

Manipulation gate 3 provides real ROS 2 action servers for `g1_msgs/action/Pick`,
`Place`, and `SetArmPosture`. Motion is planned and then executed with MoveIt's
`MoveGroupInterface` against the local `g1_moveit_config` groups.

This package is explicitly simulation-first. Grasp and release use the MuJoCo
bridge's `Trigger` services. It does not claim hardware support, acquire controller
authority, start `move_group`, or start a controller. The existing
`/arm_trajectory_controller/follow_joint_trajectory` controller must already have
authority before execution.

## Interfaces

Actions are `/g1_manipulation_server/pick`, `/place`, and `/set_arm_posture`.
Pick and place accept only `left` or `right`; named posture accepts only
`left_arm`, `right_arm`, or `both_arms`. One goal may run globally across all
three servers.

Object data comes from `/objects` as `vision_msgs/Detection3DArray`. The server
matches `results.front().hypothesis.class_id`, reads
`results.front().pose.pose`, requires a fresh array stamp, transforms poses to
the MoveIt planning frame, and uses a padded bounding-box collision object.

Pick phases are locate, pregrasp, approach, grasp service, attach to the
configured tip, and lift. Place requires an object attached to that tip, prefers
`surface_object_id` over the goal pose, places the object's center at surface top
plus half its own height, lowers, releases, detaches, retreats, and checks a fresh
final detection when one is available. Trigger service failures and timeouts
abort the action.

There are no MoveIt hand groups, no allowed-collision-matrix edits, and no
octomap collision exemptions. The target world collision object is removed only
for the final pick approach and then represented as an attached body.

## Running

Start MoveIt separately, ensure the arm trajectory controller has authority, and
start the simulation manipulation I/O before this server:

```bash
ros2 launch g1_moveit_config move_group.launch.py
ros2 launch g1_manipulation manipulation.launch.py
```

The launch file starts only `g1_manipulation_server`; it supplies the same robot
description, SRDF, kinematics, joint limits, and OMPL settings used by the local
MoveIt package so `MoveGroupInterface` can initialize.

Configuration is in `config/manipulation.yaml`. `tip_to_object_offset` is the tip
position relative to the object, expressed in the planning frame, not rotated by
the object. `grasp_rpy` describes the right-side orientation and its roll is
mirrored for the left side.

## Verification

Pure geometry is isolated from ROS and MoveIt in `geometry.hpp`/`geometry.cpp`.
The gtests cover arm resolution, mirrored orientation, tip offset and vertical
approach/lift geometry, placement height, detection lookup, and invalid inputs.

```bash
colcon test --packages-select g1_manipulation
```

See `NOTICE` for the pinned Grove G1 source attribution.
