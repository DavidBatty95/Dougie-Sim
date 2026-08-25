# Grove-G1 source profile

This optional profile pins the complete
[`Adyansh04/grove-g1`](https://github.com/Adyansh04/grove-g1) repository under
`vendor/grove-g1`. Grove-G1 is a self-contained ROS 2 Jazzy/Ubuntu 24.04 stack with its own
workspace, dependencies, patches, Docker Compose file, and lifecycle scripts. It is not a package
dependency of this repository's ROS 2 Humble workspace and must not be copied into `ros2_ws/src` or
included in the Humble `colcon` build.

## Fetch and enter its container

On the host, from this repository root:

```bash
./scripts/setup_grove_g1.sh
cd vendor/grove-g1
cp .env.example .env
./scripts/import-externals.sh
./scripts/manage.sh start
./scripts/manage.sh exec
```

The first command verifies the exact Grove-G1 commit. The remaining commands are Grove-G1's own
documented dependency and Jazzy container workflow; review `vendor/grove-g1/README.md` before
building or running a simulation. Hardware use is not part of this integration profile.

## Runtime boundary

Stop the default stack before running Grove-G1 against the same simulator, robot, or DDS domain.
The stacks are alternative owners of the same interfaces, not composable overlays:

- Both can publish `rt/lowcmd`. Grove-G1's controller owns balance and all 29 body motors, so a
  second command publisher can cause conflicting actuator commands and a fall.
- Both provide overlapping world/odometry/robot TF chains. Concurrent authorities make TF lookup
  nondeterministic and invalidate navigation and localization.
- Both publish simulated LiDAR, IMU, and related sensor streams. Duplicate topic/frame publishers
  can mix incompatible samples and corrupt perception and state estimation.

Process isolation alone is insufficient when host networking places both containers on the same
DDS graph. Use only one complete stack at a time for a given simulator or robot.

## Attribution and license

Source: `Adyansh04/grove-g1`, pinned at
`97bfb9ba13569a593b5e3ecfc5ed1d4208b2b681`. Grove-G1 is Copyright (c) 2026 Adyansh Gupta and is
distributed under the BSD 3-Clause License; its checked-out `LICENSE` file is authoritative.
Grove-G1 also fetches and vendors third-party components that retain their own licenses and notices.
