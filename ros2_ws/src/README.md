# Source Workspace

This directory is populated by `../unitree.repos` when you run `scripts/setup_sources.sh` inside the container.

Expected checkouts:

- `unitree_mujoco`: Unitree Mujoco simulator and robot assets.
- `unitree_ros2`: ROS 2 message/interface and examples used by Unitree robots.
- `unitree_sdk2`: Unitree DDS SDK used by the simulator and ROS 2 bridge examples.

Local integration packages can live here as normal ROS 2 packages.
