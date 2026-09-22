# Licensing inventory

The departing maintainer requested **preservation of existing licenses**, not
assignment of a new blanket root license. There is intentionally no new root
license grant for previously unlicensed original scripts or documentation.
The incoming team should confirm that policy before broader redistribution.

| Material | Existing declaration / authority |
| --- | --- |
| `unitree_g1_mujoco_bridge`, `unitree_g1_sim_bringup`, `unitree_g1_description`, `dynamic_obstacle_benchmarks`, social messages/costmap packages | Apache-2.0 in their `package.xml` files |
| `g1_locomotion`, `g1_msgs`, manipulation/MoveIt/task-orchestration packages | BSD-3-Clause in their `package.xml` files |
| MuJoCo, Unitree repositories and MJLab | Their own source/submodule notices and licenses |
| Grove-G1 integration | Its integration/upstream notices; optional sources are not bundled by the setup |
| Pedestrian meshes/textures and generated derivatives | Bundled through the simulator fork; asset redistribution provenance is not fully documented in this root repository and needs owner confirmation |
| Downloaded policies and optional training checkpoints | Their upstream artifact terms; checksums establish identity, not redistribution permission |

Keep upstream copyright/license headers when modifying files. The checked-in
source patches do not replace their underlying licenses. Generated natural-walk
meshes retain the licensing obligations of their input assets.
