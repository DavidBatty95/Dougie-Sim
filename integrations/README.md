# Optional integration profiles

[Project home](../README.md) · [Repository guide](../docs/REPOSITORY_GUIDE.md) · [Workspace packages](../ros2_ws/src/README.md)

Integration profiles document external systems that can be explored alongside
Geoff-Sim while retaining their own dependencies and runtime assumptions.

| Profile | Environment | Relationship to the default stack |
| --- | --- | --- |
| [Grove-G1](grove_g1/README.md) | ROS 2 Jazzy / Ubuntu 24.04 | Optional complete external checkout; separate from the Humble workspace |

The local Humble manipulation packages include attributed ports of selected
interfaces and components. Running the entire Grove-G1 stack is a different
workflow and requires the profile's explicit ownership and DDS/TF boundaries.

When adding a profile, document the pinned source, setup commands, attribution,
runtime ownership, compatibility assumptions and a source-profile check. Link
the profile here and explain its relevance to the [project plan](../docs/PROJECT_PLAN.md).
