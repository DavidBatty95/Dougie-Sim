# Social-navigation interfaces

[Workspace map](../README.md) · [Bridge and autonomy](../unitree_g1_mujoco_bridge/README.md) · [Social costmap](../unitree_social_costmap/README.md)

This package defines the ROS messages shared by person tracking, prediction,
planning and navigation visualization. The authoritative field definitions are
in [`msg/`](msg/); dependencies and interface generation are specified in
[`CMakeLists.txt`](CMakeLists.txt) and [`package.xml`](package.xml).

Treat frame IDs, timestamps, identity and prediction semantics as shared
contracts. When changing a message, rebuild producers and consumers, update
source-contract tests, and check the corresponding RViz displays and benchmark
interpretation. Message compatibility alone does not establish observation
freshness or correct frame alignment.

See [onboard source boundaries](../../../docs/ONBOARD_SOCIAL_NAVIGATION.md) for
which observations may reach navigation, and [development](../../../docs/DEVELOPMENT.md)
for the supported build environment.
