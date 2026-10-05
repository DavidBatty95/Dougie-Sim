# Social costmap and Nav2 integration

[Workspace map](../README.md) · [Architecture](../../../docs/ARCHITECTURE.md) · [Route planning](../../../docs/DYNAMIC_ROUTE_PLANNING.md)

This C++ package integrates people-aware costmap behavior and behavior-tree
support with Nav2. Current people footprints and predicted motion have distinct
roles in navigation costs; the final collision supervisor remains a separate
component in the bridge package.

## Find the code

- [`src/`](src/) and [`include/`](include/) contain the implementation and interfaces.
- [`CMakeLists.txt`](CMakeLists.txt) defines plugin builds and test targets.
- Bringup [`config/`](../unitree_g1_sim_bringup/config/) supplies the runtime
  Nav2 profiles; the package is loaded as part of the navigation stack.

When changing costmap behavior, check input frame/time semantics, current versus
predicted-person treatment and route replanning effects. Run the package tests
in the supported ROS environment and use scored cases to assess behavioral
effects. See [development](../../../docs/DEVELOPMENT.md) and the
[S2 navigation plan](../../../docs/PROJECT_PLAN.md).
