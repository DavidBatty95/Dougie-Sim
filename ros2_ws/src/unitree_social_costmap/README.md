# Social costmap and Nav2 integration

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../../README.md"><img src="../../../docs/media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../../README.md#quick-start"><img src="../../../docs/media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../../../docs/DEMO_GUIDE.md"><img src="../../../docs/media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../../../docs/INDEX.md"><img src="../../../docs/media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../../../docs/REPOSITORY_GUIDE.md"><img src="../../../docs/media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../../../docs/PROJECT_STATUS.md"><img src="../../../docs/media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../../../docs/PROJECT_PLAN.md"><img src="../../../docs/media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../../../docs/RESULTS_REVIEW.md"><img src="../../../docs/media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="../../../docs/literature_review/README.md"><img src="../../../docs/media/navigation/research.svg" width="132" height="44" alt="Research"></a>
  <a href="../../../docs/DEVELOPMENT.md"><img src="../../../docs/media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

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
