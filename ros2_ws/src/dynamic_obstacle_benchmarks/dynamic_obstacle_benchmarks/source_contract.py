"""Runtime checks for the onboard navigation information boundary."""

NAVIGATION_NODES = {
    "onboard_lidar_odometry", "slam_toolbox", "dynamic_obstacle_tracker",
    "human_aware_motion_planner", "human_collision_avoidance",
    "g1_policy_cmd_vel_controller", "controller_server", "planner_server",
    "local_costmap", "global_costmap", "scan_dynamic_filter", "livox_to_scan",
}
FORBIDDEN_TOPICS = {
    "/mujoco/pedestrians", "/simulation/static_map", "/evaluation/odom",
    "/livox/lidar_world", "/rt/sportmodestate", "/benchmark/people_states",
}
OPTIONAL_NAVIGATION_NODES = {"dynamic_route_monitor", "mujoco_goal_visualizer"}


def audit_onboard_graph(node):
    present = set()
    errors = []
    for name, namespace in node.get_node_names_and_namespaces():
        if name not in NAVIGATION_NODES | OPTIONAL_NAVIGATION_NODES:
            continue
        present.add(name)
        topics = {topic for topic, _ in node.get_subscriber_names_and_types_by_node(name, namespace)}
        for topic in sorted(topics & FORBIDDEN_TOPICS):
            errors.append(f"{namespace}/{name} consumes privileged {topic}")
    missing = NAVIGATION_NODES - present
    if missing:
        errors.append("missing navigation nodes: " + ",".join(sorted(missing)))
    publishers = {endpoint.node_name for endpoint in node.get_publishers_info_by_topic("/odom")}
    if publishers != {"onboard_lidar_odometry"}:
        errors.append("unexpected /odom owners: " + ",".join(sorted(publishers)))
    map_publishers = {endpoint.node_name for endpoint in node.get_publishers_info_by_topic("/map")}
    if map_publishers != {"slam_toolbox"}:
        errors.append("unexpected /map owners: " + ",".join(sorted(map_publishers)))
    if node.get_publishers_info_by_topic("/simulation/static_map"):
        errors.append("scene-derived map publisher present")
    return errors
