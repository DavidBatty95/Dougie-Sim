from types import SimpleNamespace as S

from dynamic_obstacle_benchmarks.benchmark_runner import BenchmarkRosNode
from dynamic_obstacle_benchmarks.observed_space import observed_free
from dynamic_obstacle_benchmarks.source_contract import NAVIGATION_NODES, audit_onboard_graph


class Graph:
    forbidden = None
    odom_owner = "onboard_lidar_odometry"

    def get_node_names_and_namespaces(self):
        return [(name, "/") for name in NAVIGATION_NODES]

    def get_subscriber_names_and_types_by_node(self, name, namespace):
        if name == "dynamic_obstacle_tracker" and self.forbidden:
            return [(self.forbidden, ["std_msgs/msg/String"])]
        return [("/onboard/lidar", ["sensor_msgs/msg/PointCloud2"])]

    def get_publishers_info_by_topic(self, topic):
        if topic == "/map":
            return [S(node_name="slam_toolbox")]
        return [S(node_name=self.odom_owner)] if topic == "/odom" else []


def test_runtime_boundary_rejects_privileged_tracking_and_localization():
    graph = Graph()
    assert not audit_onboard_graph(graph)
    graph.forbidden = "/mujoco/pedestrians"
    assert any("privileged" in error for error in audit_onboard_graph(graph))
    graph.forbidden = None
    graph.odom_owner = "g1_sim_sensors"
    assert any("/odom owners" in error for error in audit_onboard_graph(graph))


def test_goal_people_are_invariant_to_unseen_simulator_people():
    person = S(track_id=1, pose=S(pose=S(position=S(x=1., y=2.))),
        velocity=S(twist=S(linear=S(x=.5, y=0.))), body_radius=.4, confidence=.9)
    node = S(config=S(onboard_only=True, map_frame="map", person_radius_m=.42),
        get_clock=lambda: S(now=lambda: S(nanoseconds=10_100_000_000)),
        observed_people=S(header=S(frame_id="map", stamp=S(sec=10, nanosec=0)), persons=[person]),
        latest_people=[])
    before = BenchmarkRosNode.goal_people(node)
    node.latest_people = ["hidden person", "different destinations", "different velocities"]
    assert BenchmarkRosNode.goal_people(node) == before
    node.observed_people.persons = []
    assert BenchmarkRosNode.goal_people(node) == []


def test_goal_requires_observed_free_footprint():
    grid = S(info=S(resolution=.1, width=40, height=40,
        origin=S(position=S(x=0., y=0.), orientation=S(x=0., y=0., z=0., w=1.))), data=[0]*1600)
    assert observed_free(grid, 2., 2., .35)
    grid.data[22*40+20] = -1
    assert not observed_free(grid, 2., 2., .35)
    grid.data[22*40+20] = 100
    assert not observed_free(grid, 2., 2., .35)
    assert not observed_free(None, 2., 2., .35)


class Future:
    def __init__(self):
        self.callbacks = []
        self.value = None

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def complete(self, value):
        self.value = value
        for callback in self.callbacks:
            callback(self)


def test_aborted_retry_late_acceptance_is_cancelled(monkeypatch):
    import dynamic_obstacle_benchmarks.benchmark_runner as module
    monkeypatch.setattr(module, "NavigateToPose", S(Goal=lambda: S()))
    future, cancelled = Future(), []
    node = S(config=S(onboard_only=True), mission_active=True, nav_result_status=6,
        nav_retry_future=None, last_nav_retry_s=0., current_goal_index=2,
        nav_client=S(server_is_ready=lambda: True, send_goal_async=lambda request: future),
        _pose_stamped=lambda pose: pose)
    BenchmarkRosNode.retry_aborted_goal(node, S(index=2, pose="same observed goal"), 10.)
    assert node.nav_retry_future is future
    node.mission_active = False
    node.current_goal_index = None
    future.complete(S(accepted=True, cancel_goal_async=lambda: cancelled.append(True)))
    assert cancelled == [True]


def test_retry_does_not_restart_a_user_cancelled_goal():
    node = S(config=S(onboard_only=True), mission_active=True, nav_result_status=5)
    # No action client is needed: a cancelled action is not an aborted one.
    BenchmarkRosNode.retry_aborted_goal(node, S(index=1), 10.)
