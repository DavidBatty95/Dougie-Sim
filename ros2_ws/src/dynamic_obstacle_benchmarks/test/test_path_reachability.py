from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from dynamic_obstacle_benchmarks.benchmark_runner import BenchmarkRosNode
from dynamic_obstacle_benchmarks.config import BenchmarkConfig
from dynamic_obstacle_benchmarks.geometry import Pose2D


@pytest.mark.parametrize("endpoint,status,frame,expected", [
    (1.9, 4, "world", True),
    (1.0, 4, "world", False),  # A nonempty partial path is not reachable.
    (2.0, 6, "world", False),
    (2.0, 4, "map", False),
])
def test_reachability_requires_success_and_a_matching_endpoint(endpoint, status, frame, expected):
    path = Path()
    path.header.frame_id = frame
    pose = PoseStamped()
    pose.pose.position.x = endpoint
    path.poses = [pose]
    response = SimpleNamespace(status=status, result=SimpleNamespace(path=path))
    def done(value):
        return SimpleNamespace(done=lambda: True, result=lambda: value)
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: done(response))
    node = object.__new__(BenchmarkRosNode)
    node.config = BenchmarkConfig(map_frame="world", goal_tolerance_m=0.55)
    node.path_client = SimpleNamespace(wait_for_server=lambda **kwargs: True, send_goal_async=lambda request: done(handle))
    node._pose_stamped = lambda pose: PoseStamped()
    node.get_logger = lambda: SimpleNamespace(info=lambda text: None)
    assert node.goal_has_path(Pose2D(0, 0), Pose2D(2, 0)) is expected


def test_cancellation_revokes_mission_heartbeat_even_without_action_handle():
    node = object.__new__(BenchmarkRosNode)
    node.nav_goal_handle = None
    node.nav_result_future = None
    node.nav_result_status = None
    node.nav_goal_success = False
    node.mission_active = True
    messages = []
    node.mission_state_pub = SimpleNamespace(publish=messages.append)
    node.cancel_nav_goal()
    assert not node.mission_active
    assert len(messages) == 1 and messages[0].data is False
