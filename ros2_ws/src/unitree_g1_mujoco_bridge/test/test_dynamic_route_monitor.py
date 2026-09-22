from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from std_msgs.msg import Bool
from unitree_social_navigation_msgs.msg import TrackedPerson, TrackedPersons

from unitree_g1_mujoco_bridge.dynamic_route_monitor import DynamicRouteMonitor


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def monitor(monkeypatch):
    import unitree_g1_mujoco_bridge.dynamic_route_monitor as module
    now = [100.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    node = DynamicRouteMonitor()
    requests = []
    node.request_pub = SimpleNamespace(publish=requests.append)
    node._transform = lambda *args: (0., 0., 1., 0.)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = 3.
    node._goal(goal)
    node._mission(Bool(data=True))
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp.sec = 10
    path.poses = [PoseStamped(), goal]
    node._path(path)
    yield node, now, requests
    node.destroy_node()


def crossing(node):
    people = TrackedPersons()
    people.header.frame_id = "odom"
    people.header.stamp = node.get_clock().now().to_msg()
    person = TrackedPerson()
    person.track_id = 1
    person.confidence = 1.
    person.track_state = person.TRACK_STATE_CONFIRMED
    person.body_radius = .4
    person.pose.pose.position.x = 1.5
    person.pose.pose.position.y = -1.5
    person.velocity.twist.linear.y = .8
    people.persons = [person]
    return people


def test_predicted_crossing_requests_replan_and_cancellation_stops_requests(monitor):
    node, now, requests = monitor
    node._people(crossing(node))
    node._tick()
    assert len(requests) == 1
    now[0] += .01
    node._tick()
    assert len(requests) == 1
    node._mission(Bool(data=False))
    now[0] += 1.
    node._tick()
    assert len(requests) == 1


def test_stale_people_do_not_announce_a_cleared_corridor(monitor):
    node, now, requests = monitor
    people = crossing(node)
    node._people(people)
    node._tick()
    node._path(node.path)
    people.header.stamp = (node.get_clock().now() - Duration(seconds=2.)).to_msg()
    now[0] += .3
    node._tick()
    assert len(requests) == 1
    people.header.stamp = node.get_clock().now().to_msg()
    people.persons = []
    node._tick()
    assert len(requests) == 2


def test_unrelated_and_late_paths_cannot_acknowledge_a_request(monitor):
    node, now, requests = monitor
    original = node.path
    other = Path()
    other.header.frame_id = "map"
    other.header.stamp.sec = 11
    point = PoseStamped()
    point.pose.position.x = 8.
    other.poses = [point]
    node._path(other)
    assert node.path is original
    other.header.stamp.sec = 9
    other.poses = [node.goal]
    node._path(other)
    assert node.path is original
