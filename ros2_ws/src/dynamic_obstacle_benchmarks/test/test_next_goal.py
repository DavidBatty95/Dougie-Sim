import random
from types import SimpleNamespace as S

from nav_msgs.msg import Odometry

from dynamic_obstacle_benchmarks.benchmark_runner import BenchmarkRosNode, sensed_goal_reached
from dynamic_obstacle_benchmarks.config import BenchmarkConfig
from dynamic_obstacle_benchmarks.geometry import Pose2D, Velocity2D
from dynamic_obstacle_benchmarks.next_goal import NextGoalPreparer, grid_path


def setup():
    grid = S(header=S(frame_id="map"), info=S(resolution=.1, width=200, height=200, origin=S(
        position=S(x=-10., y=-10.), orientation=S(x=0., y=0., z=0., w=1.))), data=[0]*40000)
    config = BenchmarkConfig(onboard_only=True, visual_session=True, backend="mujoco", goal_tolerance_m=.5,
                             min_goal_distance_m=1.5, max_goal_distance_m=2.5)
    # Deliberately no Nav2 action client: prefetch must not preempt the active planner.
    return S(config=config, latest_robot_pose=Pose2D(0., 0.), observed_map=grid, goal_people=lambda: [])


def test_half_meter_boundary_is_exact_for_visual_goals():
    goal = Pose2D(0., 0.)
    assert sensed_goal_reached(Pose2D(.5, 0.), goal, .5, False, True)
    assert not sensed_goal_reached(Pose2D(.501, 0.), goal, .5, True, True)


def test_next_goal_is_reachable_without_interrupting_nav2_and_revalidated():
    node = setup()
    prep = NextGoalPreparer(node, random.Random(4))
    prep.prepare(2, Pose2D(1., 0.))
    assert prep.ready is not None
    goal = prep.ready[0]
    x, y = int((goal.pose.x+10)/.1), int((goal.pose.y+10)/.1)
    node.observed_map.data[y*200+x] = 100
    assert prep.take(2) is None


def test_observed_grid_connectivity_rejects_an_unobserved_barrier():
    node = setup()
    for y in range(200):
        node.observed_map.data[y*200+110] = -1
    path, _ = grid_path(node.observed_map, Pose2D(0., 0.), Pose2D(2., 0.), .35)
    assert path is None


def test_cached_goal_can_be_taken_at_arrival_and_is_not_reused():
    node = setup()
    prep = NextGoalPreparer(node, random.Random(4))
    prep.prepare(2, Pose2D(1., 0.))
    assert prep.take(2) is not None
    assert prep.take(2) is None


def test_evaluation_velocity_gap_never_replaces_velocity_with_none():
    node = S(evaluation_pose=Pose2D(0., 0.), evaluation_stamp=10., evaluation_velocity=Velocity2D())
    message = Odometry()
    message.header.frame_id = "world"
    message.header.stamp.sec = 11
    message.pose.pose.orientation.w = 1.
    BenchmarkRosNode._evaluation_callback(node, message)
    assert node.evaluation_velocity is not None
    assert node.evaluation_velocity.vx == 0.


def test_prefetch_uses_a_nearby_observed_nook_instead_of_a_thin_ring():
    node = setup()
    node.observed_map.data = [-1]*40000
    # A connected observed nook admits 1.5–1.8 m goals but cannot accommodate
    # the old, incorrectly narrowed 2.00–2.02 m sampling ring plus footprint.
    for y in range(90, 110):
        for x in range(92, 123):
            node.observed_map.data[y*200+x] = 0
    prep = NextGoalPreparer(node, random.Random(4))
    prep.prepare(1, Pose2D(0., 0.))
    assert prep.ready is not None
    assert prep.take(1) is not None
