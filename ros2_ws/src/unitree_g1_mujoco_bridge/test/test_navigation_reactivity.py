"""Behavioral regressions for perception and executable social planning."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.duration import Duration
from tf2_ros import TransformException

from unitree_g1_mujoco_bridge.dynamic_obstacle_tracker import (
    DynamicObstacleTracker, PersonDetection, Track, _motion_covariance, predict_track_motion,
)
from unitree_g1_mujoco_bridge.human_aware_motion_planner import (
    Candidate, HumanAwareMotionPlanner, HumanTrack,
)
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope
from unitree_g1_mujoco_bridge.human_collision_avoidance import HumanCollisionAvoidance
from unitree_g1_mujoco_bridge.livox_to_scan import LivoxToScan
from unitree_g1_mujoco_bridge.g1_sim_sensors import G1SimSensors
from unitree_g1_mujoco_bridge.static_distance_field import StaticDistanceField, observed_free_mask
from unitree_g1_mujoco_bridge.scan_dynamic_filter import ScanDynamicFilter
from unitree_social_navigation_msgs.msg import TrackedPerson


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def tracker():
    node = DynamicObstacleTracker()
    node.use_mujoco_people_detections = False
    node.use_ml_person_detections = False
    node.min_confirm_age = 0.15
    node.confirm_confidence_threshold = 0.5
    node._lookup_robot_xy = lambda *args: (0.0, 0.0)
    yield node
    node.destroy_node()


def observe(node, stamp, x, y=0.0):
    cluster = node._cluster_from_detection(PersonDetection(x, y, 0.8, 0.9, 0.35, stamp))
    assignments = node._global_track_assignments([cluster], stamp)
    return node._match_track(cluster, stamp, set(), assignments.get(0))


@pytest.mark.parametrize("rate", [5, 10, 20, 30])
@pytest.mark.parametrize("speed", [0.5, 1.0, 1.8])
def test_walking_initializes_velocity_without_identity_reset(tracker, rate, speed):
    identities = []
    for index in range(rate * 2):
        t = index / rate
        track_id, track = observe(tracker, 10.0 + t, 3.0 + speed * t)
        identities.append(track_id)
    assert len(set(identities)) == 1
    assert track.confirmed
    assert track.vx == pytest.approx(speed, abs=0.15)
    assert track.x == pytest.approx(3.0 + speed * t, abs=0.12)


def test_noisy_centroids_do_not_trigger_acceleration_rejection(tracker):
    identities = []
    for index in range(40):
        t = index * 0.05
        track_id, track = observe(tracker, 10.0 + t, 3.0 + t + 0.025 * math.sin(index * 2.0))
        identities.append(track_id)
    assert len(set(identities)) == 1
    assert track.vx == pytest.approx(1.0, abs=0.2)


def test_track_handles_turn_and_stop_without_lifetime_direction_prediction(tracker):
    ids = []
    for index in range(90):
        t = index * 0.1
        x = 3.0 + min(t, 2.0)
        y = max(0.0, min(t - 2.0, 2.0))
        track_id, track = observe(tracker, 10.0 + t, x, y)
        ids.append(track_id)
    assert len(set(ids)) == 1
    assert track.speed < 0.1
    assert tracker._prediction_velocity(track) == (0.0, 0.0)


def test_outlier_does_not_modify_existing_measurement_epoch(tracker):
    for index in range(10):
        _, track = observe(tracker, 10.0 + index * 0.1, 3.0 + index * 0.1)
    stamp, x = track.stamp, track.x
    outlier_id, _ = observe(tracker, stamp + 0.1, x + 5.0)
    assert track.stamp == stamp
    assert track.x == x
    assert tracker.tracks[outlier_id] is not track


def test_person_orb_refresh_uses_latest_tf_without_refreshing_control_tracks(tracker):
    from visualization_msgs.msg import Marker, MarkerArray
    published = []
    tracker.pub = SimpleNamespace(get_subscription_count=lambda: 1, publish=published.append)
    tracker.marker_display_hold_time = .9
    stamp = tracker.get_clock().now().nanoseconds * 1e-9
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = tracker.get_clock().now().to_msg()
    marker.ns, marker.id, marker.action = "tracked_people", 7, Marker.ADD
    marker.color.a = 1.
    tracker.tracks[7] = Track(3., 0., stamp, 3., 0.)
    message = MarkerArray(markers=[marker])
    stable = tracker._stable_marker_array(message, stamp)
    assert stable.markers[0].header.stamp.sec == 0
    tracker._refresh_person_markers()
    assert published[-1].markers[0].action == Marker.ADD
    assert tracker.tracks[7].stamp == stamp
    tracker.display_markers[("tracked_people", 7)] = (marker, stamp-1.)
    tracker._refresh_person_markers()
    assert published[-1].markers[0].action == Marker.DELETE
    assert tracker.tracks[7].stamp == stamp


def test_partial_body_keeps_confirmed_identity_prediction_and_footprint(tracker):
    for index in range(20):
        track_id, track = observe(tracker, 10.0 + index * .1, 3.0 + index * .05)
    assert track.confirmed
    radius, height, score = track.radius, track.height, track.person_score
    for index in range(20):
        stamp = 12.0 + index * .1
        cluster = tracker._cluster_from_detection(PersonDetection(4.0 + index * .05, 0., .4, .9, .15, stamp))
        cluster.height = .25
        assert tracker._partial_confirmed_observation(cluster, stamp)
        cluster.partial_observation = True
        identity, updated = tracker._match_track(cluster, stamp, set(), track_id)
        assert identity == track_id and updated.confirmed
        assert updated.person_score >= score
        assert updated.radius == radius and updated.height == height
        assert tracker._should_publish_prediction(updated, False, 0.)
    cluster.x += 5.
    assert not tracker._partial_confirmed_observation(cluster, stamp + .1)


def test_track_crop_requires_returns_and_cannot_refresh_unseen_people(tracker):
    for index in range(20):
        track_id, track = observe(tracker, 10.0 + index * .1, 3.0 + index * .05)
    assert track.confirmed
    points = [(4.0 + dx, dy, z) for dx in (-.15, .0, .15)
              for dy in (-.15, .15) for z in (.3, .6, .9)]
    # A distant wall contributes no points to the track-local measured crop.
    points += [(6.0, float(y), .8) for y in range(-5, 6)]
    crops = tracker._track_conditioned_crops(points, [], 12.0)
    assert len(crops) == 1 and crops[0].partial_observation
    assert crops[0].x == pytest.approx(4.0)
    assert tracker._track_conditioned_crops([], [], 12.0) == []
    assert tracker._track_conditioned_crops(points, [], 15.0) == []


def test_budget_rechecks_stop_and_previous_and_does_not_starve_directions(planner, monkeypatch):
    import unitree_g1_mujoco_bridge.human_aware_motion_planner as module
    clock = [0.0]
    monkeypatch.setattr(module.time, "perf_counter", lambda: clock[0])
    planner.planning_budget_ms = 30.0
    checked = []

    def score(candidate, *args):
        clock[0] += .007
        checked.append(candidate.name)
        return SimpleNamespace(candidate=candidate)

    planner._score_candidate = score
    candidates = [Candidate(name, 0., 0., 0.) for name in
                  ("yield_stop", "previous_command", "raw_reference", *(f"direction_{i}" for i in range(8)))]
    covered = set()
    for _ in range(4):
        planner.planning_started = clock[0]
        results = planner._evaluate_candidates(candidates, None, [], [], [], {}, 0.)
        names = [result.candidate.name for result in results]
        assert names[:3] == ["yield_stop", "previous_command", "raw_reference"]
        assert len(names) == 5 and planner.search_budget_exhausted
        assert names == checked[-5:]
        covered.update(names)
    assert covered == {candidate.name for candidate in candidates}


def test_coasting_uncertainty_is_elapsed_time_consistent():
    track = Track(0.0, 0.0, 10.0, 0.0, 0.0)
    whole = _motion_covariance(track.kf_p, 1.0)
    split = _motion_covariance(_motion_covariance(track.kf_p, 0.4), 0.6)
    for a, b in zip(whole, split):
        assert a == pytest.approx(b)
    assert whole[0][0] > track.kf_p[0][0]


@pytest.fixture
def planner():
    node = HumanAwareMotionPlanner()
    node.motion_envelope = MotionEnvelope(max_y=0.0, accel_x=0.35)
    node.max_lateral_near_human = 0.0
    node.measured_velocity = (0.35, 0.0, 0.0)
    node._static_obstacle_cost = lambda trajectory: (0.0, False)
    node.latest_cmd = Twist()
    node.latest_cmd.linear.x = 0.35
    node.latest_cmd_time = node.get_clock().now()
    node.odom_stamp = node.latest_cmd_time.nanoseconds * 1e-9
    node._robot_pose = lambda: (0.0, 0.0, 0.0)
    node.published = []
    node._publish = lambda cmd, state, results, selected: node.published.append((cmd, state, results, selected))
    yield node
    node.destroy_node()


def test_tentative_typed_track_reaches_planner(tracker, planner):
    track = Track(2.0, 0.0, 10.0, 2.0, 0.0)
    track.state = "TENTATIVE"
    message = tracker._tracked_persons_message([(1, track)], "world", 10.0)
    planner._tracked_persons_callback(message)
    assert message.persons[0].track_state == TrackedPerson.TRACK_STATE_TENTATIVE
    assert planner.tracks[1].state == "TENTATIVE"


def test_qualified_tentative_risk_is_published_before_confirmation(tracker):
    cluster = tracker._cluster_from_detection(PersonDetection(2.0, 0.0, 0.8, 0.9, 0.35, 10.0))
    tracker._clusters = lambda *args, **kwargs: [cluster]
    tracker._person_focused_clusters = lambda *args: []
    tracker._is_candidate_person_risk = lambda *args: True
    tracker.min_confirm_age = 0.5
    published = []
    tracker.tracked_persons_pub = SimpleNamespace(publish=published.append)
    tracker._publish_clusters([], 10.0, "world")
    assert published[-1].persons
    assert published[-1].persons[0].track_state == TrackedPerson.TRACK_STATE_TENTATIVE


def test_lost_track_observation_age_survives_message(tracker, planner):
    track = Track(2.0, 0.0, 10.0, 2.0, 0.0)
    track.state = "OCCLUDED"
    track.observation_age = 2.5
    message = tracker._tracked_persons_message([(1, track)], "world", 10.0)
    planner._tracked_persons_callback(message)
    assert planner.tracks[1].observation_age == pytest.approx(2.5)


def test_future_conflict_searches_alternatives_before_cpa_warning(planner):
    stamp = planner.get_clock().now().nanoseconds * 1e-9
    # Crossing at t=4s, beyond the 3s state warning; the five-second rollout
    # should find a slowing/yield alternative rather than emergency fallback.
    person = HumanTrack(1, "CONFIRMED", 1.4, -4.0, 0.0, 1.0, 0.9, 0.9, 0, 0.01, 0.01, stamp)
    planner.tracks = {1: person}
    planner.tracks_epoch = stamp
    planner._timer_callback()
    command, state, results, selected = planner.published[-1]
    assert len(results) > 1
    assert selected is not None
    assert state != "EMERGENCY_STOP"
    assert command.linear.x < 0.35


def test_empty_fresh_scene_preserves_command_but_stale_scene_stops(planner):
    planner.tracks_epoch = planner.get_clock().now().nanoseconds * 1e-9
    planner._timer_callback()
    assert planner.published[-1][0].linear.x == pytest.approx(0.35)
    planner.tracks_epoch -= 2.0
    planner._timer_callback()
    assert planner.published[-1][0].linear.x == 0.0
    assert planner.published[-1][1] == "PERCEPTION_STALE_STOP"


def test_candidate_rollout_matches_arc_transform_and_lateral_clamp(planner):
    planner.motion_envelope = MotionEnvelope(max_y=0.0, arc_forward=0.20, arc_yaw=0.12)
    candidates = planner._executable_candidates([Candidate("turn", 0.0, 0.12, 0.11)])
    assert (candidates[0].vx, candidates[0].vy, candidates[0].wz) == pytest.approx((0.20, 0.0, 0.12))
    assert planner._simulate((0.0, 0.0, 0.0), candidates[0])[-1][0] > 0.5


def test_nav2_turn_request_is_not_discarded_by_forward_arc_floor(planner):
    planner.motion_envelope = MotionEnvelope(max_y=0.0, arc_forward=0.06)
    planner.latest_cmd.linear.x = 0.0
    planner.latest_cmd.angular.z = 0.3
    candidates = planner._executable_candidates([Candidate("raw_limited", 0.0, 0.0, 0.3)])
    assert len(candidates) == 1
    assert candidates[0].vx == pytest.approx(0.06)
    stopped = planner._executable_candidates([Candidate("yield_stop", 0.0, 0.0, 0.0)])
    assert stopped[0].vx == 0.0


def test_zero_command_has_physical_braking_distance():
    envelope = MotionEnvelope(accel_x=0.35)
    trajectory = envelope.rollout((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.35, 0.0, 0.0), 2.0, 0.01, 0.15)
    assert trajectory[-1][0] == pytest.approx(0.35 * 0.15 + 0.35 ** 2 / (2 * 0.35), abs=0.005)


def test_arc_floor_cannot_exceed_policy_limits():
    envelope = MotionEnvelope(max_x=0.1, max_yaw=0.1, arc_forward=0.2, arc_yaw=0.2)
    assert envelope.normalize((0.0, 0.0, 0.1)) == pytest.approx((0.1, 0.0, 0.1))


def test_prediction_anticipates_acceleration_and_does_not_reverse_after_braking():
    track = Track(0.0, 0.0, 1.0, 0.0, 0.0, vx=0.2, ax=1.0)
    accelerated = predict_track_motion(track, 2.0, 0.1, 2.0)
    assert accelerated[-1][1] > 0.2 * 2.0 + 0.5
    track.vx, track.ax = 0.4, -2.0
    braking = predict_track_motion(track, 2.0, 0.1, 2.0)
    assert all(p[3] >= 0.0 for p in braking)
    assert braking[-1][3] == 0.0
    assert braking[-1][1] < 0.1
    track.vx = 0.0
    assert predict_track_motion(track, 2.0, 0.1, 2.0)[-1][1] == 0.0


def test_rollout_includes_software_command_slew_before_physical_reversal():
    envelope = MotionEnvelope(max_x=0.65, accel_x=0.6)
    pose = (0.0, 0.0, 0.0)
    command, velocity = (-0.4, 0.0, 0.0), (0.2, 0.0, 0.0)
    simple = envelope.rollout(pose, command, velocity, 0.5, 0.05, 0.1)
    cascaded = envelope.rollout(pose, command, velocity, 0.5, 0.05, 0.1, filtered_command=(0.6, 0.0, 0.0))
    assert cascaded[-1][0] > simple[-1][0] + 0.05


def test_swept_collision_between_prediction_samples(planner):
    planner._simulate = lambda pose, candidate: [(0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)]
    person = HumanTrack(1, "CONFIRMED", 0.0, -1.0, 0.0, 2.0, 1.0, 1.0, 0, 0.01, 0.01, 0.0, 0.42)
    result = planner._score_candidate(Candidate("hold", 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), [person], [], [], {}, 0.0)
    assert result.reason == "hard_human_collision"
    assert result.min_human_distance == pytest.approx(0.0)


def test_ground_referenced_projection_rejects_floor_but_keeps_low_obstacle(tracker):
    node = LivoxToScan()
    try:
        tf = TransformStamped()
        tf.transform.rotation.w = 1.0
        tf.transform.translation.z = 1.0
        node.tf_buffer.lookup_transform = lambda *args: tf
        published = []
        node.pub = SimpleNamespace(publish=published.append)
        cloud = tracker._prediction_cloud([(2.0, 0.0, -1.0), (3.0, 0.0, -0.64)], "lidar", 10.0)
        node._cloud_callback(cloud)
        scan = published[0]
        assert scan.header.frame_id == "base_footprint"
        assert min(scan.ranges) == pytest.approx(3.0)
        assert scan.header.stamp.sec == 10
        assert node._fill_circular_gaps([2.0, math.inf, math.inf, 8.0])[1:3] == [math.inf, math.inf]
        assert all(math.isfinite(v) for v in node._fill_circular_gaps([2.0, math.inf, 2.1]))
    finally:
        node.destroy_node()


def test_static_distance_field_never_overstates_clearance():
    rng = np.random.default_rng(23)
    obstacles = rng.uniform(-2.0, 2.0, (30, 2))
    queries = rng.uniform(-2.5, 2.5, (100, 2))
    field = StaticDistanceField(obstacles, (0.0, 0.0), 3.0, 1.5)
    exact = np.min(np.linalg.norm(queries[:, None, :] - obstacles[None, :, :], axis=2), axis=1)
    assert np.all(field.query(queries) <= exact + 1e-9)
    assert field.query([[10.0, 10.0]])[0] == 0.0


def test_static_memory_requires_measured_free_space_to_clear():
    points = np.asarray([[2.0, 0.0]])
    assert observed_free_mask(points, (0.0, 0.0, 0.0), [3.0, 3.0, 3.0], -0.02, 0.02)[0]
    assert not observed_free_mask(points, (0.0, 0.0, 0.0), [3.0, math.inf, 3.0], -0.02, 0.02)[0]
    assert not observed_free_mask(points, (0.0, 0.0, 0.0), [3.0, 2.0, 3.0], -0.02, 0.02)[0]


def test_typed_person_scan_mask_preserves_background_wall():
    node = object.__new__(ScanDynamicFilter)
    node.min_filter_range = 0.2
    node.filter_radius = 0.75
    node.range_padding = 0.45
    scan = LaserScan()
    scan.range_min, scan.range_max = 0.1, 10.0
    scan.angle_min, scan.angle_max, scan.angle_increment = -0.02, 0.02, 0.02
    scan.ranges = [2.7, 2.0, 2.7]
    node._remove_dynamic_return(scan, 2.0, 0.0, 0.52)
    assert math.isinf(scan.ranges[1])
    assert scan.ranges[0] == pytest.approx(2.7)
    assert scan.ranges[2] == pytest.approx(2.7)


def test_known_scene_map_preserves_fountain_footprint_and_excludes_floor(planner):
    sensor = object.__new__(G1SimSensors)
    sensor.get_clock = planner.get_clock
    sensor.scene_boxes = [((-10.0, -10.0, -0.1), (10.0, 10.0, 0.0)),
                          ((3.15, 1.75, 0.0), (5.05, 3.65, 0.36))]
    grid = sensor._static_scene_map()
    def cell(x, y):
        col = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y - grid.info.origin.position.y) / grid.info.resolution)
        return grid.data[row * grid.info.width + col]
    assert cell(0.0, 0.0) == 0
    assert cell(3.2, 2.7) == 100
    planner._scene_map_callback(grid)
    assert planner.static_scene_points is not None


def test_active_path_reference_is_frame_correct_and_ignores_sampling_paths(planner):
    path = Path()
    path.header.frame_id = "world"
    for x in (0.0, 1.0, 2.0):
        pose = PoseStamped()
        pose.pose.position.x = x
        path.poses.append(pose)
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.pose.position.x = 2.0
    planner._goal_callback(goal)
    planner._path_callback(path)
    reference = planner._path_reference((0.0, 0.0, math.pi / 2.0))
    assert reference.linear.x == pytest.approx(0.0, abs=1e-6)
    assert reference.linear.y == pytest.approx(-0.35)
    goal.pose.position.x = 4.0
    planner._goal_callback(goal)
    assert planner._path_reference((0.0, 0.0, 0.0)) is None


def test_valid_static_route_survives_unrelated_planning_and_waits(planner):
    planner.use_static_scene_map = True
    path = Path()
    path.header.frame_id = "world"
    for x in (0.0, 2.0):
        point = PoseStamped()
        point.pose.position.x = x
        path.poses.append(point)
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.pose.position.x = 2.0
    planner._goal_callback(goal)
    planner._path_callback(path)
    unrelated = Path()
    unrelated.header.frame_id = "world"
    point = PoseStamped()
    point.pose.position.x = 8.0
    unrelated.poses = [point]
    planner._path_callback(unrelated)
    planner.reference_path_time -= Duration(seconds=30.0)
    assert planner.reference_path is path
    assert planner._path_reference((0.0, 0.0, 0.0)).linear.x > 0.0


def test_forward_preference_turns_for_travel_but_finishes_nearby_goals(planner):
    planner.prefer_forward_travel = True
    for distance in (2., .6):
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.pose.position.x = distance
        path = Path()
        path.header.frame_id = "world"
        path.poses = [goal]
        planner._goal_callback(goal)
        planner._path_callback(path)
        reference = planner._path_reference((0., 0., math.pi/2))
        if distance > 1.:
            assert reference.linear.y == 0.
            assert reference.angular.z < 0.
        else:
            assert reference.linear.y < -.25
            assert reference.angular.z == 0.


def test_dynamic_path_handoff_rejects_late_results_to_the_same_goal(planner):
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.pose.position.x = 2.
    planner._goal_callback(goal)
    current = Path()
    current.header.frame_id = "world"
    current.header.stamp.sec = 20
    current.poses = [goal]
    planner._path_callback(current)
    late = Path()
    late.header.frame_id = "world"
    late.header.stamp.sec = 19
    late.poses = [goal]
    planner._path_callback(late)
    assert planner.reference_path is current


def test_mission_heartbeat_owns_reference_and_cancellation_stops_it(planner):
    planner.nav2_path_reference = True
    planner.use_static_scene_map = True
    path = Path()
    path.header.frame_id = "world"
    point = PoseStamped()
    point.pose.position.x = 2.0
    path.poses = [point]
    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.pose.position.x = 2.0
    planner._goal_callback(goal)
    planner._path_callback(path)
    now = planner.get_clock().now()
    planner.tracks_epoch = planner.odom_stamp = now.nanoseconds * 1e-9
    planner.latest_cmd_time = now - Duration(seconds=2.0)
    planner._mission_callback(Bool(data=True))
    planner._timer_callback()
    assert planner.published[-1][0].linear.x > 0.0
    planner._mission_callback(Bool(data=False))
    planner._timer_callback()
    assert planner.published[-1][0].linear.x == 0.0
    planner._mission_callback(Bool(data=True))
    planner.mission_state_time -= Duration(seconds=2.0)
    planner._timer_callback()
    assert planner.published[-1][1] == "MISSION_STALE_STOP"


def test_missing_scan_transform_is_attempted_only_once_per_plan(planner):
    scan = LaserScan()
    scan.header.frame_id = "test_sensor"
    scan.header.stamp = planner.get_clock().now().to_msg()
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.angle_increment = 0.1
    scan.ranges = [2.0]
    planner._scan_callback(scan)
    calls = []
    def unavailable(*args):
        calls.append(args)
        raise TransformException("not ready")
    planner.tf_buffer.lookup_transform = unavailable
    trajectory = [(0.0, 0.0, 0.0, 0.0), (0.2, 0.0, 0.0, 1.0)]
    for _ in range(3):
        assert HumanAwareMotionPlanner._static_obstacle_cost(planner, trajectory)[1]
    assert len(calls) == 1


@pytest.mark.parametrize("person_start,person_velocity", [((5.0, 0.0), (-0.6, 0.0)), ((1.5, -3.0), (0.0, 0.8))])
def test_closed_loop_avoids_nonreactive_person(planner, person_start, person_velocity):
    pose = (0.0, 0.0, 0.0)
    velocity = (0.0, 0.0, 0.0)
    pending = [(0.0, 0.0, 0.0)] * 2
    minimum = math.inf
    decisions = []
    planner.motion_envelope = MotionEnvelope(max_y=0.0, max_yaw=0.35, accel_x=0.35, arc_forward=0.06, arc_yaw=0.12)
    for step in range(100):
        t = step * 0.1
        person_x = person_start[0] + person_velocity[0] * t
        person_y = person_start[1] + person_velocity[1] * t
        minimum = min(minimum, math.hypot(pose[0] - person_x, pose[1] - person_y))
        now = planner.get_clock().now()
        stamp = now.nanoseconds * 1e-9
        planner._robot_pose = lambda: pose
        heading = math.atan2(-pose[1], 6.0 - pose[0])
        error = math.atan2(math.sin(heading - pose[2]), math.cos(heading - pose[2]))
        planner.latest_cmd.linear.x = 0.35
        planner.latest_cmd.angular.z = max(-0.35, min(0.35, error))
        planner.latest_cmd_time = now
        planner.tracks_epoch = planner.odom_stamp = stamp
        planner.measured_velocity = velocity
        planner.tracks = {1: HumanTrack(1, "CONFIRMED", person_x, person_y, *person_velocity, 1.0, 1.0, 0, 0.01, 0.01, stamp, 0.42)}
        planner._timer_callback()
        cmd = planner.published[-1][0]
        if step % 10 == 0:
            decisions.append((t, planner.published[-1][1], (cmd.linear.x, cmd.linear.y, cmd.angular.z)))
        pending.append((cmd.linear.x, cmd.linear.y, cmd.angular.z))
        target = planner.motion_envelope.normalize(pending.pop(0))
        limits = (0.35, 0.16, 0.35)
        velocity = tuple(v + max(-a * 0.1, min(a * 0.1, u - v)) for v, u, a in zip(velocity, target, limits))
        x, y, yaw = pose
        pose = (x + math.cos(yaw) * velocity[0] * 0.1, y + math.sin(yaw) * velocity[0] * 0.1, yaw + velocity[2] * 0.1)
    assert minimum > 0.80, decisions
    assert pose[0] > 0.5


def test_supervisor_rechecks_changed_commands_and_stale_predictions():
    node = HumanCollisionAvoidance()
    try:
        now = node.get_clock().now()
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        node._odom_callback(odom)
        node._robot_pose = lambda: (0.0, 0.0, 0.0, 0.0)
        node.prediction_epoch = now.nanoseconds * 1e-9
        node.latest_predictions = []
        cmd = Twist()
        cmd.linear.x = 0.3
        safe, reason = node._validate_final_command(cmd, (0.0, 0.0, 0.0), now)
        assert safe.linear.x == pytest.approx(0.3)
        assert reason is None
        node._cmd_callback(cmd)
        node._timer_callback()
        node.latest_predictions = [[(0.0, 0.9, 0.0, 0.0, 0.0, 1.0), (2.0, 0.9, 0.0, 0.0, 0.0, 1.0)]]
        safe, reason = node._validate_final_command(cmd, (0.0, 0.0, 0.0), now)
        assert safe.linear.x == 0.0
        assert reason == "temporal_override"
        node.prediction_epoch -= 2.0
        assert node._validate_final_command(cmd, (0.0, 0.0, 0.0), now)[1] == "stale_predictions_stop"
    finally:
        node.destroy_node()
