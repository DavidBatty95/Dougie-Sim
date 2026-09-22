import math

import pytest
from geometry_msgs.msg import Twist

from unitree_g1_mujoco_bridge.dynamic_obstacle_tracker import (
    DynamicObstacleTracker,
    PersonDetection,
    Track,
    _constant_velocity_covariance,
    _position_covariance_at,
    _split_seconds,
    _track_should_expire,
    _typed_track_state,
)
from unitree_g1_mujoco_bridge.human_aware_motion_planner import (
    HumanAwareMotionPlanner,
    HumanTrack,
    PredictionSample,
    align_track_to_time,
    incoming_collision_imminent,
    interpolate_prediction,
    nearest_clear_patch,
)
from unitree_g1_mujoco_bridge.human_collision_avoidance import (
    time_aligned_prediction_risk,
)
from unitree_g1_mujoco_bridge.lidar_person_detector import LidarPersonDetector
from unitree_social_navigation_msgs.msg import TrackedPerson


def test_split_seconds_normalizes_rounding_at_second_boundary():
    assert _split_seconds(12.9999999996) == (13, 0)
    assert _split_seconds(3.25) == (3, 250_000_000)


def test_constant_velocity_position_covariance_propagation():
    covariance = [
        [1.0, 0.1, 0.2, 0.0],
        [0.1, 2.0, 0.0, 0.3],
        [0.2, 0.0, 0.5, 0.0],
        [0.0, 0.3, 0.0, 0.75],
    ]
    propagated = _constant_velocity_covariance(covariance, 2.0)
    xx, xy, yx, yy = _position_covariance_at(covariance, 2.0)
    assert propagated[0][0] == pytest.approx(3.8)
    assert propagated[1][1] == pytest.approx(6.2)
    assert (xx, xy, yx, yy) == pytest.approx(
        (propagated[0][0], propagated[0][1], propagated[1][0], propagated[1][1])
    )


def test_tracker_state_maps_occlusion_to_lost():
    assert _typed_track_state("CONFIRMED") == TrackedPerson.TRACK_STATE_CONFIRMED
    assert _typed_track_state("OCCLUDED") == TrackedPerson.TRACK_STATE_LOST
    assert _typed_track_state("TENTATIVE") == TrackedPerson.TRACK_STATE_TENTATIVE


def test_confirmed_track_ignores_short_stale_timeout_and_persists_until_lost_timeout():
    track = Track(1.0, 2.0, 4.25, 1.0, 2.0)
    track.confirmed = True
    track.confidence = 0.7
    assert not _track_should_expire(track, 1.5, 0.08, 0.25, 6.0)
    assert _track_should_expire(track, 6.01, 0.08, 0.25, 6.0)

    track.confirmed = False
    assert _track_should_expire(track, 0.26, 0.08, 0.25, 6.0)


def test_occluded_confirmed_track_keeps_prediction_until_drop_threshold():
    tracker = object.__new__(DynamicObstacleTracker)
    tracker.prediction_min_confidence = 0.50
    tracker.prediction_min_person_score = 0.52
    tracker.min_person_observations = 2
    tracker.prediction_min_dynamic_observations = 1
    tracker.person_detection_confirm_confidence = 0.55
    tracker.min_static_person_observations = 4
    tracker.max_prediction_coast_time = 6.0
    tracker.max_person_speed = 2.5

    track = Track(1.0, 2.0, 4.25, 1.0, 2.0)
    track.confirmed = True
    track.confidence = 0.20
    track.person_score = 0.80
    track.person_observations = 3
    track.dynamic_observations = 2
    track.speed = 0.5

    assert not tracker._should_publish_prediction(track, coasting=False, coast_time=0.0)
    assert tracker._should_publish_prediction(track, coasting=True, coast_time=5.0)
    assert not tracker._should_publish_prediction(track, coasting=True, coast_time=6.1)


def test_tracked_person_message_preserves_state_covariance_confidence_and_radius():
    class Converter:
        _set_header = staticmethod(DynamicObstacleTracker._set_header)

    track = Track(1.0, 2.0, 4.25, 1.0, 2.0)
    track.vx = 0.4
    track.vy = -0.2
    track.confidence = 0.73
    track.radius = 0.46
    track.state = "OCCLUDED"
    track.kf_p = [
        [0.11, 0.02, 0.0, 0.0],
        [0.02, 0.22, 0.0, 0.0],
        [0.0, 0.0, 0.33, 0.04],
        [0.0, 0.0, 0.04, 0.44],
    ]
    message = DynamicObstacleTracker._tracked_persons_message(
        Converter(), [(9, track)], "world", 4.25
    )
    person = message.persons[0]
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == (4, 250_000_000)
    assert person.track_state == TrackedPerson.TRACK_STATE_LOST
    assert person.pose.covariance[0] == pytest.approx(0.11)
    assert person.pose.covariance[7] == pytest.approx(0.22)
    assert person.velocity.covariance[0] == pytest.approx(0.33)
    assert person.velocity.covariance[7] == pytest.approx(0.44)
    assert person.confidence == pytest.approx(0.73)
    assert person.body_radius == pytest.approx(0.46)


def test_authoritative_mujoco_clusters_preserve_exact_position_and_velocity():
    tracker = object.__new__(DynamicObstacleTracker)
    tracker.cell_size = 0.10
    tracker.mujoco_person_height = 1.70
    detection = PersonDetection(1.2, -0.4, 0.85, 1.0, 0.42, 5.0, -0.3, 0.2)
    tracker._active_mujoco_detections = lambda stamp: [detection]
    noisy = tracker._cluster_from_detection(
        PersonDetection(1.6, -0.1, 0.85, 0.5, 0.30, 5.0)
    )
    clusters = tracker._mujoco_authoritative_clusters([noisy], 5.0)
    assert len(clusters) == 1
    assert (clusters[0].x, clusters[0].y) == pytest.approx((1.2, -0.4))
    assert (clusters[0].measured_vx, clusters[0].measured_vy) == pytest.approx((-0.3, 0.2))


def test_effective_people_source_reports_authoritative_truth_or_fallback():
    tracker = object.__new__(DynamicObstacleTracker)
    tracker.cell_size = 0.10
    tracker.mujoco_person_height = 1.70
    tracker.mujoco_people_authoritative = True
    detection = PersonDetection(1.2, -0.4, 0.85, 1.0, 0.42, 5.0)
    tracker._active_mujoco_detections = lambda stamp: [detection]
    clusters, source = tracker._effective_people_clusters([], 5.0)
    assert len(clusters) == 1
    assert source == "mujoco_truth_oracle"

    tracker._active_mujoco_detections = lambda stamp: []
    fallback = tracker._cluster_from_detection(detection)
    clusters, source = tracker._effective_people_clusters([fallback], 5.0)
    assert clusters == [fallback]
    assert source == "sensor_or_fused"


def test_prediction_interpolation_aligns_position_covariance_and_confidence():
    points = [
        PredictionSample(0.0, 1.0, 2.0, 1.0, 0.0, 0.2, 0.4, 0.9),
        PredictionSample(2.0, 3.0, 4.0, 1.0, 0.0, 1.0, 1.2, 0.5),
    ]
    sample = interpolate_prediction(points, 0.5)
    assert sample is not None
    assert sample.x == pytest.approx(1.5)
    assert sample.y == pytest.approx(2.5)
    assert sample.cov_xx == pytest.approx(0.4)
    assert sample.cov_yy == pytest.approx(0.6)
    assert sample.confidence == pytest.approx(0.8)


def test_prediction_interpolation_does_not_extrapolate_stale_horizon():
    points = [PredictionSample(0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 1.0)]
    assert interpolate_prediction(points, -0.01) is None
    assert interpolate_prediction(points, 0.01) is None


def test_time_aligned_prediction_ignores_different_arrival_times():
    # The geometric paths cross at x=1, but the person arrives three seconds
    # after the robot, so this must not become timeless occupied space.
    trajectory = [[
        (0.0, 1.0, -1.5, 0.0, 0.0, 1.0),
        (4.0, 1.0, 0.0, 0.0, 0.0, 1.0),
    ]]
    assert time_aligned_prediction_risk(
        trajectory,
        message_age=0.0,
        robot_pose=(0.0, 0.0, 0.0),
        command=(1.0, 0.0, 0.0),
        horizon=5.0,
        integration_dt=0.1,
        collision_radius=0.8,
        uncertainty_scale=0.0,
        minimum_confidence=0.5,
    ) is None


def test_time_aligned_prediction_detects_same_time_conflict():
    trajectory = [[
        (0.0, 1.0, -1.0, 0.0, 0.0, 1.0),
        (1.0, 1.0, 0.0, 0.0, 0.0, 1.0),
    ]]
    risk = time_aligned_prediction_risk(
        trajectory,
        message_age=0.0,
        robot_pose=(0.0, 0.0, 0.0),
        command=(1.0, 0.0, 0.0),
        horizon=3.0,
        integration_dt=0.1,
        collision_radius=0.8,
        uncertainty_scale=0.0,
        minimum_confidence=0.5,
    )
    assert risk is not None
    assert risk[:2] == pytest.approx((0.0, 1.0))


def test_time_aligned_prediction_detects_conflict_between_sparse_samples():
    trajectory = [[
        (0.0, 1.0, -1.0, 0.0, 0.0, 1.0),
        (2.0, 1.0, 1.0, 0.0, 0.0, 1.0),
    ]]
    risk = time_aligned_prediction_risk(
        trajectory,
        message_age=0.0,
        robot_pose=(1.0, 0.0, 0.0),
        command=(0.0, 0.0, 0.0),
        horizon=3.0,
        integration_dt=0.1,
        collision_radius=0.4,
        uncertainty_scale=0.0,
        minimum_confidence=0.5,
    )
    assert risk is not None
    assert risk[:2] == pytest.approx((0.0, 1.0))


def test_time_aligned_prediction_rejects_future_epoch():
    assert time_aligned_prediction_risk(
        [[(1.0, 0.0, 0.0, 0.0, 0.0, 1.0)]],
        message_age=-0.2,
        robot_pose=(0.0, 0.0, 0.0),
        command=(0.0, 0.0, 0.0),
        horizon=3.0,
        integration_dt=0.1,
        collision_radius=1.0,
        uncertainty_scale=0.0,
        minimum_confidence=0.5,
    ) is None


def test_time_aligned_prediction_accounts_for_message_age():
    trajectory = [[(2.0, 1.0, 0.0, 0.0, 0.0, 1.0)]]
    risk = time_aligned_prediction_risk(
        trajectory,
        message_age=1.0,
        robot_pose=(0.0, 0.0, 0.0),
        command=(1.0, 0.0, 0.0),
        horizon=3.0,
        integration_dt=0.1,
        collision_radius=0.8,
        uncertainty_scale=0.0,
        minimum_confidence=0.5,
    )
    assert risk is not None
    assert risk[1] == pytest.approx(1.0)


def test_normal_navigation_preserves_the_bounded_upstream_command():
    planner = object.__new__(HumanAwareMotionPlanner)
    planner.max_reverse_velocity = 0.1
    planner.max_forward_near_human = 0.35
    planner.max_lateral_near_human = 0.0
    planner.max_angular_near_human = 0.4
    raw = Twist()
    raw.linear.x = 0.12
    raw.angular.z = -0.08
    candidates = planner._candidate_commands(raw, "NORMAL_NAVIGATION", (0.0, 0.0, 0.0), [])
    assert len(candidates) == 1
    assert candidates[0].name == "raw_limited"
    assert (candidates[0].vx, candidates[0].vy, candidates[0].wz) == pytest.approx(
        (0.12, 0.0, -0.08)
    )


def test_near_human_candidates_do_not_amplify_requested_forward_speed():
    planner = object.__new__(HumanAwareMotionPlanner)
    planner.holonomic_avoidance = False
    planner.max_reverse_velocity = 0.1
    planner.max_forward_near_human = 0.35
    planner.max_lateral_near_human = 0.0
    planner.max_angular_near_human = 0.4
    planner.side_commitment = 0.0
    raw = Twist()
    raw.linear.x = 0.04
    candidates = planner._candidate_commands(raw, "HUMAN_CAUTION", (0.0, 0.0, 0.0), [])
    assert all(candidate.vx <= 0.04 for candidate in candidates)
    assert all(candidate.vy == 0.0 for candidate in candidates)


def test_nearest_clear_patch_avoids_danger_corridor_and_static_obstacles():
    patch = nearest_clear_patch(
        origin=(0.0, 0.0),
        danger_points=[(1.0, 0.0, 0.55)],
        static_points=[(0.0, 0.5)],
        search_radius=2.0,
        spacing=0.5,
        clearance=0.30,
        preferred_heading=0.0,
    )
    assert patch is not None
    assert math.hypot(patch[0] - 1.0, patch[1]) >= 0.85
    assert math.hypot(patch[0], patch[1] - 0.5) >= 0.30


def test_nearest_clear_patch_returns_none_when_ground_is_fully_blocked():
    static_points = [
        (math.cos(index * math.pi / 4.0), math.sin(index * math.pi / 4.0))
        for index in range(8)
    ]
    assert nearest_clear_patch(
        origin=(0.0, 0.0),
        danger_points=[],
        static_points=static_points,
        search_radius=1.0,
        spacing=1.0,
        clearance=0.75,
        preferred_heading=0.0,
    ) is None


def test_collision_reflex_requires_a_closing_person_with_relevant_cpa():
    approaching = HumanTrack(
        1, "CONFIRMED", 2.0, 0.0, -1.0, 0.0, 0.9, 0.8, 0, 0.2, 0.2, 0.0, 0.4
    )
    receding = HumanTrack(
        2, "CONFIRMED", 2.0, 0.0, 1.0, 0.0, 0.9, 0.8, 0, 0.2, 0.2, 0.0, 0.4
    )
    crossing_clear = HumanTrack(
        3, "CONFIRMED", 2.0, 2.0, -1.0, 0.0, 0.9, 0.8, 0, 0.2, 0.2, 0.0, 0.4
    )
    early_warning = HumanTrack(
        4, "CONFIRMED", 5.0, 0.0, -1.0, 0.0, 0.9, 0.8, 0, 0.2, 0.2, 0.0, 0.4
    )
    pose = (0.0, 0.0, 0.0)
    assert incoming_collision_imminent(pose, [approaching], 3.0, 1.1, 0.12)
    assert not incoming_collision_imminent(pose, [receding], 3.0, 1.1, 0.12)
    assert not incoming_collision_imminent(pose, [crossing_clear], 3.0, 1.1, 0.12)
    assert incoming_collision_imminent(pose, [early_warning], 6.0, 1.1, 0.12)


def test_track_is_aligned_from_observation_time_to_planning_time():
    track = HumanTrack(7, "CONFIRMED", 1.0, 2.0, 0.5, -0.25, 0.9, 0.8, 0, 0.2, 0.3, 10.0, 0.42)
    aligned = align_track_to_time(track, 12.0)
    assert (aligned.x, aligned.y) == pytest.approx((2.0, 1.5))
    assert aligned.stamp == 12.0
    assert aligned.body_radius == pytest.approx(0.42)


def test_accumulated_world_points_are_compensated_for_robot_motion():
    world_points = [(2.0, 1.0, 0.8), (8.0, 0.0, 0.8)]
    local = LidarPersonDetector._world_to_local_points(
        world_points, robot_x=1.0, robot_y=1.0, yaw=0.0, grid_size=4.0
    )
    assert local == pytest.approx([(1.0, 0.0, 0.8)])

    turned = LidarPersonDetector._world_to_local_points(
        [(1.0, 2.0, 0.8)], robot_x=1.0, robot_y=1.0, yaw=math.pi / 2.0, grid_size=4.0
    )
    assert turned[0] == pytest.approx((1.0, 0.0, 0.8))
