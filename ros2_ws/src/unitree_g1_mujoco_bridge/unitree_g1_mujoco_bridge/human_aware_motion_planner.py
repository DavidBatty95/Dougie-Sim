import json
import math
import time
import numpy as np
from .observed_route import route_is_free
from .forward_reference import forward_reference
from .avoidance_candidates import crossing_escape_directions
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import ColorRGBA, String, Bool
from unitree_hg.msg import LowState
from unitree_social_navigation_msgs.msg import PredictedTrajectories, TrackedPerson, TrackedPersons
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .bridge_safety import (
    blocked_motion_state,
    no_unrequested_forward,
    relative_motion_metrics,
    scan_is_fresh,
    trajectory_static_cost,
    ttc_is_relevant,
)
from .pointcloud import iter_xyz
from .motion_envelope import load_motion_envelope
from .static_distance_field import StaticDistanceField, observed_free_mask


LATEST_SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


@dataclass
class HumanTrack:
    track_id: int
    state: str
    x: float
    y: float
    vx: float
    vy: float
    confidence: float
    person_score: float
    missed_count: int
    cov_xx: float
    cov_yy: float
    stamp: float
    body_radius: float = 0.35
    observation_age: float = 0.0


def align_track_to_time(track: HumanTrack, query_stamp: float) -> HumanTrack:
    """Advance a track state from its observation stamp to a planning epoch."""
    age = max(0.0, query_stamp - track.stamp)
    return HumanTrack(
        track_id=track.track_id,
        state=track.state,
        x=track.x + track.vx * age,
        y=track.y + track.vy * age,
        vx=track.vx,
        vy=track.vy,
        confidence=track.confidence,
        person_score=track.person_score,
        missed_count=track.missed_count,
        cov_xx=track.cov_xx,
        cov_yy=track.cov_yy,
        stamp=query_stamp,
        body_radius=track.body_radius,
        observation_age=track.observation_age + age,
    )


@dataclass(frozen=True)
class PredictionSample:
    time_offset: float
    x: float
    y: float
    vx: float
    vy: float
    cov_xx: float
    cov_yy: float
    confidence: float


def _ros_time_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def interpolate_prediction(
    points: list[PredictionSample], time_offset: float
) -> PredictionSample | None:
    """Interpolate a prediction at an offset from its message epoch."""
    if not points or time_offset < points[0].time_offset or time_offset > points[-1].time_offset:
        return None
    for right_index, right in enumerate(points):
        if time_offset > right.time_offset:
            continue
        if right_index == 0 or time_offset == right.time_offset:
            return right
        left = points[right_index - 1]
        span = right.time_offset - left.time_offset
        if span <= 0.0:
            return None
        fraction = (time_offset - left.time_offset) / span
        return PredictionSample(
            time_offset=time_offset,
            x=left.x + fraction * (right.x - left.x),
            y=left.y + fraction * (right.y - left.y),
            vx=left.vx + fraction * (right.vx - left.vx),
            vy=left.vy + fraction * (right.vy - left.vy),
            cov_xx=left.cov_xx + fraction * (right.cov_xx - left.cov_xx),
            cov_yy=left.cov_yy + fraction * (right.cov_yy - left.cov_yy),
            confidence=left.confidence + fraction * (right.confidence - left.confidence),
        )
    return None


def nearest_clear_patch(
    origin: tuple[float, float],
    danger_points: list[tuple[float, float, float]],
    static_points: list[tuple[float, float]],
    search_radius: float,
    spacing: float,
    clearance: float,
    preferred_heading: float,
) -> tuple[float, float] | None:
    """Return the closest straight-line reachable patch outside all danger regions."""
    ox, oy = origin
    spacing = max(0.1, spacing)
    ring_count = max(1, math.ceil(search_radius / spacing))

    def clear(x: float, y: float) -> bool:
        if any(math.hypot(x - px, y - py) < radius + clearance for px, py, radius in danger_points):
            return False
        return not any(math.hypot(x - px, y - py) < clearance for px, py in static_points)

    def static_clear(x: float, y: float) -> bool:
        return not any(math.hypot(x - px, y - py) < clearance for px, py in static_points)

    for ring in range(1, ring_count + 1):
        radius = min(search_radius, ring * spacing)
        candidates = []
        sample_count = max(16, math.ceil(2.0 * math.pi * radius / spacing))
        for index in range(sample_count):
            angle = 2.0 * math.pi * index / sample_count
            x = ox + radius * math.cos(angle)
            y = oy + radius * math.sin(angle)
            if not clear(x, y):
                continue
            segment_clear = True
            for step in range(1, ring + 1):
                fraction = step / ring
                if not static_clear(ox + (x - ox) * fraction, oy + (y - oy) * fraction):
                    segment_clear = False
                    break
            if segment_clear:
                heading_error = abs(math.atan2(math.sin(angle - preferred_heading), math.cos(angle - preferred_heading)))
                candidates.append((heading_error, x, y))
        if candidates:
            _, x, y = min(candidates)
            return x, y
    return None


def incoming_collision_imminent(
    pose: tuple[float, float, float],
    tracks: list[HumanTrack],
    ttc_limit: float,
    dcpa_limit: float,
    minimum_person_speed: float,
) -> bool:
    robot_x, robot_y, _ = pose
    for track in tracks:
        if track.state not in ("CONFIRMED", "OCCLUDED"):
            continue
        if math.hypot(track.vx, track.vy) < minimum_person_speed:
            continue
        metrics = relative_motion_metrics(
            robot_x,
            robot_y,
            0.0,
            0.0,
            track.x,
            track.y,
            track.vx,
            track.vy,
        )
        if ttc_is_relevant(metrics, ttc_limit, dcpa_limit):
            return True
    return False


@dataclass
class Candidate:
    name: str
    vx: float
    vy: float
    wz: float
    preferred_state: str = ""
    follow_vx: float | None = None
    switch_time: float = 1.2


@dataclass
class CandidateResult:
    candidate: Candidate
    score: float
    rejected: bool
    reason: str
    trajectory: list[tuple[float, float, float, float]]
    min_human_distance: float
    min_tcpa: float
    front_cut_cost: float
    score_terms: dict[str, float]


class HumanAwareMotionPlanner(Node):
    """Sampled human-aware velocity planner above the hard safety filter."""

    def __init__(self) -> None:
        super().__init__("human_aware_motion_planner")
        self.declare_parameter("input_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("output_cmd_vel_topic", "/cmd_vel_human_aware")
        self.declare_parameter("tracked_cloud_topic", "/tracked_dynamic_obstacles")
        self.declare_parameter("predicted_cloud_topic", "/predicted_dynamic_obstacles")
        self.declare_parameter("tentative_cloud_topic", "/tentative_dynamic_obstacles")
        self.declare_parameter("debug_tracks_topic", "/dynamic_obstacle_tracker/debug_tracks")
        self.declare_parameter("tracked_persons_topic", "/tracked_persons")
        self.declare_parameter("predicted_trajectories_topic", "/predicted_person_trajectories")
        self.declare_parameter("use_legacy_person_inputs", False)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("rate_hz", 12.0)
        self.declare_parameter("prediction_horizon", 5.0)
        self.declare_parameter("simulation_dt", 0.2)
        self.declare_parameter("input_timeout", 0.6)
        self.declare_parameter("max_person_cloud_age", 0.8)
        self.declare_parameter("max_scan_age", 0.6)
        self.declare_parameter("robot_radius", 0.38)
        self.declare_parameter("human_body_radius", 0.35)
        self.declare_parameter("body_prediction_margin", 0.05)
        self.declare_parameter("hard_safety_margin", 0.40)
        self.declare_parameter("comfort_radius", 1.40)
        self.declare_parameter("front_comfort_scale", 1.7)
        self.declare_parameter("front_arc_risk_enabled", True)
        self.declare_parameter("front_arc_angle_deg", 30.0)
        self.declare_parameter("front_arc_distance", 1.5)
        self.declare_parameter("front_arc_radius_scale", 1.15)
        self.declare_parameter("front_arc_weight", 0.55)
        self.declare_parameter("uncertainty_growth_rate", 0.18)
        self.declare_parameter("low_conf_uncertainty_gain", 0.35)
        self.declare_parameter("occluded_risk_scale", 0.5)
        self.declare_parameter("tentative_caution_radius", 1.0)
        self.declare_parameter("ttc_warning_threshold", 3.0)
        self.declare_parameter("ttc_hard_threshold", 1.0)
        self.declare_parameter("dcpa_comfort_threshold", 1.4)
        self.declare_parameter("crossing_angle_min_deg", 35.0)
        self.declare_parameter("side_commitment_timeout", 2.0)
        self.declare_parameter("stale_track_decay_time", 0.8)
        self.declare_parameter("yield_timeout", 4.0)
        self.declare_parameter("deadlock_timeout", 6.0)
        self.declare_parameter("max_forward_near_human", 0.35)
        self.declare_parameter("max_lateral_near_human", 0.16)
        self.declare_parameter("max_reverse_velocity", 0.10)
        self.declare_parameter("max_angular_near_human", 0.55)
        self.declare_parameter("static_clearance", 0.48)
        self.declare_parameter("static_influence_range", 1.25)
        self.declare_parameter("w_goal", 1.0)
        self.declare_parameter("w_path", 0.8)
        self.declare_parameter("w_static", 8.0)
        self.declare_parameter("w_personal_space", 12.0)
        self.declare_parameter("w_ttc", 10.0)
        self.declare_parameter("w_pass_front", 18.0)
        self.declare_parameter("w_yield", 8.0)
        self.declare_parameter("w_smooth", 4.0)
        self.declare_parameter("w_jerk", 3.0)
        self.declare_parameter("w_legibility", 6.0)
        self.declare_parameter("w_deadlock", 4.0)
        self.declare_parameter("publish_risk_grid", True)
        self.declare_parameter("risk_grid_size", 8.0)
        self.declare_parameter("risk_grid_resolution", 0.20)
        self.declare_parameter("local_safe_space_enabled", False)
        self.declare_parameter("response_delay", 0.10)
        self.declare_parameter("command_duration", 1.7)
        self.declare_parameter("hard_collision_horizon", 6.0)
        self.declare_parameter("social_cost_decay", 0.35)
        self.declare_parameter("odom_timeout", 0.4)
        self.declare_parameter("require_fresh_tracks", True)
        self.declare_parameter("visualization_rate_hz", 2.0)
        self.declare_parameter("holonomic_avoidance", False)
        self.declare_parameter("prefer_forward_travel", False)
        self.declare_parameter("static_memory_seconds", 10.0)
        self.declare_parameter("use_low_obstacle_cloud", False)
        self.declare_parameter("use_static_scene_map", False)
        self.declare_parameter("nav2_path_reference", False)
        self.declare_parameter("retain_observed_routes", False)
        self.declare_parameter("planning_budget_ms", 0.0)
        self.planning_budget_ms = float(self.get_parameter("planning_budget_ms").value)
        self.search_phase = 0
        self.search_budget_exhausted = False
        self.observed_map = None
        self.observed_map_time = None
        self.observed_route_valid = None
        self.declare_parameter("local_escape_enabled", False)
        self.declare_parameter("path_reference_speed", 0.35)
        self.declare_parameter("path_reference_lookahead", 0.8)
        self.declare_parameter("local_safe_radius", 1.35)
        self.declare_parameter("local_danger_radius", 4.0)
        self.declare_parameter("predicted_danger_radius", 1.25)
        self.declare_parameter("safe_space_max_speed", 0.22)
        self.declare_parameter("safe_space_max_lateral", 0.18)
        self.declare_parameter("safe_space_max_yaw", 0.45)
        self.declare_parameter("safe_space_max_tilt", 0.45)
        self.declare_parameter("safe_patch_search_radius", 3.0)
        self.declare_parameter("safe_patch_spacing", 0.30)
        self.declare_parameter("safe_patch_clearance", 0.55)
        self.declare_parameter("collision_reflex_ttc", 6.0)
        self.declare_parameter("collision_reflex_dcpa", 1.10)
        self.declare_parameter("collision_reflex_min_person_speed", 0.12)

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.horizon = float(self.get_parameter("prediction_horizon").value)
        self.dt = float(self.get_parameter("simulation_dt").value)
        self.input_timeout = float(self.get_parameter("input_timeout").value)
        self.max_person_cloud_age = float(self.get_parameter("max_person_cloud_age").value)
        self.max_scan_age = float(self.get_parameter("max_scan_age").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.human_body_radius = float(self.get_parameter("human_body_radius").value)
        self.body_prediction_margin = float(self.get_parameter("body_prediction_margin").value)
        self.hard_safety_margin = float(self.get_parameter("hard_safety_margin").value)
        self.comfort_radius = float(self.get_parameter("comfort_radius").value)
        self.front_comfort_scale = float(self.get_parameter("front_comfort_scale").value)
        self.front_arc_risk_enabled = bool(self.get_parameter("front_arc_risk_enabled").value)
        self.front_arc_angle = math.radians(float(self.get_parameter("front_arc_angle_deg").value))
        self.front_arc_distance = float(self.get_parameter("front_arc_distance").value)
        self.front_arc_radius_scale = float(self.get_parameter("front_arc_radius_scale").value)
        self.front_arc_weight = float(self.get_parameter("front_arc_weight").value)
        self.uncertainty_growth_rate = float(self.get_parameter("uncertainty_growth_rate").value)
        self.low_conf_uncertainty_gain = float(self.get_parameter("low_conf_uncertainty_gain").value)
        self.occluded_risk_scale = float(self.get_parameter("occluded_risk_scale").value)
        self.tentative_caution_radius = float(self.get_parameter("tentative_caution_radius").value)
        self.ttc_warning_threshold = float(self.get_parameter("ttc_warning_threshold").value)
        self.ttc_hard_threshold = float(self.get_parameter("ttc_hard_threshold").value)
        self.dcpa_comfort_threshold = float(self.get_parameter("dcpa_comfort_threshold").value)
        self.crossing_angle_min = math.radians(float(self.get_parameter("crossing_angle_min_deg").value))
        self.side_commitment_timeout = float(self.get_parameter("side_commitment_timeout").value)
        self.stale_track_decay_time = float(self.get_parameter("stale_track_decay_time").value)
        self.yield_timeout = float(self.get_parameter("yield_timeout").value)
        self.deadlock_timeout = float(self.get_parameter("deadlock_timeout").value)
        self.max_forward_near_human = float(self.get_parameter("max_forward_near_human").value)
        self.max_lateral_near_human = float(self.get_parameter("max_lateral_near_human").value)
        self.max_reverse_velocity = float(self.get_parameter("max_reverse_velocity").value)
        self.max_angular_near_human = float(self.get_parameter("max_angular_near_human").value)
        self.static_clearance = float(self.get_parameter("static_clearance").value)
        self.static_influence_range = float(self.get_parameter("static_influence_range").value)
        self.weights = {
            name: float(self.get_parameter(name).value)
            for name in (
                "w_goal",
                "w_path",
                "w_static",
                "w_personal_space",
                "w_ttc",
                "w_pass_front",
                "w_yield",
                "w_smooth",
                "w_jerk",
                "w_legibility",
                "w_deadlock",
            )
        }
        self.publish_risk_grid = bool(self.get_parameter("publish_risk_grid").value)
        self.risk_grid_size = float(self.get_parameter("risk_grid_size").value)
        self.risk_grid_resolution = float(self.get_parameter("risk_grid_resolution").value)
        self.local_safe_space_enabled = bool(self.get_parameter("local_safe_space_enabled").value)
        self.local_safe_radius = float(self.get_parameter("local_safe_radius").value)
        self.local_danger_radius = float(self.get_parameter("local_danger_radius").value)
        self.predicted_danger_radius = float(self.get_parameter("predicted_danger_radius").value)
        self.safe_space_max_speed = float(self.get_parameter("safe_space_max_speed").value)
        self.safe_space_max_lateral = float(self.get_parameter("safe_space_max_lateral").value)
        self.safe_space_max_yaw = float(self.get_parameter("safe_space_max_yaw").value)
        self.safe_space_max_tilt = float(self.get_parameter("safe_space_max_tilt").value)
        self.safe_patch_search_radius = float(self.get_parameter("safe_patch_search_radius").value)
        self.safe_patch_spacing = float(self.get_parameter("safe_patch_spacing").value)
        self.safe_patch_clearance = float(self.get_parameter("safe_patch_clearance").value)
        self.collision_reflex_ttc = float(self.get_parameter("collision_reflex_ttc").value)
        self.collision_reflex_dcpa = float(self.get_parameter("collision_reflex_dcpa").value)
        self.collision_reflex_min_person_speed = float(
            self.get_parameter("collision_reflex_min_person_speed").value
        )
        self.use_legacy_person_inputs = bool(self.get_parameter("use_legacy_person_inputs").value)
        self.motion_envelope = load_motion_envelope(self)
        self.response_delay = float(self.get_parameter("response_delay").value)
        self.command_duration = float(self.get_parameter("command_duration").value)
        self.hard_collision_horizon = float(self.get_parameter("hard_collision_horizon").value)
        self.social_cost_decay = float(self.get_parameter("social_cost_decay").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.require_fresh_tracks = bool(self.get_parameter("require_fresh_tracks").value)
        self.visualization_period = 1.0 / max(0.1, float(self.get_parameter("visualization_rate_hz").value))
        self.holonomic_avoidance = bool(self.get_parameter("holonomic_avoidance").value)
        self.prefer_forward_travel = bool(self.get_parameter("prefer_forward_travel").value)
        self.static_memory_seconds = float(self.get_parameter("static_memory_seconds").value)
        self.static_memory = {}
        self.low_obstacle_memory = {}
        self.static_scene_points = None
        self.use_static_scene_map = bool(self.get_parameter("use_static_scene_map").value)
        self.static_scan_stamp = None
        self.static_error = ""
        self.nav2_path_reference = bool(self.get_parameter("nav2_path_reference").value)
        self.local_escape_enabled = bool(self.get_parameter("local_escape_enabled").value)
        self.path_reference_speed = float(self.get_parameter("path_reference_speed").value)
        self.path_reference_lookahead = float(self.get_parameter("path_reference_lookahead").value)
        self.reference_path = None
        self.reference_goal = None
        self.reference_goal_world = None
        self.reference_path_time = None
        self.path_reference_status = "not_requested"
        self.mission_active = False
        self.mission_state_time = None
        self.upstream_cmd = Twist()
        self.last_visualization = -math.inf
        self.measured_velocity = (0.0, 0.0, 0.0)
        self.policy_command = None
        self.policy_command_stamp = None
        self.odom_stamp = None
        self.odom_pose = None
        self.odom_pose_history = {}
        self.planning_started = time.perf_counter()
        self.search_budget_exhausted = False
        self.planning_cpu_started = time.thread_time()
        self.planning_phases = {}
        self.prediction_cache = {}
        self.static_cache = {}
        self.static_points_cache = None
        self.static_field_cache = None
        self.static_frame_valid = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.latest_scan: LaserScan | None = None
        self.latest_scan_time = None
        self.tracks: dict[int, HumanTrack] = {}
        self.tracks_epoch: float | None = None
        self.legacy_tracks: dict[int, HumanTrack] = {}
        self.predicted_points: list[tuple[float, float, float]] = []
        self.predicted_time = None
        self.predictions: dict[int, list[PredictionSample]] = {}
        self.prediction_epoch: float | None = None
        self.tentative_points: list[tuple[float, float, float]] = []
        self.tentative_time = None
        self.state = "NORMAL_NAVIGATION"
        self.state_since = self.get_clock().now()
        self.side_commitment = 0.0
        self.side_commitment_until = self.get_clock().now()
        self.last_cmd = Twist()
        self.last_tilt = 0.0
        self.last_progress_time = self.get_clock().now()
        self.last_debug = {}
        self.selected_safe_patch: tuple[float, float] | None = None

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("output_cmd_vel_topic").value), 1)
        self.state_pub = self.create_publisher(String, "/human_avoidance_state", 1)
        self.debug_pub = self.create_publisher(String, "/human_avoidance_debug", 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/human_social_risk_grid", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/human_avoidance_markers", 10)
        self.safe_patch_pub = self.create_publisher(PoseStamped, "/human_avoidance_safe_patch", 10)
        self.create_subscription(Twist, str(self.get_parameter("input_cmd_vel_topic").value), self._cmd_callback, 1)
        self.create_subscription(
            TrackedPersons, str(self.get_parameter("tracked_persons_topic").value), self._tracked_persons_callback, 1
        )
        self.create_subscription(
            PredictedTrajectories,
            str(self.get_parameter("predicted_trajectories_topic").value),
            self._predicted_trajectories_callback,
            1,
        )
        if self.use_legacy_person_inputs:
            self.create_subscription(String, str(self.get_parameter("debug_tracks_topic").value), self._debug_tracks_callback, 10)
            self.create_subscription(PointCloud2, str(self.get_parameter("predicted_cloud_topic").value), self._predicted_callback, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, str(self.get_parameter("tentative_cloud_topic").value), self._tentative_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self._scan_callback, LATEST_SENSOR_QOS)
        self.create_subscription(LowState, "/lowstate", self._lowstate_callback, LATEST_SENSOR_QOS)
        self.create_subscription(Odometry, "/odom", self._odom_callback, LATEST_SENSOR_QOS)
        if bool(self.get_parameter("use_low_obstacle_cloud").value):
            self.create_subscription(PointCloud2, "/livox/lidar_world", self._low_obstacle_callback, qos_profile_sensor_data)
        if self.use_static_scene_map:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, "/simulation/static_map", self._scene_map_callback, qos)
        self.create_subscription(TwistStamped, "/g1/policy_cmd_vel", self._policy_command_callback, LATEST_SENSOR_QOS)
        if self.nav2_path_reference:
            if bool(self.get_parameter("retain_observed_routes").value):
                qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
                self.create_subscription(OccupancyGrid, "/map", self._observed_map_callback, qos)
            self.create_subscription(Path, "/plan", self._path_callback, 1)
            self.create_subscription(PoseStamped, "/benchmark/current_goal", self._goal_callback, 1)
            self.create_subscription(Bool, "/benchmark/goal_active", self._mission_callback, 1)
        self.create_timer(1.0 / max(1.0, float(self.get_parameter("rate_hz").value)), self._timer_callback)
        self.get_logger().info("Human-aware sampled motion planner ready")

    def _cmd_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.upstream_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def _path_callback(self, msg: Path) -> None:
        if not msg.poses:
            return
        if not all(math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y) for p in msg.poses):
            return
        goal = self.reference_goal
        endpoint = msg.poses[-1].pose.position
        if goal is not None and (msg.header.frame_id != goal.header.frame_id or
                math.hypot(endpoint.x - goal.pose.position.x, endpoint.y - goal.pose.position.y) > 0.30):
            return
        if (self.reference_path is not None and msg.header.frame_id == self.reference_path.header.frame_id
                and _ros_time_seconds(msg.header.stamp) < _ros_time_seconds(self.reference_path.header.stamp)):
            return  # A late result must not replace a newer route to the same goal.
        self.reference_path = msg
        self.observed_route_valid = None
        self.reference_path_time = self.get_clock().now()

    def _observed_map_callback(self, msg):
        self.observed_map = msg
        self.observed_map_time = self.get_clock().now()
        self.observed_route_valid = None

    def _retained_route_observed(self, path):
        if getattr(self, "observed_map_time", None) is None or (self.get_clock().now() - self.observed_map_time).nanoseconds * 1e-9 > 2.5:
            return False
        if self.observed_route_valid is None:
            self.observed_route_valid = route_is_free(self.observed_map, path, self.robot_radius)
        return self.observed_route_valid

    def _mission_callback(self, msg: Bool) -> None:
        self.mission_active = bool(msg.data)
        self.mission_state_time = self.get_clock().now()

    def _goal_callback(self, msg: PoseStamped) -> None:
        self.reference_goal = msg
        self.reference_goal_world = None
        if self.reference_path is not None:
            endpoint = self.reference_path.poses[-1].pose.position
            if self.reference_path.header.frame_id != msg.header.frame_id or math.hypot(endpoint.x - msg.pose.position.x, endpoint.y - msg.pose.position.y) > 0.30:
                self.reference_path = None
                self.reference_path_time = None

    def _path_reference(self, pose) -> Twist | None:
        path, goal = self.reference_path, self.reference_goal
        self.path_reference_status = "missing_path_or_goal"
        if path is None or goal is None or not path.poses or path.header.frame_id != goal.header.frame_id:
            return None
        if self.reference_path_time is None or (not self.use_static_scene_map and (self.get_clock().now() - self.reference_path_time).nanoseconds * 1e-9 > 2.5 and not self._retained_route_observed(path)):
            self.path_reference_status = "stale_path"
            return None
        endpoint = path.poses[-1].pose.position
        if math.hypot(endpoint.x - goal.pose.position.x, endpoint.y - goal.pose.position.y) > 0.30:
            self.path_reference_status = "path_goal_mismatch"
            return None  # ComputePathToPose also publishes paths for rejected samples.
        points = np.asarray([(p.pose.position.x, p.pose.position.y) for p in path.poses])
        robot_in_path_frame = pose[:2]
        if path.header.frame_id != self.target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.target_frame, path.header.frame_id, rclpy.time.Time())
            except TransformException:
                self.path_reference_status = "missing_path_tf"
                return None
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            rx, ry = pose[0]-tf.transform.translation.x, pose[1]-tf.transform.translation.y
            robot_in_path_frame = (c*rx+s*ry, -s*rx+c*ry)
            points = points @ np.asarray(((c, s), (-s, c)))
            points += (tf.transform.translation.x, tf.transform.translation.y)
        goal_distance = float(np.linalg.norm(points[-1] - pose[:2]))
        self.reference_goal_world = tuple(points[-1])
        command = Twist()
        self.path_reference_status = "active"
        if goal_distance <= 0.4:
            self.path_reference_status = "at_goal"
            return command
        nearest = int(np.argmin(np.sum((points - pose[:2]) ** 2, axis=1)))
        index, distance = nearest, 0.0
        while index + 1 < len(points) and distance < self.path_reference_lookahead:
            distance += float(np.linalg.norm(points[index + 1] - points[index]))
            index += 1
        if (getattr(self, "prefer_forward_travel", False) and goal_distance <= 3. and
                getattr(self, "observed_map_time", None) is not None and
                (self.get_clock().now()-self.observed_map_time).nanoseconds*1e-9 <= 2.5):
            shortcut = Path()
            shortcut.header.frame_id = path.header.frame_id
            current = PoseStamped()
            current.pose.position.x, current.pose.position.y = robot_in_path_frame
            shortcut.poses = [current, goal]
            if route_is_free(self.observed_map, shortcut, self.robot_radius+.10):
                # Smooth a short route only through known-free footprint cells.
                # Dynamic people are still checked by every candidate rollout.
                index = len(points)-1
                self.path_reference_status = "observed_goal_shortcut"
        dx, dy = points[index] - pose[:2]
        norm = max(1e-6, math.hypot(dx, dy))
        # The official policy stalls at tiny requested forward speeds. Keep a
        # useful gait until the arrival region instead of asymptotically parking
        # just outside the benchmark's 0.5 m handoff radius.
        minimum_speed = .30 if getattr(self, "prefer_forward_travel", False) else .08
        speed = min(self.path_reference_speed, max(minimum_speed, 0.6 * (goal_distance - 0.35)))
        command.linear.x = speed * (math.cos(pose[2]) * dx + math.sin(pose[2]) * dy) / norm
        command.linear.y = speed * (-math.sin(pose[2]) * dx + math.cos(pose[2]) * dy) / norm
        if getattr(self, "prefer_forward_travel", False) and goal_distance > 1.0:
            command.linear.x, command.linear.y, command.angular.z = forward_reference(
                math.cos(pose[2])*dx + math.sin(pose[2])*dy,
                -math.sin(pose[2])*dx + math.cos(pose[2])*dy,
                speed, self.max_angular_near_human)
        elif getattr(self, "prefer_forward_travel", False):
            # Finish the final short adjustment holonomically. An executable
            # forward-turn arc has a finite radius and otherwise circles a
            # nearby goal instead of entering the 0.5 m arrival region.
            scale = min(1., .30 / max(1e-6, math.hypot(command.linear.x, command.linear.y)))
            command.linear.x *= scale
            command.linear.y *= scale
        return command

    def _scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time = rclpy.time.Time.from_msg(msg.header.stamp)

    def _odom_callback(self, msg: Odometry) -> None:
        velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
        if all(math.isfinite(value) for value in velocity):
            self.measured_velocity = velocity
            self.odom_stamp = _ros_time_seconds(msg.header.stamp)
            if msg.header.frame_id == self.target_frame:
                q = msg.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
                self.odom_pose_history[self.odom_stamp] = self.odom_pose
                self.odom_pose_history = {stamp: pose for stamp, pose in self.odom_pose_history.items() if self.odom_stamp - stamp <= 2.0}

    def _scene_map_callback(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id != self.target_frame or len(msg.data) != msg.info.width * msg.info.height:
            return
        ys, xs = np.nonzero(np.asarray(msg.data).reshape((msg.info.height, msg.info.width)) >= 50)
        points = np.column_stack(((xs + 0.5) * msg.info.resolution, (ys + 0.5) * msg.info.resolution))
        q = msg.info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        self.static_scene_points = points @ np.asarray(((c, s), (-s, c))) + (msg.info.origin.position.x, msg.info.origin.position.y)

    def _low_obstacle_callback(self, msg: PointCloud2) -> None:
        if msg.header.frame_id != self.target_frame:
            return
        stamp = _ros_time_seconds(msg.header.stamp)
        if not 0.0 <= self.get_clock().now().nanoseconds * 1e-9 - stamp <= self.max_scan_age:
            return
        people = [(t.x + t.vx * (stamp - t.stamp), t.y + t.vy * (stamp - t.stamp), t.body_radius + 0.15) for t in self.tracks.values()]
        def person_return(x, y):
            return any(math.hypot(x - px, y - py) <= radius for px, py, radius in people)
        for x, y, z in iter_xyz(msg):
            if 0.08 <= z <= 0.55 and not person_return(x, y):
                self.low_obstacle_memory[(round(x / 0.10), round(y / 0.10))] = (x, y, stamp)
        self.low_obstacle_memory = {key: value for key, value in self.low_obstacle_memory.items()
                                    if 0.0 <= stamp - value[2] <= 60.0 and not person_return(value[0], value[1])}

    def _policy_command_callback(self, msg: TwistStamped) -> None:
        values = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        if all(math.isfinite(value) for value in values):
            self.policy_command = values
            self.policy_command_stamp = _ros_time_seconds(msg.header.stamp)

    def _lowstate_callback(self, msg: LowState) -> None:
        rpy = msg.imu_state.rpy
        roll = math.atan2(math.sin(float(rpy[0])), math.cos(float(rpy[0])))
        pitch = math.atan2(math.sin(float(rpy[1])), math.cos(float(rpy[1])))
        self.last_tilt = max(abs(roll), abs(pitch))

    def _tentative_callback(self, msg: PointCloud2) -> None:
        self.tentative_points = list(iter_xyz(msg))[:64]
        self.tentative_time = _ros_time_seconds(msg.header.stamp)

    def _predicted_callback(self, msg: PointCloud2) -> None:
        self.predicted_points = list(iter_xyz(msg))[:256]
        self.predicted_time = _ros_time_seconds(msg.header.stamp)

    def _tracked_persons_callback(self, msg: TrackedPersons) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            self.get_logger().warn(
                f"Ignoring tracked persons in {msg.header.frame_id}; expected {self.target_frame}",
                throttle_duration_sec=2.0,
            )
            return
        stamp = _ros_time_seconds(msg.header.stamp)
        tracks = {}
        for person in msg.persons:
            if person.track_state not in (
                TrackedPerson.TRACK_STATE_TENTATIVE,
                TrackedPerson.TRACK_STATE_CONFIRMED,
                TrackedPerson.TRACK_STATE_LOST,
            ):
                continue
            tracks[int(person.track_id)] = HumanTrack(
                track_id=int(person.track_id),
                state={TrackedPerson.TRACK_STATE_CONFIRMED: "CONFIRMED", TrackedPerson.TRACK_STATE_LOST: "OCCLUDED", TrackedPerson.TRACK_STATE_TENTATIVE: "TENTATIVE"}[person.track_state],
                x=float(person.pose.pose.position.x),
                y=float(person.pose.pose.position.y),
                vx=float(person.velocity.twist.linear.x),
                vy=float(person.velocity.twist.linear.y),
                confidence=max(0.0, min(1.0, float(person.confidence))),
                person_score=max(0.0, min(1.0, float(person.confidence))),
                missed_count=1 if person.track_state == TrackedPerson.TRACK_STATE_LOST else 0,
                cov_xx=max(0.0, float(person.pose.covariance[0])),
                cov_yy=max(0.0, float(person.pose.covariance[7])),
                stamp=stamp,
                body_radius=max(0.0, float(person.body_radius)),
                observation_age=_ros_time_seconds(person.observation_age),
            )
        self.tracks = tracks
        self.tracks_epoch = stamp

    def _predicted_trajectories_callback(self, msg: PredictedTrajectories) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            self.get_logger().warn(
                f"Ignoring person predictions in {msg.header.frame_id}; expected {self.target_frame}",
                throttle_duration_sec=2.0,
            )
            return
        predictions = {}
        for trajectory in msg.trajectories:
            points = []
            previous_offset = -1.0
            for point in trajectory.points:
                offset = _ros_time_seconds(point.time_offset)
                values = (point.position.x, point.position.y, point.velocity.x, point.velocity.y)
                if offset < 0.0 or offset <= previous_offset or not all(math.isfinite(float(value)) for value in values):
                    continue
                points.append(PredictionSample(
                    time_offset=offset,
                    x=float(point.position.x),
                    y=float(point.position.y),
                    vx=float(point.velocity.x),
                    vy=float(point.velocity.y),
                    cov_xx=max(0.0, float(point.position_covariance[0])),
                    cov_yy=max(0.0, float(point.position_covariance[4])),
                    confidence=max(0.0, min(1.0, float(point.confidence))),
                ))
                previous_offset = offset
            if points:
                predictions[int(trajectory.track_id)] = points
        self.predictions = predictions
        self.prediction_epoch = _ros_time_seconds(msg.header.stamp)

    def _debug_tracks_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        stamp = float(payload.get("stamp", self.get_clock().now().nanoseconds * 1e-9))
        tracks = {}
        for item in payload.get("tracks", []):
            state = str(item.get("state", ""))
            if state not in ("CONFIRMED", "OCCLUDED"):
                continue
            try:
                track_id = int(item["track_id"])
                tracks[track_id] = HumanTrack(
                    track_id=track_id,
                    state=state,
                    x=float(item.get("x", 0.0)),
                    y=float(item.get("y", 0.0)),
                    vx=float(item.get("vx", 0.0)),
                    vy=float(item.get("vy", 0.0)),
                    confidence=float(item.get("confidence", 0.0)),
                    person_score=float(item.get("person_score", 0.0)),
                    missed_count=int(item.get("missed_count", 0)),
                    cov_xx=float(item.get("cov_xx", 0.20)),
                    cov_yy=float(item.get("cov_yy", 0.20)),
                    stamp=stamp,
                    body_radius=self.human_body_radius,
                )
            except (TypeError, ValueError, KeyError):
                continue
        self.legacy_tracks = tracks

    def _timer_callback(self) -> None:
        if self.nav2_path_reference:
            self.latest_cmd = self.upstream_cmd
        self.planning_started = time.perf_counter()
        self.planning_cpu_started = time.thread_time()
        self.planning_phases = {}
        self.prediction_cache = {}
        self.static_cache = {}
        self.static_points_cache = None
        self.static_field_cache = None
        self.static_frame_valid = None
        now = self.get_clock().now()
        self.selected_safe_patch = None
        pose = self._robot_pose()
        tracks = self._fresh_tracks(now)
        tentative_points = self._fresh_tentative(now)
        predictions, prediction_age, typed_predictions_fresh = self._fresh_predictions(now)
        predicted_points = [] if typed_predictions_fresh else self._fresh_predicted(now)
        if self.nav2_path_reference:
            if self.mission_state_time is None or (now - self.mission_state_time).nanoseconds * 1e-9 > 1.0:
                self._publish(Twist(), "MISSION_STALE_STOP", [], None)
                return
            if not self.mission_active:
                self.latest_cmd = Twist()
        if self._input_stale(now):
            autonomous_risk = (self.nav2_path_reference and self.reference_goal is not None and pose is not None and
                               ((self.mission_active and self.reference_path is not None) or
                                incoming_collision_imminent(pose, tracks, self.horizon, self.robot_radius + self.human_body_radius + 0.5, 0.12)))
            if not autonomous_risk:
                self._publish(Twist(), "NORMAL_NAVIGATION", [], None)
                return
            self.latest_cmd = Twist()
        if pose is None:
            self._publish(Twist(), "EMERGENCY_STOP", [], None)
            return
        now_seconds = now.nanoseconds * 1e-9
        if self.require_fresh_tracks and (self.tracks_epoch is None or not 0.0 <= now_seconds - self.tracks_epoch <= self.max_person_cloud_age):
            self.last_cmd = Twist()
            self._publish(Twist(), "PERCEPTION_STALE_STOP", [], None)
            return
        if self.odom_stamp is None or not 0.0 <= now_seconds - self.odom_stamp <= self.odom_timeout:
            self.last_cmd = Twist()
            self._publish(Twist(), "ODOMETRY_STALE_STOP", [], None)
            return
        if self.nav2_path_reference and self.mission_active:
            reference = self._path_reference(pose)
            if reference is not None:
                self.latest_cmd = reference
        if self.local_safe_space_enabled and self.last_tilt <= self.safe_space_max_tilt:
            safe_selected, safe_results = self._select_local_safe_space_command(
                pose, tracks, tentative_points, predicted_points, predictions, prediction_age
            )
            if safe_selected is not None and safe_selected.candidate.name == "nearest_clear_patch":
                cmd = self._twist_from_candidate(safe_selected.candidate)
                self.last_cmd = cmd
                self._publish(cmd, "LOCAL_SAFE_SPACE", safe_results, safe_selected)
                return
        state = self._update_state(now, pose, tracks, tentative_points)
        # Full-horizon feasibility must activate search before the short CPA
        # state thresholds. Otherwise a rejected raw candidate means a late stop.
        if state == "NORMAL_NAVIGATION" and tracks:
            raw = self._candidate_commands(self.latest_cmd, state, pose, tracks)[0]
            raw_result = self._score_candidate(raw, pose, tracks, tentative_points, predicted_points, predictions, prediction_age)
            warning_distance = max(self.robot_radius + track.body_radius + self.hard_safety_margin for track in tracks)
            holding_risk = (getattr(self, "prefer_forward_travel", False) and
                incoming_collision_imminent(pose, tracks, 3., warning_distance+.2, .12))
            if holding_risk or raw_result.rejected or raw_result.min_human_distance < warning_distance or raw_result.score_terms.get("terminal_conflict", 0.0) > 0.0:
                state = "HUMAN_CAUTION"
                self.state = state
                self.state_since = now
                if self.side_commitment == 0.0 or now > self.side_commitment_until:
                    self.side_commitment = self._preferred_side(pose, tracks)
                    self.side_commitment_until = now + Duration(seconds=self.side_commitment_timeout)
        candidates = self._candidate_commands(self.latest_cmd, state, pose, tracks)
        candidates = self._executable_candidates(candidates)
        scoring_started = time.perf_counter()
        self.planning_phases["prepare_ms"] = (scoring_started - self.planning_started) * 1000.0
        results = self._evaluate_candidates(candidates, pose, tracks, tentative_points,
                                            predicted_points, predictions, prediction_age)
        feasible = [result for result in results if not result.rejected]
        self.planning_phases["scoring_ms"] = (time.perf_counter() - scoring_started) * 1000.0
        if not feasible:
            # A comfort-margin breach is preferable to a predicted body
            # collision while standing still. Never relax body/static geometry.
            recovery = [result for result in results if result.reason == "human_margin"]
            if recovery:
                feasible = [max(recovery, key=lambda result: result.min_human_distance)]
        if not feasible:
            if self.local_escape_enabled:
                escape = self._short_horizon_escape(results, tracks)
                if escape is not None:
                    feasible = [escape]
        if not feasible:
            feasible = [result for result in results if result.reason == "terminal_conflict"]
        selected = min(feasible, key=lambda result: result.score) if feasible else None
        if selected is None:
            cmd = Twist()
            state = "EMERGENCY_STOP"
        else:
            cmd = self._twist_from_candidate(selected.candidate)
            if selected.rejected:
                state = "CLEARANCE_RECOVERY"
            elif selected.candidate.name == "yield_stop" and tracks:
                state = "YIELD_TO_HUMAN"
            elif selected.candidate.preferred_state:
                state = selected.candidate.preferred_state
        self.last_cmd = cmd
        self._publish(cmd, state, results, selected)

    def _evaluate_candidates(self, candidates, pose, tracks, tentative_points, predicted_points, predictions, prediction_age):
        """Bound search latency; every selectable command is still fully checked.

        Always recheck stop, the previous command, and the route command first.
        Rotate additional candidates across ticks so a deadline cannot permanently
        starve a direction or a step-and-hold refinement.
        """
        budget = self.planning_budget_ms
        ordered = candidates
        mandatory = 3
        if budget > 0 and len(candidates) > 3:
            names = ("yield_stop", "early_crossing_escape", "previous_command", "raw_reference")
            priority = set(names)
            first = [c for name in names for c in candidates if c.name == name]
            mandatory = max(3, len(first))
            remaining = [c for c in candidates if c.name not in priority]
            if remaining:
                offset = (self.search_phase * 6) % len(remaining)
                remaining = remaining[offset:] + remaining[:offset]
            ordered = first + remaining
            self.search_phase += 1
        results = []
        self.search_budget_exhausted = False
        for candidate in ordered:
            if budget > 0 and len(results) >= mandatory and (time.perf_counter() - self.planning_started) * 1000 >= budget:
                self.search_budget_exhausted = True
                break
            results.append(self._score_candidate(candidate, pose, tracks, tentative_points,
                                                  predicted_points, predictions, prediction_age))
        return results

    def _short_horizon_escape(self, results, tracks):
        """Time-checked one-second escape when holding predicts body contact.

        This is an explicitly labelled receding-horizon recovery, not a claim
        that a constant command remains safe for the entire prediction horizon.
        """
        stop = next((r for r in results if r.candidate.name == "yield_stop"), None)
        if stop is None or stop.reason != "hard_human_collision" or not tracks:
            return None
        best = None
        best_gain = 0.05
        stop_points = np.asarray(stop.trajectory)
        for result in results:
            if result is stop:
                continue
            robot = np.asarray(result.trajectory)
            mask = robot[:, 3] <= 1.0 + 1e-6
            short = robot[mask]
            if not len(short) or self._static_obstacle_cost([tuple(p) for p in short])[1]:
                continue
            minimum = math.inf
            final_clearance = math.inf
            stop_clearance = math.inf
            safe = True
            for track in tracks:
                human, _, _ = self.prediction_cache[(track.track_id, tuple(robot[:, 3]))]
                h = human[mask, :2]
                radius = self.robot_radius + track.body_radius
                separation = np.linalg.norm(short[:, :2] - h, axis=1)
                if np.min(separation) < radius + 0.10:
                    safe = False
                    break
                minimum = min(minimum, float(np.min(separation)))
                final_clearance = min(final_clearance, float(separation[-1]) - radius)
                stop_clearance = min(stop_clearance, float(np.linalg.norm(stop_points[mask][-1, :2] - h[-1])) - radius)
            gain = final_clearance - stop_clearance
            if safe and gain > best_gain:
                best_gain = gain
                best = CandidateResult(result.candidate, -gain, True, "local_escape", [tuple(p) for p in short], minimum, 1.0, 0.0, {"clearance_gain": gain})
        return best

    def _fresh_tracks(self, now) -> list[HumanTrack]:
        now_seconds = now.nanoseconds * 1e-9
        if self.tracks_epoch is not None and 0.0 <= now_seconds - self.tracks_epoch <= self.max_person_cloud_age:
            return [align_track_to_time(track, now_seconds) for track in self.tracks.values()]
        if not self.use_legacy_person_inputs:
            return []
        return [
            align_track_to_time(track, now_seconds) for track in self.legacy_tracks.values()
            if 0.0 <= now_seconds - track.stamp <= self.max_person_cloud_age
        ]

    def _fresh_tentative(self, now) -> list[tuple[float, float, float]]:
        if not self.use_legacy_person_inputs or self.tentative_time is None:
            return []
        age = now.nanoseconds * 1e-9 - self.tentative_time
        if age < 0.0 or age > self.max_person_cloud_age:
            return []
        return self.tentative_points

    def _fresh_predicted(self, now) -> list[tuple[float, float, float]]:
        if not self.use_legacy_person_inputs or self.predicted_time is None:
            return []
        age = now.nanoseconds * 1e-9 - self.predicted_time
        if age < 0.0 or age > self.max_person_cloud_age:
            return []
        return self.predicted_points

    def _fresh_predictions(self, now) -> tuple[dict[int, list[PredictionSample]], float, bool]:
        if self.prediction_epoch is None:
            return {}, 0.0, False
        age = now.nanoseconds * 1e-9 - self.prediction_epoch
        if age < 0.0 or age > self.max_person_cloud_age:
            return {}, 0.0, False
        return self.predictions, age, True

    def _update_state(self, now, pose: tuple[float, float, float], tracks: list[HumanTrack], tentative_points) -> str:
        min_distance = math.inf
        hard_ttc = False
        hard_distance = False
        warning_ttc = False
        crossing_front = False
        for track in tracks:
            if track.state == "TENTATIVE":
                if math.hypot(track.x - pose[0], track.y - pose[1]) < self.comfort_radius + 1.0:
                    warning_ttc = True
            metrics = self._relative_metrics_for_raw_cmd(pose, track)
            min_distance = min(min_distance, metrics["distance"])
            hard_radius = self.robot_radius + track.body_radius + self.hard_safety_margin
            hard_distance = hard_distance or metrics["distance"] <= hard_radius
            hard_ttc = hard_ttc or ttc_is_relevant(metrics, self.ttc_hard_threshold, hard_radius)
            warning_ttc = warning_ttc or ttc_is_relevant(metrics, self.ttc_warning_threshold, self.dcpa_comfort_threshold)
            crossing_front = crossing_front or metrics["front_cut"]
        next_state = self.state
        forward_blocked = self._human_blocks_forward_path(pose, tracks)
        state_age = (now - self.state_since).nanoseconds * 1e-9
        blocked_age = (now - self.last_progress_time).nanoseconds * 1e-9
        if hard_distance or hard_ttc:
            next_state = "EMERGENCY_STOP"
        elif crossing_front:
            next_state = "PASS_BEHIND"
        elif min_distance <= self.comfort_radius or warning_ttc:
            if forward_blocked:
                next_state = blocked_motion_state(
                    self.state, state_age, blocked_age, self.yield_timeout, self.deadlock_timeout
                )
            else:
                next_state = "HUMAN_CAUTION"
        elif tentative_points:
            next_state = "HUMAN_CAUTION"
        else:
            next_state = "NORMAL_NAVIGATION"
            self.last_progress_time = now
        if next_state != self.state:
            self.state_since = now
        self.state = next_state
        if self.state in ("HUMAN_CAUTION", "PASS_BEHIND") and now > self.side_commitment_until:
            side = self._preferred_side(pose, tracks)
            if side != 0.0:
                self.side_commitment = side
                self.side_commitment_until = now + Duration(seconds=self.side_commitment_timeout)
        return self.state

    def _candidate_commands(
        self,
        raw: Twist,
        state: str,
        pose: tuple[float, float, float],
        tracks: list[HumanTrack],
    ) -> list[Candidate]:
        raw_x = self._clamp(float(raw.linear.x), -self.max_reverse_velocity, self.max_forward_near_human)
        raw_y = self._clamp(float(raw.linear.y), -self.max_lateral_near_human, self.max_lateral_near_human)
        raw_w = self._clamp(float(raw.angular.z), -self.max_angular_near_human, self.max_angular_near_human)
        if state == "NORMAL_NAVIGATION":
            return [Candidate("raw_limited", raw_x, raw_y, raw_w)]
        if self.holonomic_avoidance and self.max_lateral_near_human > 0.0:
            # Keep the torso heading stable during close encounters. Translating
            # out of a crossing corridor is more predictable than a slow G1 yaw
            # transient. Nav2 heading tracking resumes once the conflict clears.
            incoming = incoming_collision_imminent(pose, tracks, self.horizon, self.robot_radius + self.human_body_radius + 0.5, 0.12)
            early = []
            if getattr(self, "prefer_forward_travel", False):
                for x, y in crossing_escape_directions(pose, [(t.x,t.y,t.vx,t.vy) for t in tracks],
                                                       clearance=self.robot_radius+self.human_body_radius+.6):
                    early.append(Candidate("early_crossing_escape",
                        x*(self.max_forward_near_human if x >= 0 else self.max_reverse_velocity),
                        y*self.max_lateral_near_human, 0.0))
            active_request = max(abs(raw_x), abs(raw_y), abs(raw_w)) > 0.03
            forward = self.max_forward_near_human if incoming or active_request else 0.0
            route_yaw = raw_w if getattr(self, "prefer_forward_travel", False) else 0.0
            translations = [Candidate("raw_reference", raw_x, raw_y, route_yaw),
                            Candidate("previous_command", self.last_cmd.linear.x, self.last_cmd.linear.y,
                                      self.last_cmd.angular.z if getattr(self, "prefer_forward_travel", False) else 0.0)] + [
                Candidate("escape_translation" if incoming else "social_translation", vx, vy, 0.0)
                for vx in (-self.max_reverse_velocity, 0.0, 0.5 * forward, forward)
                for vy in (-self.max_lateral_near_human, 0.0, self.max_lateral_near_human)
                if abs(vx) + abs(vy) > 1e-6
            ]
            # Search how long to translate, not just the direction. A six-second
            # constant retreat can hit someone that a short step-and-hold avoids.
            refinements = [Candidate("fine_translation", vx, vy, 0.0)
                           for vx in (-0.5 * self.max_reverse_velocity, 0.5 * forward, forward)
                           for vy in (-0.5 * self.max_lateral_near_human, 0.5 * self.max_lateral_near_human)]
            return [Candidate("yield_stop", 0.0, 0.0, 0.0, "YIELD_TO_HUMAN")] + early + translations + refinements + [
                Candidate("step_and_hold", c.vx, c.vy, 0.0, "", 0.0, duration)
                for c in translations for duration in (.6, 1.2, 2.0)
            ]
        side = self.side_commitment if self.side_commitment != 0.0 else 1.0
        candidates = [
            Candidate("raw_limited", raw_x, raw_y, raw_w),
            Candidate("half_speed", raw_x * 0.5, raw_y * 0.5, raw_w * 0.5),
            Candidate("quarter_speed", raw_x * 0.25, raw_y * 0.25, raw_w * 0.25),
            Candidate("slow_forward", max(0.0, min(max(raw_x * 0.65, 0.18), self.max_forward_near_human)), 0.0, raw_w * 0.5),
            Candidate("creep", min(0.18, self.max_forward_near_human), 0.0, 0.0),
            Candidate("yield_stop", 0.0, 0.0, 0.0, "YIELD_TO_HUMAN"),
            Candidate("sidestep_left", 0.10, self.max_lateral_near_human, 0.0, "SIDESTEP_LEFT"),
            Candidate("sidestep_right", 0.10, -self.max_lateral_near_human, 0.0, "SIDESTEP_RIGHT"),
            Candidate("lateral_left", 0.0, self.max_lateral_near_human, 0.0, "SIDESTEP_LEFT"),
            Candidate("lateral_right", 0.0, -self.max_lateral_near_human, 0.0, "SIDESTEP_RIGHT"),
            Candidate("arc_left", 0.20, 0.0, self.max_angular_near_human * 0.65),
            Candidate("arc_right", 0.20, 0.0, -self.max_angular_near_human * 0.65),
            Candidate("turn_left", 0.0, 0.0, self.max_angular_near_human * 0.55),
            Candidate("turn_right", 0.0, 0.0, -self.max_angular_near_human * 0.55),
            Candidate("retreat", -self.max_reverse_velocity, 0.0, 0.0, "RETREAT_OR_BACKUP"),
        ]
        for speed in (raw_x, min(raw_x, 0.25)):
            for yaw in (-self.max_angular_near_human, -0.5 * self.max_angular_near_human, 0.5 * self.max_angular_near_human, self.max_angular_near_human):
                candidates.append(Candidate(f"sample_{speed:.2f}_{yaw:.2f}", speed, 0.0, yaw))
        if self.max_lateral_near_human > 0.0:
            for speed in (0.0, 0.5 * raw_x, raw_x):
                for lateral in (-self.max_lateral_near_human, self.max_lateral_near_human):
                    for yaw in (0.0, raw_w):
                        candidates.append(Candidate("velocity_grid", speed, lateral, yaw))
        away = self._move_away_candidate(pose, tracks)
        if away is not None:
            candidates.insert(1, away)
        if state in ("PASS_BEHIND", "HUMAN_CAUTION"):
            candidates.insert(1, Candidate("committed_side", 0.14, side * self.max_lateral_near_human, -side * self.max_angular_near_human * 0.25))
        if state == "DEADLOCK_RECOVERY":
            lateral = side * self.max_lateral_near_human
            candidates = [
                Candidate("deadlock_sidestep", 0.0, lateral, -side * self.max_angular_near_human * 0.25),
                Candidate("deadlock_retreat", -self.max_reverse_velocity, 0.5 * lateral, 0.0, "RETREAT_OR_BACKUP"),
                Candidate("deadlock_turn", 0.0, 0.0, -side * self.max_angular_near_human * 0.55),
                Candidate("yield_stop", 0.0, 0.0, 0.0, "YIELD_TO_HUMAN"),
            ]
        # A stopped robot is not safe if a person is walking into it. Keep
        # executable escape candidates available even in emergency/yield mode.
        for candidate in candidates:
            candidate.vx = no_unrequested_forward(candidate.vx, raw_x)
            if raw_x >= 0.0:
                candidate.vx = min(candidate.vx, raw_x)
        if tracks and incoming_collision_imminent(pose, tracks, self.horizon, self.robot_radius + self.human_body_radius + 0.5, 0.12):
            # Active avoidance may need more speed than a blocked Nav2 command.
            # Every escape is still checked against people and static geometry.
            for yaw in (-self.max_angular_near_human, 0.0, self.max_angular_near_human):
                candidates.append(Candidate("escape_arc", self.max_forward_near_human, 0.0, yaw, "CLEARANCE_RECOVERY"))
                for lateral in (-self.max_lateral_near_human, self.max_lateral_near_human):
                    candidates.append(Candidate("escape_diagonal", self.max_forward_near_human, lateral, yaw, "CLEARANCE_RECOVERY"))
                for duration in (1.0, 2.0):
                    candidates.append(Candidate("escape_turn_then_walk", 0.0, 0.0, yaw, "CLEARANCE_RECOVERY", self.max_forward_near_human, duration))
        return candidates

    def _executable_candidates(self, candidates: list[Candidate]) -> list[Candidate]:
        result = []
        seen = set()
        for candidate in candidates:
            command = self.motion_envelope.normalize((candidate.vx, candidate.vy, candidate.wz), stopped=max(abs(self.last_cmd.linear.x), abs(self.last_cmd.linear.y), abs(self.last_cmd.angular.z)) <= 1e-6)
            # A requested turn may require the configured forward arc floor.
            # Score that executable arc rather than discarding Nav2's turn.
            # An all-zero stop remains zero in MotionEnvelope.normalize().
            key = (command, candidate.follow_vx, candidate.switch_time)
            if key in seen:
                continue
            seen.add(key)
            result.append(Candidate(candidate.name, *command, candidate.preferred_state, candidate.follow_vx, candidate.switch_time))
        return result

    def _move_away_candidate(self, pose: tuple[float, float, float], tracks: list[HumanTrack]) -> Candidate | None:
        if not tracks:
            return None
        robot_x, robot_y, yaw = pose
        nearest = None
        nearest_distance = math.inf
        nearest_local_y = 0.0
        for track in tracks:
            dx = track.x - robot_x
            dy = track.y - robot_y
            local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
            local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            distance = math.hypot(dx, dy)
            if -0.4 <= local_x <= 3.0 and distance < nearest_distance:
                nearest = track
                nearest_distance = distance
                nearest_local_y = local_y
        if nearest is None or nearest_distance > max(2.8, self.comfort_radius * 2.0):
            return None
        if abs(nearest_local_y) < 0.18 and self.side_commitment != 0.0:
            side = self.side_commitment
        else:
            side = -1.0 if nearest_local_y >= 0.0 else 1.0
        return Candidate(
            "move_away_from_person",
            min(0.18, self.max_forward_near_human),
            side * self.max_lateral_near_human,
            -side * self.max_angular_near_human * 0.25,
            "SIDESTEP_LEFT" if side > 0.0 else "SIDESTEP_RIGHT",
        )

    def _select_local_safe_space_command(
        self,
        pose: tuple[float, float, float],
        tracks: list[HumanTrack],
        tentative_points: list[tuple[float, float, float]],
        predicted_points: list[tuple[float, float, float]],
        predictions: dict[int, list[PredictionSample]],
        prediction_age: float,
    ) -> tuple[CandidateResult | None, list[CandidateResult]]:
        danger_points = self._danger_zone_points(
            tracks, tentative_points, predicted_points, predictions, prediction_age
        )
        if not self._local_danger_present(pose, danger_points):
            return None, []

        speed = max(0.05, min(self.safe_space_max_speed, self.max_forward_near_human))
        lateral = max(0.0, min(self.safe_space_max_lateral, self.max_lateral_near_human))
        yaw = max(0.10, min(self.safe_space_max_yaw, self.max_angular_near_human))
        candidates = [
            Candidate("safe_hold", 0.0, 0.0, 0.0),
            Candidate("safe_retreat", -min(speed, self.max_reverse_velocity), 0.0, 0.0, "RETREAT_OR_BACKUP"),
            Candidate("safe_left", 0.0, lateral, 0.0, "SIDESTEP_LEFT"),
            Candidate("safe_right", 0.0, -lateral, 0.0, "SIDESTEP_RIGHT"),
            Candidate("safe_back_left", -min(speed, self.max_reverse_velocity), lateral, yaw * 0.35, "RETREAT_OR_BACKUP"),
            Candidate("safe_back_right", -min(speed, self.max_reverse_velocity), -lateral, -yaw * 0.35, "RETREAT_OR_BACKUP"),
            Candidate("safe_forward_left", speed, lateral, yaw * 0.45, "SIDESTEP_LEFT"),
            Candidate("safe_forward_right", speed, -lateral, -yaw * 0.45, "SIDESTEP_RIGHT"),
            Candidate("safe_arc_left", speed * 0.7, 0.0, yaw, "SIDESTEP_LEFT"),
            Candidate("safe_arc_right", speed * 0.7, 0.0, -yaw, "SIDESTEP_RIGHT"),
        ]
        patch = self._nearest_clear_patch(pose, danger_points)
        if patch is not None:
            self.selected_safe_patch = patch
            dx = patch[0] - pose[0]
            dy = patch[1] - pose[1]
            local_x = math.cos(pose[2]) * dx + math.sin(pose[2]) * dy
            local_y = -math.sin(pose[2]) * dx + math.cos(pose[2]) * dy
            distance = max(1e-6, math.hypot(local_x, local_y))
            heading = math.atan2(local_y, local_x)
            candidates.insert(
                0,
                Candidate(
                    "nearest_clear_patch",
                    speed * local_x / distance,
                    lateral * local_y / distance,
                    self._clamp(heading, -yaw, yaw),
                    "LOCAL_SAFE_SPACE",
                ),
            )
        results = [self._score_safe_space_candidate(candidate, pose, danger_points) for candidate in candidates]
        patch_result = next(
            (result for result in results if result.candidate.name == "nearest_clear_patch" and not result.rejected),
            None,
        )
        if patch_result is not None:
            return patch_result, results
        feasible = [result for result in results if not result.rejected]
        if feasible:
            return min(feasible, key=lambda result: result.score), results
        return None, results

    def _danger_zone_points(
        self,
        tracks: list[HumanTrack],
        tentative_points: list[tuple[float, float, float]],
        predicted_points: list[tuple[float, float, float]],
        predictions: dict[int, list[PredictionSample]],
        prediction_age: float,
    ) -> list[tuple[float, float, float]]:
        points = [(x, y, self.predicted_danger_radius) for x, y, _ in predicted_points]
        points.extend((x, y, self.tentative_caution_radius) for x, y, _ in tentative_points)
        for samples in predictions.values():
            points.extend(
                (sample.x, sample.y, self.predicted_danger_radius)
                for sample in samples
                if prediction_age <= sample.time_offset <= prediction_age + self.horizon
            )
        steps = max(1, int(self.horizon / max(0.05, self.dt)))
        for track in tracks:
            if track.track_id in predictions:
                continue
            base_radius = self._comfort_radius(track, 0.0)
            for step in range(steps + 1):
                t = step * self.dt
                radius = min(self.local_safe_radius, base_radius + self.uncertainty_growth_rate * t)
                points.append((track.x + track.vx * t, track.y + track.vy * t, radius))
        return points

    def _nearest_clear_patch(
        self,
        pose: tuple[float, float, float],
        danger_points: list[tuple[float, float, float]],
    ) -> tuple[float, float] | None:
        static_points = []
        now = self.get_clock().now()
        scan_time = None if self.latest_scan_time is None else self.latest_scan_time.nanoseconds * 1e-9
        if self.latest_scan is None or not scan_is_fresh(scan_time, now.nanoseconds * 1e-9, self.max_scan_age):
            return None
        for index, distance in enumerate(self.latest_scan.ranges):
            if not math.isfinite(distance) or distance <= self.latest_scan.range_min:
                continue
            angle = pose[2] + self.latest_scan.angle_min + index * self.latest_scan.angle_increment
            static_points.append((pose[0] + distance * math.cos(angle), pose[1] + distance * math.sin(angle)))
        raw_x = float(self.latest_cmd.linear.x)
        raw_y = float(self.latest_cmd.linear.y)
        preferred_heading = pose[2]
        if math.hypot(raw_x, raw_y) > 1e-3:
            preferred_heading += math.atan2(raw_y, raw_x)
        return nearest_clear_patch(
            (pose[0], pose[1]),
            danger_points,
            static_points,
            self.safe_patch_search_radius,
            self.safe_patch_spacing,
            self.safe_patch_clearance,
            preferred_heading,
        )

    def _local_danger_present(self, pose: tuple[float, float, float], danger_points: list[tuple[float, float, float]]) -> bool:
        if not danger_points:
            return False
        robot_x, robot_y, _ = pose
        for x, y, radius in danger_points:
            if math.hypot(robot_x - x, robot_y - y) <= max(self.local_danger_radius, radius):
                return True
        return False

    def _score_safe_space_candidate(
        self,
        candidate: Candidate,
        pose: tuple[float, float, float],
        danger_points: list[tuple[float, float, float]],
    ) -> CandidateResult:
        trajectory = self._simulate(pose, candidate)
        min_clearance = math.inf
        final_clearance = -math.inf
        initial_clearance = -math.inf
        first_safe_time = math.inf
        path_length = 0.0
        last_xy = None
        for x, y, _, t in trajectory:
            if last_xy is not None:
                path_length += math.hypot(x - last_xy[0], y - last_xy[1])
            last_xy = (x, y)
            clearance = min(math.hypot(x - px, y - py) - radius for px, py, radius in danger_points)
            if not math.isfinite(initial_clearance):
                initial_clearance = clearance
            min_clearance = min(min_clearance, clearance)
            final_clearance = clearance
            if not math.isfinite(first_safe_time) and clearance >= self.local_safe_radius:
                first_safe_time = t
        static_cost, static_reject = self._static_obstacle_cost(trajectory)
        if static_reject:
            return CandidateResult(candidate, math.inf, True, "static_obstacle", trajectory, max(0.0, min_clearance), math.inf, 0.0, {})
        if not math.isfinite(first_safe_time):
            if candidate.name == "nearest_clear_patch" and final_clearance >= initial_clearance + 0.10:
                score_terms = {
                    "clearance_progress": -(final_clearance - initial_clearance),
                    "static": self.weights["w_static"] * static_cost,
                    "path_length": 0.35 * path_length,
                }
                return CandidateResult(
                    candidate,
                    sum(score_terms.values()),
                    False,
                    "",
                    trajectory,
                    max(0.0, min_clearance),
                    math.inf,
                    0.0,
                    score_terms,
                )
            speed_cost = 0.15 * (abs(candidate.vx) + abs(candidate.vy))
            yaw_cost = 0.08 * abs(candidate.wz)
            score = 100.0 - final_clearance + 0.25 * path_length + speed_cost + yaw_cost
            return CandidateResult(
                candidate,
                score,
                True,
                "no_safe_space_reached",
                trajectory,
                final_clearance,
                math.inf,
                0.0,
                {"final_clearance": final_clearance, "min_clearance": min_clearance},
            )
        speed_cost = 0.15 * (abs(candidate.vx) + abs(candidate.vy))
        yaw_cost = 0.08 * abs(candidate.wz)
        score_terms = {
            "time_to_safe": first_safe_time,
            "path_length": 0.35 * path_length,
            "static": self.weights["w_static"] * static_cost,
            "speed": speed_cost,
            "yaw": yaw_cost,
            "clearance": -0.25 * min_clearance,
        }
        score = sum(score_terms.values())
        return CandidateResult(candidate, score, False, "", trajectory, max(0.0, min_clearance), math.inf, 0.0, score_terms)

    def _score_candidate(
        self,
        candidate: Candidate,
        pose: tuple[float, float, float],
        tracks: list[HumanTrack],
        tentative_points: list[tuple[float, float, float]],
        predicted_points: list[tuple[float, float, float]],
        predictions: dict[int, list[PredictionSample]],
        prediction_age: float,
    ) -> CandidateResult:
        trajectory = self._simulate(pose, candidate)
        min_human_distance = math.inf
        min_tcpa = math.inf
        personal_space_cost = 0.0
        ttc_cost = 0.0
        front_cut_cost = 0.0
        yield_cost = 0.0
        margin_violation = False
        body_collision_time = math.inf
        terminal_conflict = 0.0
        robot_points = np.asarray(trajectory)
        times = robot_points[:, 3]
        time_key = tuple(times)
        for track in tracks:
            track_hard_radius = self.robot_radius + track.body_radius + self.hard_safety_margin
            body_distance = self.robot_radius + track.body_radius + getattr(self, "body_prediction_margin", .05)
            initial_distance = math.hypot(pose[0] - track.x, pose[1] - track.y)
            samples = predictions.get(track.track_id)
            cache_key = (track.track_id, time_key)
            if cache_key not in self.prediction_cache:
                values = []
                for t in times:
                    sample = interpolate_prediction(samples, prediction_age + t) if samples else None
                    values.append((track.x + track.vx * t, track.y + track.vy * t, track.vx, track.vy, self._comfort_radius(track, t)) if sample is None else
                                  (sample.x, sample.y, sample.vx, sample.vy, self._prediction_comfort_radius(track, sample)))
                human = np.asarray(values)
                angles = np.arctan2(human[:, 3], human[:, 2])[:, None] + np.asarray((-1.0, -0.5, 0.0, 0.5, 1.0)) * self.front_arc_angle
                arc_x = human[:, 0, None] + max(0.4, self.front_arc_distance) * np.cos(angles)
                arc_y = human[:, 1, None] + max(0.4, self.front_arc_distance) * np.sin(angles)
                self.prediction_cache[cache_key] = human, arc_x, arc_y
            human, arc_x, arc_y = self.prediction_cache[cache_key]
            distances = np.hypot(robot_points[:, 0] - human[:, 0], robot_points[:, 1] - human[:, 1])
            relevant = times <= 1.5 if track.state == "TENTATIVE" else np.ones(len(times), dtype=bool)
            min_human_distance = min(min_human_distance, float(np.min(distances[relevant])))
            required_distance = max(body_distance, min(track_hard_radius, initial_distance - 0.02))
            hard_window = relevant & (times <= self.hard_collision_horizon)
            hits = hard_window & (distances < body_distance)
            if np.any(hits):
                body_collision_time = min(body_collision_time, float(times[np.flatnonzero(hits)[0]]))
            margin_violation = margin_violation or bool(np.any(hard_window & (distances < required_distance)))
            # Swept relative segments avoid missing a crossing between samples.
            relative = robot_points[:, :2] - human[:, :2]
            delta = np.diff(relative, axis=0)
            fraction = np.clip(-np.sum(relative[:-1] * delta, axis=1) / np.maximum(1e-12, np.sum(delta * delta, axis=1)), 0.0, 1.0)
            swept = np.linalg.norm(relative[:-1] + fraction[:, None] * delta, axis=1)
            swept_times = times[:-1] + fraction * np.diff(times)
            swept_relevant = swept_times <= (1.5 if track.state == "TENTATIVE" else self.horizon)
            if np.any(swept_relevant):
                min_human_distance = min(min_human_distance, float(np.min(swept[swept_relevant])))
            swept_window = swept_relevant & (swept_times <= self.hard_collision_horizon)
            swept_hits = swept_window & (swept < body_distance)
            if np.any(swept_hits):
                body_collision_time = min(body_collision_time, float(np.min(swept_times[swept_hits])))
            margin_violation = margin_violation or bool(np.any(swept_window & (swept < required_distance)))
            weights = self.dt * np.exp(-self.social_cost_decay * times) * relevant
            if track.state == "TENTATIVE":
                weights *= track.confidence
            personal_space_cost += float(np.sum(weights * np.maximum(0.0, np.exp(-0.5 * (distances / np.maximum(0.1, human[:, 4])) ** 2) - math.exp(-0.5))))
            if self.front_arc_risk_enabled:
                arc_distances = np.hypot(robot_points[:, 0, None] - arc_x, robot_points[:, 1, None] - arc_y)
                arc_cost = self.front_arc_weight * np.sum(np.exp(-0.5 * (arc_distances / np.maximum(0.1, human[:, 4, None] * 0.55)) ** 2), axis=1)
                personal_space_cost += float(np.sum(weights * arc_cost * (np.hypot(human[:, 2], human[:, 3]) >= 0.08)))
            future = relevant & (times > self.hard_collision_horizon)
            personal_space_cost += float(np.sum(weights[future] * 2.0 * np.maximum(0.0, track_hard_radius - distances[future]) ** 2))
            metrics = self._relative_metrics_for_candidate(pose, candidate, track)
            if math.hypot(track.vx, track.vy) > 0.12 and len(trajectory) > 1:
                end, before = trajectory[-1], trajectory[-2]
                interval = max(1e-6, end[3] - before[3])
                terminal = relative_motion_metrics(end[0], end[1], (end[0] - before[0]) / interval, (end[1] - before[1]) / interval,
                                                   human[-1, 0], human[-1, 1], human[-1, 2], human[-1, 3])
                terminal_horizon = 2.0 * self.horizon if candidate.vx <= 0.0 and abs(candidate.vy) < 0.1 else 2.0
                if terminal["closing"] and terminal["dcpa"] < self.robot_radius + track.body_radius + 0.05 and terminal["tcpa"] < terminal_horizon:
                    terminal_conflict += 50.0
            min_tcpa = min(min_tcpa, metrics["tcpa"])
            if metrics["dcpa"] < self.dcpa_comfort_threshold and metrics["tcpa"] < self.ttc_warning_threshold:
                ttc_cost += ((self.ttc_warning_threshold - metrics["tcpa"]) / self.ttc_warning_threshold) ** 2
            if metrics["front_cut"]:
                front_cut_cost += math.exp(-metrics["dcpa"] / max(0.1, self.dcpa_comfort_threshold))
            if self.state == "YIELD_TO_HUMAN" and candidate.vx > 0.03:
                yield_cost += 1.0
        for tx, ty, _ in tentative_points:
            for x, y, _, _ in trajectory[:4]:
                if math.hypot(x - tx, y - ty) < self.tentative_caution_radius:
                    personal_space_cost += 0.35
        for px, py, _ in predicted_points:
            for x, y, _, t in trajectory:
                distance = math.hypot(x - px, y - py)
                hard_radius = self.robot_radius + self.human_body_radius + self.hard_safety_margin
                if distance < hard_radius:
                    return CandidateResult(candidate, math.inf, True, "predicted_danger_zone", trajectory, distance, min_tcpa, front_cut_cost, {})
                if distance < self.predicted_danger_radius:
                    personal_space_cost += math.exp(-0.5 * (distance / max(0.1, self.predicted_danger_radius)) ** 2) * (1.0 + 0.2 * t)
        if math.isfinite(body_collision_time):
            return CandidateResult(candidate, math.inf, True, "hard_human_collision", trajectory, min_human_distance, body_collision_time, front_cut_cost, {})
        static_cost, static_reject = self._static_obstacle_cost(trajectory)
        if static_reject:
            return CandidateResult(candidate, math.inf, True, "static_obstacle", trajectory, min_human_distance, min_tcpa, front_cut_cost, {})
        progress = self._goal_progress_cost(candidate)
        if self.nav2_path_reference and self.reference_goal_world is not None:
            endpoint = min(trajectory, key=lambda point: abs(point[3] - self.command_duration))
            gx, gy = self.reference_goal_world
            progress = (math.hypot(endpoint[0] - gx, endpoint[1] - gy) - math.hypot(pose[0] - gx, pose[1] - gy)) / max(0.1, endpoint[3])
        path = self._path_alignment_cost(candidate)
        smooth = (candidate.vx - float(self.last_cmd.linear.x)) ** 2 + (candidate.vy - float(self.last_cmd.linear.y)) ** 2
        jerk = smooth + 0.25 * (candidate.wz - float(self.last_cmd.angular.z)) ** 2
        legibility = self._side_switch_cost(candidate)
        deadlock = 1.0 if self.state == "DEADLOCK_RECOVERY" and candidate.name == "yield_stop" else 0.0
        score_terms = {
            "goal": self.weights["w_goal"] * progress,
            "path": self.weights["w_path"] * path,
            "static": self.weights["w_static"] * static_cost,
            "personal_space": self.weights["w_personal_space"] * personal_space_cost,
            "ttc": self.weights["w_ttc"] * ttc_cost,
            "pass_front": self.weights["w_pass_front"] * front_cut_cost,
            "yield": self.weights["w_yield"] * yield_cost,
            "smooth": self.weights["w_smooth"] * smooth,
            "jerk": self.weights["w_jerk"] * jerk,
            "legibility": self.weights["w_legibility"] * legibility,
            "deadlock": self.weights["w_deadlock"] * deadlock,
            "terminal_conflict": self.weights["w_deadlock"] * terminal_conflict,
        }
        score = (
            score_terms["goal"]
            + score_terms["path"]
            + score_terms["static"]
            + score_terms["personal_space"]
            + score_terms["ttc"]
            + score_terms["pass_front"]
            + score_terms["yield"]
            + score_terms["smooth"]
            + score_terms["jerk"]
            + score_terms["legibility"]
            + score_terms["deadlock"]
            + score_terms["terminal_conflict"]
        )
        reason = "terminal_conflict" if terminal_conflict else ("human_margin" if margin_violation else "")
        return CandidateResult(candidate, score, bool(reason), reason, trajectory, min_human_distance, min_tcpa, front_cut_cost, score_terms)

    def _simulate(self, pose: tuple[float, float, float], candidate: Candidate) -> list[tuple[float, float, float, float]]:
        next_command = None if candidate.follow_vx is None else (candidate.follow_vx, 0.0, 0.0)
        duration = math.inf if next_command is None else candidate.switch_time
        filtered = self.policy_command if self.policy_command_stamp is not None and 0.0 <= self.get_clock().now().nanoseconds * 1e-9 - self.policy_command_stamp <= 0.4 else None
        return self.motion_envelope.rollout(pose, (candidate.vx, candidate.vy, candidate.wz), self.measured_velocity, self.horizon, self.dt, self.response_delay, duration, next_command, filtered)

    def _static_obstacle_cost(self, trajectory: list[tuple[float, float, float, float]]) -> tuple[float, bool]:
        if self.static_frame_valid is False:
            return math.inf, True
        # Nav2 replans curved static routes continuously. Check a local command
        # interval plus braking, rather than an unchanged five-second arc that
        # Nav2 never requested. Human occupancy remains checked over the horizon.
        braking = max(abs(self.measured_velocity[0]) / max(0.01, self.motion_envelope.accel_x), abs(self.measured_velocity[1]) / max(0.01, self.motion_envelope.accel_y))
        static_horizon = max(self.command_duration, self.response_delay + braking + 0.3)
        trajectory = [point for point in trajectory if point[3] <= static_horizon]
        now = self.get_clock().now()
        scan_time = None if self.latest_scan_time is None else self.latest_scan_time.nanoseconds * 1e-9
        if self.latest_scan is None or not scan_is_fresh(scan_time, now.nanoseconds * 1e-9, self.max_scan_age):
            self.static_frame_valid = False
            moving = any(math.hypot(point[0] - trajectory[0][0], point[1] - trajectory[0][1]) > 1e-4 for point in trajectory[1:])
            return (math.inf, True) if moving else (0.0, False)
        key = tuple(trajectory)
        if key in self.static_cache:
            return self.static_cache[key]
        if self.use_static_scene_map and self.static_points_cache is None:
            if self.static_scene_points is None:
                self.static_frame_valid = False
                return math.inf, True
            extent = self.static_influence_range + self.horizon * self.motion_envelope.max_x
            nearby = np.linalg.norm(self.static_scene_points - trajectory[0][:2], axis=1) < extent
            self.static_points_cache = self.static_scene_points[nearby]
            self.static_frame_valid = True
        if self.static_points_cache is None:
            ranges = np.asarray(self.latest_scan.ranges, dtype=float)
            angles = self.latest_scan.angle_min + np.arange(len(ranges)) * self.latest_scan.angle_increment
            valid = np.isfinite(ranges) & (ranges > self.latest_scan.range_min) & (ranges < self.static_influence_range + self.horizon * self.motion_envelope.max_x)
            points = np.column_stack((ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid])))
            stamp = _ros_time_seconds(self.latest_scan.header.stamp)
            scan_pose = self.odom_pose_history.get(stamp) if self.latest_scan.header.frame_id == self.robot_frame else None
            if scan_pose is None and self.latest_scan.header.frame_id == self.robot_frame and self.odom_pose_history:
                nearest_stamp = min(self.odom_pose_history, key=lambda value: abs(value - stamp))
                offset = stamp - nearest_stamp
                if abs(offset) <= 0.15:
                    x, y, yaw = self.odom_pose_history[nearest_stamp]
                    vx, vy, wz = self.measured_velocity
                    scan_pose = (x + (math.cos(yaw) * vx - math.sin(yaw) * vy) * offset,
                                 y + (math.sin(yaw) * vx + math.cos(yaw) * vy) * offset, yaw + wz * offset)
            if scan_pose is None:
                try:
                    transform = self.tf_buffer.lookup_transform(self.target_frame, self.latest_scan.header.frame_id, rclpy.time.Time.from_msg(self.latest_scan.header.stamp))
                    q = transform.transform.rotation
                    scan_pose = (transform.transform.translation.x, transform.transform.translation.y,
                                 math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
                except TransformException as exc:
                    self.static_error = str(exc)
                    if self.static_scan_stamp is None or now.nanoseconds * 1e-9 - self.static_scan_stamp > self.max_scan_age:
                        self.static_frame_valid = False
                        return math.inf, True
            if scan_pose is not None:
                self.static_error = ""
                yaw = scan_pose[2]
                c, s = math.cos(yaw), math.sin(yaw)
                points = points @ np.asarray(((c, s), (-s, c)))
                points += scan_pose[:2]
                if self.static_scan_stamp != stamp:
                    keys = list(self.static_memory)
                    if keys:
                        old_points = np.asarray([self.static_memory[key][:2] for key in keys])
                        cleared = observed_free_mask(old_points, scan_pose, ranges, self.latest_scan.angle_min, self.latest_scan.angle_increment)
                        for key, free in zip(keys, cleared):
                            if free:
                                del self.static_memory[key]
                    for x, y in points:
                        if not any(math.hypot(x - track.x, y - track.y) <= track.body_radius + 0.10 for track in self.tracks.values()):
                            self.static_memory[(round(x / 0.10), round(y / 0.10))] = (x, y, stamp)
                    self.static_scan_stamp = stamp
                    self.static_memory = {key: value for key, value in self.static_memory.items() if 0.0 <= stamp - value[2] <= self.static_memory_seconds
                        and not any(math.hypot(value[0] - track.x, value[1] - track.y) <= track.body_radius + 0.10 for track in self.tracks.values())}
            # Maintain recently observed low obstacles when they move below the
            # sensor's vertical field of view. All points are in world, not the
            # offset/tilted LiDAR frame previously mistaken for base_footprint.
            origin_x, origin_y = trajectory[0][:2]
            remembered = list(self.static_memory.values()) + list(self.low_obstacle_memory.values())
            self.static_points_cache = np.asarray([(x, y) for x, y, _ in remembered
                if math.hypot(x - origin_x, y - origin_y) < self.static_influence_range + self.horizon * self.motion_envelope.max_x]).reshape((-1, 2))
            self.static_frame_valid = True
        obstacles = self.static_points_cache
        if len(obstacles) == 0:
            result = (0.0, False)
        else:
            positions = np.asarray(trajectory)[:, :2]
            if self.static_field_cache is None:
                self.static_field_cache = StaticDistanceField(obstacles, trajectory[0][:2], self.static_influence_range + self.horizon * self.motion_envelope.max_x, self.static_influence_range)
            nearest = self.static_field_cache.query(positions)
            rejected = bool(np.any(nearest < self.static_clearance))
            cost = float(np.sum(np.maximum(0.0, (self.static_influence_range - nearest) / self.static_influence_range) ** 2)) * self.dt
            result = (math.inf, True) if rejected else (cost, False)
        self.static_cache[key] = result
        return result

    def _relative_metrics_for_raw_cmd(self, pose: tuple[float, float, float], track: HumanTrack) -> dict[str, float | bool]:
        return self._relative_metrics_for_candidate(
            pose,
            Candidate("raw", float(self.latest_cmd.linear.x), float(self.latest_cmd.linear.y), float(self.latest_cmd.angular.z)),
            track,
        )

    def _relative_metrics_for_candidate(self, pose: tuple[float, float, float], candidate: Candidate, track: HumanTrack) -> dict[str, float | bool]:
        robot_x, robot_y, yaw = pose
        rvx = math.cos(yaw) * candidate.vx - math.sin(yaw) * candidate.vy
        rvy = math.sin(yaw) * candidate.vx + math.cos(yaw) * candidate.vy
        metrics = relative_motion_metrics(robot_x, robot_y, rvx, rvy, track.x, track.y, track.vx, track.vy)
        tcpa = float(metrics["tcpa"])
        dcpa = float(metrics["dcpa"])
        human_speed = math.hypot(track.vx, track.vy)
        robot_speed = math.hypot(rvx, rvy)
        crossing = False
        front_cut = False
        if bool(metrics["closing"]) and human_speed > 0.08 and robot_speed > 0.03:
            dot = (track.vx * rvx + track.vy * rvy) / max(1e-6, human_speed * robot_speed)
            angle = abs(math.acos(max(-1.0, min(1.0, dot))))
            crossing = self.crossing_angle_min < angle < math.pi - self.crossing_angle_min
            human_future_x = track.x + track.vx * tcpa
            human_future_y = track.y + track.vy * tcpa
            robot_future_x = robot_x + rvx * tcpa
            robot_future_y = robot_y + rvy * tcpa
            ahead = ((robot_future_x - human_future_x) * track.vx + (robot_future_y - human_future_y) * track.vy) / human_speed
            front_cut = crossing and ahead > 0.0 and dcpa < self.dcpa_comfort_threshold
        return {**metrics, "front_cut": front_cut}

    def _comfort_radius(self, track: HumanTrack, t: float) -> float:
        uncertainty = math.sqrt(max(0.0, track.cov_xx + track.cov_yy))
        uncertainty += self.uncertainty_growth_rate * t
        uncertainty += self.low_conf_uncertainty_gain * max(0.0, 1.0 - track.confidence)
        uncertainty += 0.08 * max(0, track.missed_count)
        if track.state == "OCCLUDED":
            uncertainty += self.occluded_risk_scale
        return self.comfort_radius + uncertainty

    def _prediction_comfort_radius(self, track: HumanTrack, sample: PredictionSample) -> float:
        uncertainty = math.sqrt(max(0.0, sample.cov_xx + sample.cov_yy))
        uncertainty += self.low_conf_uncertainty_gain * max(0.0, 1.0 - sample.confidence)
        uncertainty += 0.08 * max(0, track.missed_count)
        if track.state == "OCCLUDED":
            uncertainty += self.occluded_risk_scale
        return self.comfort_radius + uncertainty

    def _front_arc_risk_cost(self, robot_x: float, robot_y: float, track: HumanTrack, t: float, radius: float) -> float:
        return self._front_arc_risk_at(
            robot_x,
            robot_y,
            track.x + track.vx * t,
            track.y + track.vy * t,
            track.vx,
            track.vy,
            radius,
        )

    def _front_arc_risk_at(
        self, robot_x: float, robot_y: float, hx: float, hy: float,
        vx: float, vy: float, radius: float,
    ) -> float:
        if not self.front_arc_risk_enabled:
            return 0.0
        speed = math.hypot(vx, vy)
        if speed < 0.08:
            return 0.0
        heading = math.atan2(vy, vx)
        arc_radius = max(0.4, self.front_arc_distance)
        cost = 0.0
        for fraction in (-1.0, -0.5, 0.0, 0.5, 1.0):
            angle = heading + fraction * self.front_arc_angle
            ax = hx + arc_radius * math.cos(angle)
            ay = hy + arc_radius * math.sin(angle)
            distance = math.hypot(robot_x - ax, robot_y - ay)
            cost += self.front_arc_weight * math.exp(-0.5 * (distance / max(0.1, radius * 0.55)) ** 2)
        return cost

    def _goal_progress_cost(self, candidate: Candidate) -> float:
        raw_x = float(self.latest_cmd.linear.x)
        raw_y = float(self.latest_cmd.linear.y)
        norm = math.hypot(raw_x, raw_y)
        if norm < 1e-3:
            return 0.0
        progress = (candidate.vx * raw_x + candidate.vy * raw_y) / norm
        return -progress

    def _path_alignment_cost(self, candidate: Candidate) -> float:
        if (getattr(self, "prefer_forward_travel", False) and
                math.hypot(self.latest_cmd.linear.x, self.latest_cmd.linear.y) < .03):
            # A turn request has no translational direction. atan2(0, 0) used
            # to penalize a necessary reverse/side escape as a 180-degree path
            # violation, trapping the robot against room boundaries.
            return abs(candidate.wz-self.latest_cmd.angular.z) + .2*abs(candidate.vy)
        raw_angle = math.atan2(float(self.latest_cmd.linear.y), float(self.latest_cmd.linear.x))
        cand_angle = math.atan2(candidate.vy, candidate.vx) if math.hypot(candidate.vx, candidate.vy) > 1e-3 else raw_angle
        return abs(self._wrap_pi(cand_angle - raw_angle)) + abs(candidate.wz - float(self.latest_cmd.angular.z))

    def _side_switch_cost(self, candidate: Candidate) -> float:
        side = candidate.vy if abs(candidate.vy) > 1e-3 else candidate.wz
        if self.side_commitment == 0.0 or abs(side) < 1e-3:
            return 0.0
        return 1.0 if side * self.side_commitment < 0.0 else 0.0

    def _human_blocks_forward_path(self, pose: tuple[float, float, float], tracks: list[HumanTrack]) -> bool:
        robot_x, robot_y, yaw = pose
        for track in tracks:
            dx = track.x - robot_x
            dy = track.y - robot_y
            local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
            local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            if 0.0 < local_x < self.comfort_radius * 1.4 and abs(local_y) < self.comfort_radius * 0.55:
                return True
        return False

    def _preferred_side(self, pose: tuple[float, float, float], tracks: list[HumanTrack]) -> float:
        if not tracks:
            return 0.0
        robot_x, robot_y, yaw = pose
        nearest = min(tracks, key=lambda track: math.hypot(track.x - robot_x, track.y - robot_y))
        dx = nearest.x - robot_x
        dy = nearest.y - robot_y
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return -1.0 if local_y >= 0.0 else 1.0

    def _publish(self, cmd: Twist, state: str, results: list[CandidateResult], selected: CandidateResult | None) -> None:
        publishing_started = time.perf_counter()
        now = self.get_clock().now()
        tracks = self._fresh_tracks(now)
        closest_track = self._closest_track_debug(tracks)
        state_msg = String()
        state_msg.data = state
        self.state_pub.publish(state_msg)
        if self.selected_safe_patch is not None:
            patch_msg = PoseStamped()
            patch_msg.header.frame_id = self.target_frame
            patch_msg.header.stamp = now.to_msg()
            patch_msg.pose.position.x = self.selected_safe_patch[0]
            patch_msg.pose.position.y = self.selected_safe_patch[1]
            patch_msg.pose.orientation.w = 1.0
            self.safe_patch_pub.publish(patch_msg)
        self.cmd_pub.publish(cmd)
        self.planning_phases["control_publish_ms"] = (time.perf_counter() - publishing_started) * 1000.0
        debug_pose = self._robot_pose()
        debug = {
            "static_error": self.static_error,
            "static_memory_points": len(self.static_memory),
            "low_obstacle_points": len(self.low_obstacle_memory),
            "nearest_static": sorted(self.static_memory.values(), key=lambda p: math.hypot(p[0] - debug_pose[0], p[1] - debug_pose[1]))[:3] if self.static_memory and debug_pose is not None else [],
            "planning_ms": round((time.perf_counter() - self.planning_started) * 1000.0, 3),
            "search_budget_exhausted": self.search_budget_exhausted,
            "evaluated_candidates": len(results),
            "planning_thread_ms": round((time.thread_time() - self.planning_cpu_started) * 1000.0, 3),
            "planning_phases_ms": {key: round(value, 3) for key, value in self.planning_phases.items()},
            "tracks_age_s": None if self.tracks_epoch is None else round(now.nanoseconds * 1e-9 - self.tracks_epoch, 4),
            "prediction_age_s": None if self.prediction_epoch is None else round(now.nanoseconds * 1e-9 - self.prediction_epoch, 4),
            "measured_velocity": list(self.measured_velocity),
            "policy_command": self.policy_command,
            "state": state,
            "state_age_s": round((now - self.state_since).nanoseconds * 1e-9, 3),
            "selected": selected.candidate.name if selected else "none",
            "selected_reason": selected.reason if selected else "no_feasible_candidate",
            "raw_cmd": self._twist_dict(self.upstream_cmd if self.nav2_path_reference else self.latest_cmd),
            "reference_cmd": self._twist_dict(self.latest_cmd),
            "path_reference_status": self.path_reference_status,
            "mission_active": self.mission_active,
            "selected_cmd": self._twist_dict(cmd),
            "score": round(selected.score, 3) if selected and math.isfinite(selected.score) else None,
            "rejected": sum(1 for result in results if result.rejected),
            "min_human_distance": round(selected.min_human_distance, 3) if selected and math.isfinite(selected.min_human_distance) else None,
            "min_tcpa": round(selected.min_tcpa, 3) if selected and math.isfinite(selected.min_tcpa) else None,
            "side_commitment": self.side_commitment,
            "input_age_s": self._age_seconds(self.latest_cmd_time, now),
            "scan_age_s": self._age_seconds(self.latest_scan_time, now),
            "track_count": len(tracks),
            "tentative_count": len(self._fresh_tentative(now)),
            "closest_track": closest_track,
            "candidate_results": [self._candidate_result_debug(result) for result in results],
        }
        debug_msg = String()
        debug_msg.data = json.dumps(debug, separators=(",", ":"), sort_keys=True)
        self.debug_pub.publish(debug_msg)
        if time.perf_counter() - self.last_visualization >= self.visualization_period:
            self.last_visualization = time.perf_counter()
            if self.marker_pub.get_subscription_count():
                self.marker_pub.publish(self._markers(results, selected, state))
            if self.publish_risk_grid and self.grid_pub.get_subscription_count():
                self.grid_pub.publish(self._risk_grid())

    @staticmethod
    def _twist_dict(msg: Twist) -> dict[str, float]:
        return {
            "x": round(float(msg.linear.x), 4),
            "y": round(float(msg.linear.y), 4),
            "z": round(float(msg.angular.z), 4),
        }

    @staticmethod
    def _age_seconds(stamp, now) -> float | None:
        if stamp is None:
            return None
        return round((now - stamp).nanoseconds * 1e-9, 3)

    def _closest_track_debug(self, tracks: list[HumanTrack]) -> dict[str, float | int | str] | None:
        pose = self._robot_pose()
        if pose is None or not tracks:
            return None
        robot_x, robot_y, _ = pose
        track = min(tracks, key=lambda item: math.hypot(item.x - robot_x, item.y - robot_y))
        metrics = self._relative_metrics_for_raw_cmd(pose, track)
        return {
            "track_id": track.track_id,
            "state": track.state,
            "x": round(track.x, 3),
            "y": round(track.y, 3),
            "vx": round(track.vx, 3),
            "vy": round(track.vy, 3),
            "confidence": round(track.confidence, 3),
            "person_score": round(track.person_score, 3),
            "distance": round(float(metrics["distance"]), 3),
            "tcpa": round(float(metrics["tcpa"]), 3) if math.isfinite(float(metrics["tcpa"])) else None,
            "dcpa": round(float(metrics["dcpa"]), 3),
            "front_cut": bool(metrics["front_cut"]),
        }

    def _candidate_result_debug(self, result: CandidateResult) -> dict[str, object]:
        data = {
            "name": result.candidate.name,
            "cmd": {"x": round(result.candidate.vx, 4), "y": round(result.candidate.vy, 4), "z": round(result.candidate.wz, 4)},
            "score": round(result.score, 3) if math.isfinite(result.score) else None,
            "rejected": result.rejected,
            "reason": result.reason,
            "min_human_distance": round(result.min_human_distance, 3) if math.isfinite(result.min_human_distance) else None,
            "min_tcpa": round(result.min_tcpa, 3) if math.isfinite(result.min_tcpa) else None,
            "front_cut_cost": round(result.front_cut_cost, 3),
        }
        if result.score_terms:
            data["score_terms"] = {key: round(value, 3) for key, value in result.score_terms.items()}
        return data

    def _markers(self, results: list[CandidateResult], selected: CandidateResult | None, state: str) -> MarkerArray:
        markers = MarkerArray()
        now = rclpy.time.Time().to_msg()  # Display with latest TF, not a future ego transform.
        alternatives = [result for result in results if result is not selected][:12]
        for index, result in enumerate(alternatives):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = now
            marker.ns = "human_aware_candidates"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.
            marker.scale.x = 0.015
            if result.rejected:
                marker.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.18)
            else:
                marker.color = ColorRGBA(r=0.25, g=0.55, b=1.0, a=0.22)
            for x, y, _, _ in result.trajectory:
                marker.points.append(Point(x=x, y=y, z=0.08))
            marker.lifetime = Duration(seconds=0.4).to_msg()
            markers.markers.append(marker)
        for index in range(len(alternatives), getattr(self, "_last_candidate_marker_count", 0)):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.ns, marker.id, marker.action = "human_aware_candidates", index, Marker.DELETE
            markers.markers.append(marker)
        self._last_candidate_marker_count = len(alternatives)
        chosen = Marker()
        chosen.header.frame_id = self.target_frame
        chosen.ns, chosen.id = "human_aware_selected", 0
        chosen.pose.orientation.w = 1.
        chosen.type = Marker.LINE_STRIP
        chosen.scale.x = .075
        chosen.lifetime = Duration(seconds=.6).to_msg()
        if selected is None:
            chosen.action = Marker.DELETE
        else:
            chosen.action = Marker.ADD
            chosen.color = ColorRGBA(r=1., g=.65, b=.1, a=1.) if selected.rejected else ColorRGBA(r=.1, g=1., b=.35, a=1.)
            chosen.points = [Point(x=x, y=y, z=.12) for x, y, _, _ in selected.trajectory]
        markers.markers.append(chosen)
        label = Marker()
        label.header.frame_id = self.robot_frame
        label.header.stamp = now
        label.ns = "human_aware_state"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.z = 1.85
        label.pose.orientation.w = 1.
        label.frame_locked = True
        label.scale.z = 0.14
        label.color = ColorRGBA(r=1.0, g=0.9, b=0.2, a=1.0)
        reasons = {}
        for result in results:
            if result.rejected:
                reasons[result.reason] = reasons.get(result.reason, 0) + 1
        top = sorted(reasons.items(), key=lambda item: item[1], reverse=True)[:2]
        checks = ",".join(f"{reason}:{count}" for reason, count in top) or "checks:clear"
        label.text = (f"{state}\n"
                      f"Choose:{selected.candidate.name if selected else 'hold'}\n"
                      f"Route:{self.path_reference_status};tracks:{len(self.tracks)}\n{checks}")
        label.lifetime = Duration(seconds=0.5).to_msg()
        markers.markers.append(label)
        if self.selected_safe_patch is not None:
            patch = Marker()
            patch.header.frame_id = self.target_frame
            patch.header.stamp = now
            patch.ns = "nearest_clear_ground_patch"
            patch.id = 1
            patch.type = Marker.CYLINDER
            patch.action = Marker.ADD
            patch.pose.position.x = self.selected_safe_patch[0]
            patch.pose.position.y = self.selected_safe_patch[1]
            patch.pose.position.z = 0.025
            patch.pose.orientation.w = 1.0
            patch.scale.x = self.safe_patch_clearance * 2.0
            patch.scale.y = self.safe_patch_clearance * 2.0
            patch.scale.z = 0.05
            patch.color = ColorRGBA(r=0.05, g=1.0, b=0.25, a=0.65)
            patch.lifetime = Duration(seconds=0.5).to_msg()
            markers.markers.append(patch)
        return markers

    def _risk_grid(self) -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.frame_id = self.target_frame
        grid.header.stamp = self.get_clock().now().to_msg()
        pose = self._robot_pose()
        center_x, center_y = (pose[0], pose[1]) if pose else (0.0, 0.0)
        resolution = max(0.05, self.risk_grid_resolution)
        width = max(1, int(self.risk_grid_size / resolution))
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = width
        grid.info.origin.position.x = center_x - self.risk_grid_size * 0.5
        grid.info.origin.position.y = center_y - self.risk_grid_size * 0.5
        grid.info.origin.orientation.w = 1.0
        tracks = self._fresh_tracks(self.get_clock().now())
        now = self.get_clock().now()
        predictions, prediction_age, typed_predictions_fresh = self._fresh_predictions(now)
        predicted_points = [] if typed_predictions_fresh else self._fresh_predicted(now)
        data = []
        for row in range(width):
            y = grid.info.origin.position.y + (row + 0.5) * resolution
            for col in range(width):
                x = grid.info.origin.position.x + (col + 0.5) * resolution
                risk = 0.0
                for track in tracks:
                    radius = self._comfort_radius(track, 0.0)
                    risk = max(risk, math.exp(-0.5 * (math.hypot(x - track.x, y - track.y) / max(0.1, radius)) ** 2))
                for px, py, _ in predicted_points:
                    risk = max(risk, math.exp(-0.5 * (math.hypot(x - px, y - py) / max(0.1, self.predicted_danger_radius)) ** 2))
                for samples in predictions.values():
                    for sample in samples:
                        if prediction_age <= sample.time_offset <= prediction_age + self.horizon:
                            risk = max(
                                risk,
                                math.exp(-0.5 * (math.hypot(x - sample.x, y - sample.y) / max(0.1, self.predicted_danger_radius)) ** 2),
                            )
                data.append(int(max(0.0, min(1.0, risk)) * 100.0))
        grid.data = data
        return grid

    def _twist_from_candidate(self, candidate: Candidate) -> Twist:
        msg = Twist()
        vx, vy, wz = self.motion_envelope.normalize((candidate.vx, candidate.vy, candidate.wz))
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        return msg

    def _robot_pose(self) -> tuple[float, float, float] | None:
        if self.odom_pose is not None and self.odom_stamp is not None:
            age = self.get_clock().now().nanoseconds * 1e-9 - self.odom_stamp
            if 0.0 <= age <= self.odom_timeout:
                x, y, yaw = self.odom_pose
                vx, vy, wz = self.measured_velocity
                return (x + (math.cos(yaw) * vx - math.sin(yaw) * vy) * age,
                        y + (math.sin(yaw) * vx + math.cos(yaw) * vy) * age, yaw + wz * age)
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.robot_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(f"No transform {self.target_frame} <- {self.robot_frame}: {exc}", throttle_duration_sec=2.0)
            return None
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return transform.transform.translation.x, transform.transform.translation.y, yaw

    def _input_stale(self, now) -> bool:
        if self.latest_cmd_time is None:
            return True
        return (now - self.latest_cmd_time).nanoseconds * 1e-9 > self.input_timeout

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    @staticmethod
    def _wrap_pi(value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value


def main() -> None:
    rclpy.init()
    node = HumanAwareMotionPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
