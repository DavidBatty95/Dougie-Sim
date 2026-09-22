from __future__ import annotations

import json
import math
import random
import time
from pathlib import Path

import yaml
try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String, Bool
    from tf2_ros import Buffer, TransformException, TransformListener
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from visualization_msgs.msg import Marker
except ModuleNotFoundError:  # Allows CI-safe mock benchmarking without a sourced ROS Python environment.
    rclpy = None
    PoseStamped = None
    Twist = None
    Odometry = None
    LaserScan = None
    ActionClient = None
    qos_profile_sensor_data = 10
    String = None
    Bool = None
    Buffer = None
    TransformException = Exception
    TransformListener = None
    NavigateToPose = None
    ComputePathToPose = None
    Marker = None
    Node = object

from .config import BenchmarkConfig
from .geometry import Pose2D, Velocity2D, distance_xy
from .goal_sampler import Goal, GoalSampler
from .observed_space import observed_free
from .source_contract import audit_onboard_graph
from .metrics import MetricsMonitor, SafetyEvent
from .people_manager import PeopleManager
from .person_motion_models import PersonState, update_person
from .result_writer import ResultWriter
from .scoring import ScoreBreakdown, score_benchmark

NAV_STATUS_SUCCEEDED = 4


def sensed_goal_reached(robot_pose, goal_pose, tolerance, nav_success=False, visual_session=False):
    """Visual handoff follows the requested distance, not an action's looser tolerance."""
    return distance_xy(robot_pose, goal_pose) <= tolerance or (nav_success and not visual_session)


class SampleProgress:
    """Tracks whether a source timestamp or sequence continues to advance."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.value: float | None = None
        self.advanced_s: float | None = None
        self.advances = 0
        self.error: str | None = None
        self.max_stall_s = 0.0

    def update(self, value: float, received_s: float) -> None:
        if not math.isfinite(value):
            self.error = f"{self.name}_sample_invalid"
            return
        if self.value is None:
            self.value = value
            self.advanced_s = received_s
            self.advances = 1
        elif value > self.value + 1e-9:
            self.value = value
            self.advanced_s = received_s
            self.advances += 1
        elif value < self.value - 1e-9:
            self.error = f"{self.name}_sample_regressed"

    def health_error(self, timeout_s: float, now_s: float) -> str | None:
        if self.error is not None:
            return self.error
        if self.advanced_s is None:
            return f"{self.name}_sample_missing"
        stall = max(0.0, now_s - self.advanced_s)
        self.max_stall_s = max(self.max_stall_s, stall)
        if self.advances < 2:
            return f"{self.name}_sample_not_advancing"
        if stall > timeout_s:
            return f"{self.name}_sample_stale:{stall:.3f}s>{timeout_s:.3f}s"
        return None


def live_state_health_error(
    *,
    requested_people: int,
    observed_people: int | None,
    odom_age_s: float | None,
    pedestrian_age_s: float | None,
    scan_age_s: float | None,
    scan_source_age_s: float | None,
    scan_source_expected: str,
    scan_source_observed: str | None,
    odom_stale_timeout_s: float,
    pedestrian_stale_timeout_s: float,
    scan_stale_timeout_s: float,
    require_exact_people_count: bool,
) -> str | None:
    """Return a stable invalid-trial reason when required live state is unhealthy."""
    if odom_age_s is None:
        return "odom_missing"
    if odom_age_s > odom_stale_timeout_s:
        return f"odom_stale:{odom_age_s:.3f}s>{odom_stale_timeout_s:.3f}s"
    if scan_age_s is None:
        return "scan_missing"
    if scan_age_s > scan_stale_timeout_s:
        return f"scan_stale:{scan_age_s:.3f}s>{scan_stale_timeout_s:.3f}s"
    if scan_source_age_s is None or scan_source_observed is None:
        return "scan_source_status_missing"
    if scan_source_age_s > scan_stale_timeout_s:
        return f"scan_source_status_stale:{scan_source_age_s:.3f}s>{scan_stale_timeout_s:.3f}s"
    if scan_source_expected not in {"auto", "not_used"} and scan_source_observed != scan_source_expected:
        return f"scan_source_mismatch:expected={scan_source_expected},observed={scan_source_observed}"
    if requested_people > 0:
        if pedestrian_age_s is None or observed_people is None:
            return "pedestrian_truth_missing"
        if pedestrian_age_s > pedestrian_stale_timeout_s:
            return (
                f"pedestrian_truth_stale:{pedestrian_age_s:.3f}s>"
                f"{pedestrian_stale_timeout_s:.3f}s"
            )
    if (
        require_exact_people_count
        and observed_people is not None
        and observed_people != requested_people
    ):
        return f"people_count_mismatch:requested={requested_people},observed={observed_people}"
    return None


class LiveEventEpisodes:
    """Counts transitions into states exposed by the existing safety debug stream."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.emergency_stops = 0
        self.stale_events = 0
        self.avoidance_interventions = 0
        self._emergency_active = False
        self._stale_active = False
        self._override_active = False

    def update(self, payload: dict) -> None:
        if "status" in payload:
            emergency = payload.get("status") == "SAFETY_FILTER_STOPPING"
            if emergency and not self._emergency_active:
                self.emergency_stops += 1
            self._emergency_active = emergency
        if "mode" in payload:
            stale = payload.get("mode") == "stale_input"
            if stale and not self._stale_active:
                self.stale_events += 1
            self._stale_active = stale
            override = payload.get("mode") == "override"
            if override and not self._override_active:
                self.avoidance_interventions += 1
            self._override_active = override

    def pause(self) -> None:
        self._emergency_active = False
        self._stale_active = False
        self._override_active = False


class HumanAvoidanceEpisodes:
    """Tracks planner yield/resume transitions independently of safety overrides."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.yield_episodes = 0
        self.resume_episodes = 0
        self.yield_time_s = 0.0
        self.state: str | None = None
        self._yield_started_s: float | None = None
        self._last_resume_s: float | None = None
        self._awaiting_resume = False

    def update(self, state: str, now_s: float) -> None:
        state = state.strip().upper()
        if not state:
            return
        previous = self.state
        if state == "YIELD_TO_HUMAN" and previous != "YIELD_TO_HUMAN":
            self.yield_episodes += 1
            self._yield_started_s = now_s
            self._awaiting_resume = True
        elif previous == "YIELD_TO_HUMAN" and state != "YIELD_TO_HUMAN":
            if self._yield_started_s is not None:
                self.yield_time_s += max(0.0, now_s - self._yield_started_s)
                self._yield_started_s = None
        if state == "NORMAL_NAVIGATION" and self._awaiting_resume:
            self.resume_episodes += 1
            self._last_resume_s = now_s
            self._awaiting_resume = False
        self.state = state

    def total_yield_time(self, now_s: float) -> float:
        active = 0.0
        if self._yield_started_s is not None:
            active = max(0.0, now_s - self._yield_started_s)
        return self.yield_time_s + active

    def pause(self, now_s: float) -> None:
        if self._yield_started_s is not None:
            self.yield_time_s += max(0.0, now_s - self._yield_started_s)
        self.state = None
        self._yield_started_s = None
        self._last_resume_s = None
        self._awaiting_resume = False

    def display_phase(self, now_s: float) -> str:
        if self.state == "YIELD_TO_HUMAN":
            return "YIELD"
        if self._last_resume_s is not None and now_s - self._last_resume_s <= 2.0:
            return "RESUME"
        return "WALK"


def transform_planar_state(
    pose: Pose2D, velocity: Velocity2D, translation_x: float, translation_y: float, yaw: float
) -> tuple[Pose2D, Velocity2D]:
    """Apply a planar rigid transform to a pose and a world-frame velocity vector."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        Pose2D(
            translation_x + cos_yaw * pose.x - sin_yaw * pose.y,
            translation_y + sin_yaw * pose.x + cos_yaw * pose.y,
            pose.yaw + yaw,
        ),
        Velocity2D(
            cos_yaw * velocity.vx - sin_yaw * velocity.vy,
            sin_yaw * velocity.vx + cos_yaw * velocity.vy,
            velocity.wz,
        ),
    )


def people_from_payload(
    payload: dict, default_radius_m: float, transform: tuple[float, float, float] | None = None
) -> list[PersonState]:
    """Decode simulator truth, optionally transforming it into the scoring frame."""
    people = []
    items = payload.get("people")
    if not isinstance(items, list):
        raise ValueError("people must be a list")
    person_ids: set[int] = set()
    for item in items:
        if not isinstance(item, dict) or not all(key in item for key in ("id", "x", "y", "vx", "vy")):
            raise ValueError("pedestrian entry is missing required fields")
        person_id = int(item["id"])
        values = tuple(float(item[key]) for key in ("x", "y", "vx", "vy"))
        radius = float(item.get("radius_m", default_radius_m))
        if person_id in person_ids or not all(math.isfinite(value) for value in (*values, radius)) or radius <= 0.0:
            raise ValueError("pedestrian entry has duplicate or invalid values")
        person_ids.add(person_id)
        pose = Pose2D(values[0], values[1], 0.0)
        velocity = Velocity2D(values[2], values[3], 0.0)
        if transform is not None:
            pose, velocity = transform_planar_state(pose, velocity, *transform)
        people.append(
            PersonState(
                person_id=person_id,
                pose=pose,
                velocity=velocity,
                radius_m=radius,
                behaviour="mujoco",
                speed_mps=math.hypot(velocity.vx, velocity.vy),
            )
        )
    return people


def velocity_from_pose_delta(previous: Pose2D, current: Pose2D, dt_s: float) -> Velocity2D | None:
    if dt_s <= 1e-3 or dt_s > 0.5:
        return None
    yaw_delta = math.atan2(math.sin(current.yaw - previous.yaw), math.cos(current.yaw - previous.yaw))
    return Velocity2D(
        (current.x - previous.x) / dt_s,
        (current.y - previous.y) / dt_s,
        yaw_delta / dt_s,
    )


class BenchmarkRosNode(Node):
    def __init__(self, config: BenchmarkConfig) -> None:
        self.config = config
        self.ros_enabled = rclpy is not None
        self.live_events = LiveEventEpisodes()
        self.human_avoidance = HumanAvoidanceEpisodes()
        self.odom_frame_error: str | None = None
        self.pedestrian_frame_error: str | None = None
        self.latest_robot_pose: Pose2D | None = None
        self.evaluation_pose = None
        self.evaluation_received_s = None
        self.evaluation_velocity = Velocity2D()
        self.evaluation_stamp = None
        self.observed_map = None
        self.observed_people = None
        self.latest_odom_received_s: float | None = None
        self.latest_robot_velocity = Velocity2D()
        self._last_odom_pose: Pose2D | None = None
        self._last_odom_stamp: float | None = None
        self._derived_robot_velocity = Velocity2D()
        self.latest_people: list[PersonState] = []
        self.latest_people_population: int | None = None
        self.latest_people_stamp = None
        self.latest_people_received_s: float | None = None
        self.latest_scan_received_s: float | None = None
        self.scan_messages_observed = 0
        self.latest_scan_source_received_s: float | None = None
        self.latest_scan_source: str | None = None
        self.latest_control_people_source: str | None = None
        self.latest_control_people_source_received_s: float | None = None
        self.health_monitoring_started = False
        self.latched_health_error: str | None = None
        self.max_odom_age_s = 0.0
        self.max_pedestrian_age_s = 0.0
        self.max_scan_age_s = 0.0
        self.max_control_people_source_age_s = 0.0
        self.odom_progress = SampleProgress("odom")
        self.pedestrian_progress = SampleProgress("pedestrian_truth")
        self.scan_progress = SampleProgress("scan")
        self.scoring_active = False
        self.latest_raw_cmd_stamp: float | None = None
        self.latest_raw_cmd_magnitude = 0.0
        self.max_abs_safe_lateral_cmd_mps = 0.0
        self.latest_locomotion: dict | None = None
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_result_status: int | None = None
        self.nav_goal_success = False
        self.current_goal_index = None
        self.nav_retry_future = None
        self.last_nav_retry_s = -math.inf
        self.mission_active = False
        self.mission_state_pub = None
        if self.ros_enabled:
            super().__init__("person_avoidance_benchmark")
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.people_pub = self.create_publisher(String, "/benchmark/people_states", 10)
            self.metrics_pub = self.create_publisher(String, "/benchmark/metrics", 10)
            self.result_pub = self.create_publisher(String, "/benchmark/result", 10)
            self.goal_pub = self.create_publisher(PoseStamped, "/benchmark/current_goal", 10)
            self.mission_state_pub = self.create_publisher(Bool, "/benchmark/goal_active", 1)
            self.create_timer(0.2, self.publish_mission_state)
            self.oracle_marker_pub = self.create_publisher(Marker, "/benchmark/social_demo_oracle", 10)
            self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
            if config.onboard_only:
                from nav_msgs.msg import OccupancyGrid
                from unitree_social_navigation_msgs.msg import TrackedPersons
                from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
                self.create_subscription(Odometry, "/evaluation/odom", self._evaluation_callback, 10)
                self.create_subscription(TrackedPersons, "/tracked_persons", self._observed_people_callback, qos_profile_sensor_data)
                self.create_subscription(OccupancyGrid, "/map", self._map_callback,
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
            self.create_subscription(String, "/mujoco/pedestrians", self._people_callback, 10)
            self.create_subscription(LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data)
            self.create_subscription(String, "/g1/sim_sensor_status", self._sensor_status_callback, 10)
            self.create_subscription(Twist, "/cmd_vel", self._raw_cmd_callback, 10)
            self.create_subscription(Twist, "/cmd_vel_safe", self._safe_cmd_callback, 10)
            self.create_subscription(String, "/g1/locomotion_diagnostics", self._locomotion_callback, 10)
            self.create_subscription(String, "/human_collision_avoidance_debug", self._safety_debug_callback, 10)
            self.create_subscription(String, "/human_avoidance_state", self._human_avoidance_callback, 10)
            self.create_subscription(String, "/dynamic_obstacle_tracker/debug_tracks", self._tracker_debug_callback, 10)
            self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose") if NavigateToPose is not None else None
            self.path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose") if ComputePathToPose is not None else None

    def publish_mission_state(self) -> None:
        if getattr(self, "mission_state_pub", None) is not None:
            message = Bool()
            message.data = self.mission_active
            self.mission_state_pub.publish(message)

    def _evaluation_callback(self, msg):
        if msg.header.frame_id != "world":
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = Pose2D(p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.evaluation_stamp is not None and stamp > self.evaluation_stamp:
            self.evaluation_velocity = velocity_from_pose_delta(self.evaluation_pose, pose, stamp - self.evaluation_stamp) or Velocity2D()
        self.evaluation_pose, self.evaluation_stamp = pose, stamp
        self.evaluation_received_s = time.monotonic()

    def _map_callback(self, msg):
        if msg.header.frame_id == self.config.map_frame:
            self.observed_map = msg

    def _observed_people_callback(self, msg):
        self.observed_people = msg

    def goal_people(self):
        if not self.config.onboard_only:
            return self.latest_people
        msg = self.observed_people
        if msg is None:
            return []
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not 0 <= self.get_clock().now().nanoseconds * 1e-9 - stamp <= .8:
            return []
        planar = None
        if msg.header.frame_id != self.config.map_frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.config.map_frame, msg.header.frame_id, rclpy.time.Time()).transform
            except TransformException:
                return []
            q = tf.rotation
            planar = (tf.translation.x, tf.translation.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        return people_from_payload({"people": [
            {"id": p.track_id, "x": p.pose.pose.position.x, "y": p.pose.pose.position.y,
             "vx": p.velocity.twist.linear.x, "vy": p.velocity.twist.linear.y, "radius_m": p.body_radius}
            for p in msg.persons if p.confidence >= .35]}, self.config.person_radius_m, planar)

    def publish_people(self, people) -> None:
        if not self.ros_enabled:
            return
        msg = String()
        msg.data = json.dumps([person.as_dict() for person in people])
        self.people_pub.publish(msg)

    def publish_goal(self, goal: Goal) -> None:
        if not self.ros_enabled:
            return
        msg = PoseStamped()
        msg.header.frame_id = self.config.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = goal.pose.x
        msg.pose.position.y = goal.pose.y
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def publish_metrics(self, summary: dict) -> None:
        if not self.ros_enabled:
            return
        msg = String()
        msg.data = json.dumps(summary)
        self.metrics_pub.publish(msg)

    def publish_result(self, score: ScoreBreakdown) -> None:
        if not self.ros_enabled:
            return
        msg = String()
        msg.data = json.dumps(score.as_dict())
        self.result_pub.publish(msg)

    def publish_oracle(self, stage: str, clearance_m: float, now_s: float) -> None:
        if not self.ros_enabled:
            return
        marker = Marker()
        marker.header.frame_id = self.config.robot_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "benchmark_social_demo"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.1
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.28
        phase = self.human_avoidance.display_phase(now_s)
        if phase == "YIELD":
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.65, 0.05
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.1, 1.0, 0.35
        marker.color.a = 1.0
        clearance = "--" if not math.isfinite(clearance_m) else f"{clearance_m:.2f} m"
        marker.text = f"{phase}  |  GOAL {stage}\nCLEARANCE {clearance}  |  ORACLE"
        self.oracle_marker_pub.publish(marker)

    def _odom_callback(self, msg) -> None:
        pose = msg.pose.pose
        yaw = _yaw_from_quat(pose.orientation)
        odom_pose = Pose2D(float(pose.position.x), float(pose.position.y), yaw)
        current_pose = odom_pose
        source_frame = str(getattr(msg.header, "frame_id", ""))
        if source_frame and source_frame != self.config.map_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.config.map_frame, source_frame, rclpy.time.Time()
                ).transform
                transform_yaw = _yaw_from_quat(transform.rotation)
                current_pose, _ = transform_planar_state(
                    current_pose, Velocity2D(), float(transform.translation.x),
                    float(transform.translation.y), transform_yaw
                )
            except TransformException:
                if self.latest_robot_pose is None:
                    self.odom_frame_error = f"transform_unavailable:{source_frame}->{self.config.map_frame}"
                return
        self.odom_frame_error = None
        received_s = time.monotonic()
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp <= 0.0:
            self.odom_frame_error = "odom_sample_stamp_invalid"
            self.odom_progress.error = "odom_sample_stamp_invalid"
            return
        self.odom_progress.update(stamp, received_s)
        reported = Velocity2D(
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.angular.z),
        )
        if self._last_odom_pose is None or self._last_odom_stamp is None:
            self._last_odom_pose = odom_pose
            self._last_odom_stamp = stamp
        elif stamp - self._last_odom_stamp >= 0.25:
            derived = velocity_from_pose_delta(self._last_odom_pose, odom_pose, stamp - self._last_odom_stamp)
            plausible_limit = max(1.0, self.config.max_speed_mps * 2.0)
            if derived is not None and math.hypot(derived.vx, derived.vy) <= plausible_limit:
                self._derived_robot_velocity = derived
            self._last_odom_pose = odom_pose
            self._last_odom_stamp = stamp
        self.latest_robot_velocity = (
            self._derived_robot_velocity
            if math.hypot(reported.vx, reported.vy) < 0.005
            else reported
        )
        self.latest_robot_pose = current_pose
        self.latest_odom_received_s = received_s

    def _people_callback(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        sim_time = data.get("sim_time")
        if type(sim_time) not in (int, float) or not math.isfinite(sim_time):
            self.pedestrian_frame_error = "pedestrian_sim_time_invalid"
            self.pedestrian_progress.error = "pedestrian_sim_time_invalid"
            return
        source_frame = str(data.get("frame_id", "")).strip()
        if not source_frame:
            self.pedestrian_frame_error = "pedestrian_frame_id_missing"
            return
        planar_transform = None
        if source_frame != ("world" if self.config.onboard_only else self.config.map_frame):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.config.map_frame, source_frame, rclpy.time.Time()
                ).transform
                planar_transform = (
                    float(transform.translation.x), float(transform.translation.y),
                    _yaw_from_quat(transform.rotation),
                )
            except TransformException:
                if self.latest_people_stamp is None:
                    self.pedestrian_frame_error = f"transform_unavailable:{source_frame}->{self.config.map_frame}"
                return
        try:
            people = people_from_payload(data, self.config.person_radius_m, planar_transform)
        except (TypeError, ValueError, OverflowError):
            self.pedestrian_frame_error = "pedestrian_payload_invalid"
            self.pedestrian_progress.error = "pedestrian_payload_invalid"
            return
        population_count = data.get("population_count", len(people))
        if type(population_count) is not int or population_count < len(people):
            self.pedestrian_frame_error = "pedestrian_population_invalid"
            self.pedestrian_progress.error = "pedestrian_population_invalid"
            return
        self.latest_people = people
        self.latest_people_population = population_count
        received_s = time.monotonic()
        self.pedestrian_progress.update(float(sim_time), received_s)
        self.pedestrian_frame_error = None
        self.latest_people_stamp = self.get_clock().now()
        self.latest_people_received_s = received_s

    def _raw_cmd_callback(self, msg) -> None:
        self.latest_raw_cmd_stamp = time.monotonic()
        self.latest_raw_cmd_magnitude = (
            math.hypot(float(msg.linear.x), float(msg.linear.y)) + abs(float(msg.angular.z))
        )

    def _scan_callback(self, _msg) -> None:
        self.latest_scan_received_s = time.monotonic()
        self.scan_messages_observed += 1

    def _sensor_status_callback(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        source = data.get("lidar_source")
        source_sequence = data.get("lidar_source_sequence")
        if not isinstance(source, str) or not source or type(source_sequence) is not int:
            self.scan_progress.error = "scan_source_status_invalid"
            return
        received_s = time.monotonic()
        self.latest_scan_source = source
        self.latest_scan_source_received_s = received_s
        self.scan_progress.update(float(source_sequence), received_s)

    def _safe_cmd_callback(self, msg) -> None:
        self.max_abs_safe_lateral_cmd_mps = max(
            self.max_abs_safe_lateral_cmd_mps,
            abs(float(msg.linear.y)),
        )

    def _locomotion_callback(self, msg) -> None:
        try:
            self.latest_locomotion = json.loads(msg.data)
            if self.scoring_active:
                self.live_events.update(self.latest_locomotion)
        except json.JSONDecodeError:
            return

    def _safety_debug_callback(self, msg) -> None:
        if not self.scoring_active:
            return
        try:
            self.live_events.update(json.loads(msg.data))
        except json.JSONDecodeError:
            return

    def _human_avoidance_callback(self, msg) -> None:
        if self.scoring_active:
            self.human_avoidance.update(str(msg.data), time.monotonic())

    def _tracker_debug_callback(self, msg) -> None:
        try:
            source = json.loads(msg.data).get("effective_people_source")
        except (json.JSONDecodeError, AttributeError):
            return
        if not isinstance(source, str) or not source:
            return
        self.latest_control_people_source = source
        self.latest_control_people_source_received_s = time.monotonic()

    def set_scoring_active(self, active: bool, now_s: float | None = None) -> None:
        if self.scoring_active == active:
            return
        now = time.monotonic() if now_s is None else now_s
        self.scoring_active = active
        if not active:
            self.live_events.pause()
            self.human_avoidance.pause(now)

    def frame_error(self) -> str | None:
        return self.odom_frame_error or self.pedestrian_frame_error

    def live_state_ages(
        self, now_s: float | None = None
    ) -> tuple[float | None, float | None, float | None, float | None]:
        now = time.monotonic() if now_s is None else now_s
        odom_age = None if self.latest_odom_received_s is None else max(0.0, now - self.latest_odom_received_s)
        people_age = (
            None if self.latest_people_received_s is None
            else max(0.0, now - self.latest_people_received_s)
        )
        scan_age = None if self.latest_scan_received_s is None else max(0.0, now - self.latest_scan_received_s)
        scan_source_age = (
            None if self.latest_scan_source_received_s is None
            else max(0.0, now - self.latest_scan_source_received_s)
        )
        return odom_age, people_age, scan_age, scan_source_age

    def live_state_health_error(self, now_s: float | None = None) -> str | None:
        now = time.monotonic() if now_s is None else now_s
        odom_age, people_age, scan_age, scan_source_age = self.live_state_ages(now_s)
        observed_people = (
            None if self.latest_people_received_s is None else self.latest_people_population
        )
        error = live_state_health_error(
            requested_people=self.config.people_count,
            observed_people=observed_people,
            odom_age_s=odom_age,
            pedestrian_age_s=people_age,
            scan_age_s=scan_age,
            scan_source_age_s=scan_source_age,
            scan_source_expected=self.config.scan_source,
            scan_source_observed=self.latest_scan_source,
            odom_stale_timeout_s=self.config.odom_stale_timeout_s,
            pedestrian_stale_timeout_s=self.config.pedestrian_stale_timeout_s,
            scan_stale_timeout_s=self.config.scan_stale_timeout_s,
            require_exact_people_count=self.config.require_exact_people_count,
        )
        if error is not None:
            return error
        expected_control_source = self.config.control_people_source
        if self.config.people_count > 0 and expected_control_source not in {"auto", "not_used"}:
            if self.latest_control_people_source_received_s is None:
                return "control_people_source_status_missing"
            source_age = max(0.0, now - self.latest_control_people_source_received_s)
            if source_age > self.config.pedestrian_stale_timeout_s:
                return (
                    "control_people_source_status_stale:"
                    f"{source_age:.3f}s>{self.config.pedestrian_stale_timeout_s:.3f}s"
                )
            if self.latest_control_people_source != expected_control_source:
                return (
                    "control_people_source_mismatch:"
                    f"expected={expected_control_source},observed={self.latest_control_people_source}"
                )
        for progress, timeout in (
            (self.odom_progress, self.config.odom_stale_timeout_s),
            (self.scan_progress, self.config.scan_stale_timeout_s),
        ):
            error = progress.health_error(timeout, now)
            if error is not None:
                return error
        if self.config.people_count > 0:
            return self.pedestrian_progress.health_error(
                self.config.pedestrian_stale_timeout_s, now
            )
        return None

    def monitor_live_state_health(self, now_s: float | None = None) -> str | None:
        odom_age, people_age, scan_age, _ = self.live_state_ages(now_s)
        if odom_age is not None:
            self.max_odom_age_s = max(self.max_odom_age_s, odom_age)
        if people_age is not None and self.config.people_count > 0:
            self.max_pedestrian_age_s = max(self.max_pedestrian_age_s, people_age)
        if scan_age is not None:
            self.max_scan_age_s = max(self.max_scan_age_s, scan_age)
        if self.latest_control_people_source_received_s is not None and self.config.people_count > 0:
            control_source_age = max(
                0.0,
                (time.monotonic() if now_s is None else now_s)
                - self.latest_control_people_source_received_s,
            )
            self.max_control_people_source_age_s = max(
                self.max_control_people_source_age_s, control_source_age
            )
        error = self.live_state_health_error(now_s)
        if self.health_monitoring_started and error is not None and self.latched_health_error is None:
            self.latched_health_error = error
        return self.latched_health_error or error

    def robot_fallen(self, threshold: float = 0.95) -> bool:
        if not self.latest_locomotion:
            return False
        lowstate = (self.latest_locomotion.get("lowstate") or {}).get("value") or {}
        tilt = float(lowstate.get("tilt", 0.0) or 0.0)
        max_tilt = float(self.latest_locomotion.get("max_tilt_rad", 0.0) or 0.0)
        return max(tilt, max_tilt) >= threshold

    def wait_for_live_state(self, timeout_s: float = 15.0) -> None:
        if not self.ros_enabled:
            raise RuntimeError("MuJoCo backend requires ROS 2 Python support")
        deadline = time.monotonic() + timeout_s
        graph_errors = []
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            population_error = None
            if self.latest_people_received_s is not None:
                population_error = live_state_health_error(
                    requested_people=self.config.people_count,
                    observed_people=self.latest_people_population,
                    odom_age_s=0.0,
                    pedestrian_age_s=0.0,
                    scan_age_s=0.0,
                    scan_source_age_s=0.0,
                    scan_source_expected="auto",
                    scan_source_observed="test",
                    odom_stale_timeout_s=self.config.odom_stale_timeout_s,
                    pedestrian_stale_timeout_s=self.config.pedestrian_stale_timeout_s,
                    scan_stale_timeout_s=self.config.scan_stale_timeout_s,
                    require_exact_people_count=self.config.require_exact_people_count,
                )
            if population_error and population_error.startswith("people_count_mismatch"):
                raise RuntimeError(population_error)
            if self.live_state_health_error() is None:
                if self.config.onboard_only and (self.observed_map is None or self.evaluation_pose is None):
                    continue
                if self.config.onboard_only:
                    graph_errors = audit_onboard_graph(self)
                    if graph_errors:
                        continue
                    self.get_logger().info("Onboard graph audit passed: navigation has no privileged subscriptions; /odom is owned by the LiDAR/IMU estimator")
                    grid = self.observed_map
                    self.get_logger().info(f"Observed map {grid.info.width}x{grid.info.height}, free={sum(0 <= v < 20 for v in grid.data)}, robot={self.latest_robot_pose}")
                self.health_monitoring_started = True
                return
        if self.frame_error() is not None:
            raise RuntimeError(self.frame_error())
        if graph_errors:
            raise RuntimeError("onboard_source_contract: " + "; ".join(graph_errors))
        health_error = self.live_state_health_error()
        if health_error is not None:
            raise RuntimeError(health_error)
        required = "/odom" if self.config.people_count == 0 else "/odom and /mujoco/pedestrians"
        raise RuntimeError(f"Timed out waiting for live {required} state")

    def wait_for_safe_start(self, timeout_s: float = 20.0) -> None:
        if self.config.onboard_only:
            return  # Truth proximity is scoring-only, never a mission admission input.
        if not self.ros_enabled:
            return
        deadline = time.monotonic() + timeout_s
        last_warning = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            health_error = self.monitor_live_state_health()
            if health_error is not None and not health_error.endswith("_missing"):
                raise RuntimeError(health_error)
            if self.latest_robot_pose is None:
                continue
            if self.config.people_count == 0:
                return
            if self.latest_people_stamp is None:
                continue
            if not self.latest_people:
                return
            min_distance = min(distance_xy(self.latest_robot_pose, person.pose) for person in self.latest_people)
            min_required = max(
                self.config.warning_distance_m,
                max(self.config.robot_radius_m + person.radius_m + 0.45 for person in self.latest_people),
            )
            if min_distance >= min_required:
                return
            now = time.monotonic()
            if now - last_warning > 2.0:
                self.get_logger().warn(
                    f"Waiting for safe benchmark start: closest live person {min_distance:.2f}m < {min_required:.2f}m"
                )
                last_warning = now
        raise RuntimeError("Timed out waiting for people to clear the benchmark start area")

    def goal_has_path(self, start: Pose2D, goal: Pose2D, timeout_s: float = 5.0) -> bool:
        """Require a successful path whose endpoint actually reaches the goal."""
        if self.path_client is None or not self.path_client.wait_for_server(timeout_sec=timeout_s):
            return False
        if self.config.onboard_only:
            rclpy.spin_once(self, timeout_sec=0.0)
        if self.config.onboard_only and not observed_free(self.observed_map, goal.x, goal.y, self.config.robot_radius_m + 0.15):
            self.get_logger().info(f"Goal not observed free: {goal}", throttle_duration_sec=2.0)
            return False
        request = ComputePathToPose.Goal()
        request.use_start = True
        request.start = self._pose_stamped(start)
        request.goal = self._pose_stamped(goal)
        future = self.path_client.send_goal_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.monitor_live_state_health() is not None:
                return False
        if not future.done() or future.result() is None or not future.result().accepted:
            return False
        result_future = future.result().get_result_async()
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.monitor_live_state_health() is not None:
                return False
        if not result_future.done() or result_future.result() is None:
            return False
        result = result_future.result()
        poses = result.result.path.poses
        has_path = result.status == 4 and len(poses) > 0
        if result.result.path.header.frame_id and result.result.path.header.frame_id != self.config.map_frame:
            return False
        if has_path:
            endpoint = poses[-1].pose.position
            endpoint_error = math.hypot(endpoint.x - goal.x, endpoint.y - goal.y)
            has_path = math.isfinite(endpoint_error) and endpoint_error <= min(0.25, self.config.goal_tolerance_m)
        if has_path and self.config.onboard_only:
            has_path = all(observed_free(self.observed_map, p.pose.position.x, p.pose.position.y, self.config.robot_radius_m)
                           for p in poses)
        if has_path:
            self.get_logger().info(f"Reachable goal accepted at x={goal.x:+.2f}, y={goal.y:+.2f}")
        return has_path

    def wait_for_nav_path(self, start: Pose2D, goal: Pose2D, timeout_s: float = 15.0) -> bool:
        deadline = time.monotonic() + timeout_s
        last_warning = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if self.goal_has_path(start, goal, min(2.0, remaining)):
                return True
            if self.latched_health_error is not None:
                return False
            if self.robot_fallen():
                return False
            now = time.monotonic()
            if now - last_warning > 2.0:
                self.get_logger().warn("Waiting for Nav2 to produce a path before submitting the goal")
                last_warning = now
        return False

    def _pose_stamped(self, pose: Pose2D):
        msg = PoseStamped()
        msg.header.frame_id = self.config.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.orientation.z = math.sin(pose.yaw * 0.5)
        msg.pose.orientation.w = math.cos(pose.yaw * 0.5)
        return msg

    def send_nav_goal(self, goal: Goal, timeout_s: float = 30.0) -> bool:
        self.publish_goal(goal)
        if self.nav_client is None:
            return False
        if not self.nav_client.wait_for_server(timeout_sec=timeout_s):
            self.get_logger().warn("NavigateToPose action server unavailable; publishing benchmark goal only")
            return False
        action_goal = NavigateToPose.Goal()
        action_goal.pose.header.frame_id = self.config.map_frame
        action_goal.pose.header.stamp = self.get_clock().now().to_msg()
        action_goal.pose.pose.position.x = goal.pose.x
        action_goal.pose.pose.position.y = goal.pose.y
        qz = math.sin(goal.pose.yaw * 0.5)
        qw = math.cos(goal.pose.yaw * 0.5)
        action_goal.pose.pose.orientation.z = qz
        action_goal.pose.pose.orientation.w = qw
        future = self.nav_client.send_goal_async(action_goal)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.monitor_live_state_health() is not None:
                return False
        if not future.done():
            self.get_logger().warn("NavigateToPose goal was not accepted before startup timeout")
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("NavigateToPose goal was rejected")
            return False
        self.nav_goal_handle = goal_handle
        self.current_goal_index = goal.index
        self.mission_active = True
        self.publish_mission_state()
        self.nav_result_status = None
        self.nav_goal_success = False
        self.nav_result_future = goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(self._nav_result_callback)
        return True

    def _nav_result_callback(self, future) -> None:
        if future is not self.nav_result_future:
            return
        result = future.result()
        self.nav_result_status = None if result is None else int(result.status)
        self.nav_goal_success = self.nav_result_status == NAV_STATUS_SUCCEEDED
        self.get_logger().info(f"NavigateToPose terminal status: {self.nav_result_status}")

    def nav_goal_succeeded(self) -> bool:
        return self.nav_goal_success

    def retry_aborted_goal(self, goal, now_s):
        """Restart a transiently aborted action without resetting the goal clock.

        This is asynchronous: scoring and the mission heartbeat continue while
        Nav2 responds. A late acceptance after cancellation is itself cancelled.
        """
        if (not self.config.onboard_only or not self.mission_active or self.nav_result_status != 6
                or self.nav_retry_future is not None or now_s - self.last_nav_retry_s < 2.0
                or not self.nav_client.server_is_ready()):
            return
        self.last_nav_retry_s = now_s
        request = NavigateToPose.Goal()
        request.pose = self._pose_stamped(goal.pose)
        self.nav_retry_future = self.nav_client.send_goal_async(request)

        def accepted(future):
            self.nav_retry_future = None
            handle = future.result()
            if handle is None or not handle.accepted:
                return
            if not self.mission_active or self.current_goal_index != goal.index:
                handle.cancel_goal_async()
                return
            self.nav_goal_handle = handle
            self.nav_result_status = None
            self.nav_goal_success = False
            self.nav_result_future = handle.get_result_async()
            self.nav_result_future.add_done_callback(self._nav_result_callback)
            self.get_logger().info("Retrying the same waypoint after a Nav2 abort; original deadline retained")

        self.nav_retry_future.add_done_callback(accepted)

    def cancel_nav_goal(self, timeout_s: float = 2.0) -> None:
        """Release the active Nav2 action before advancing benchmark goals."""
        self.mission_active = False
        self.current_goal_index = None
        self.publish_mission_state()
        goal_handle = self.nav_goal_handle
        result_future = self.nav_result_future
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_result_status = None
        self.nav_goal_success = False
        if goal_handle is None:
            return
        if result_future is not None and result_future.done():
            return
        future = goal_handle.cancel_goal_async()
        deadline = time.monotonic() + timeout_s
        while _ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.monitor_live_state_health()
        result_future = goal_handle.get_result_async()
        while _ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.monitor_live_state_health()

    def wait_for_motion_cmd_after(self, stamp: float, timeout_s: float = 15.0) -> bool:
        if not self.ros_enabled:
            return True
        deadline = time.monotonic() + timeout_s
        last_warning = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.monitor_live_state_health() is not None:
                return False
            if (
                self.latest_raw_cmd_stamp is not None
                and self.latest_raw_cmd_stamp >= stamp
                and self.latest_raw_cmd_magnitude >= 0.01
            ):
                return True
            now = time.monotonic()
            if now - last_warning > 2.0:
                self.get_logger().warn("Waiting for first nonzero /cmd_vel after accepted Nav2 goal")
                last_warning = now
        self.get_logger().warn("No nonzero /cmd_vel received after accepted Nav2 goal before startup timeout")
        return False


class BenchmarkRunner:
    def __init__(self, config: BenchmarkConfig, output_path: str | None = None) -> None:
        self.config = config
        self.rng = random.Random(config.seed)
        self.writer = ResultWriter(config, output_path)

    def run(self) -> tuple[ScoreBreakdown, str]:
        if self.config.backend not in {"mock", "mujoco"}:
            raise ValueError(f"unsupported backend: {self.config.backend}")

        node = BenchmarkRosNode(self.config)
        try:
            score, result_path = self._run_mujoco(node) if self.config.backend == "mujoco" else self._run_mock(node)
            node.publish_result(score)
            return score, result_path
        finally:
            if node.ros_enabled:
                if rclpy.ok():
                    preparer = getattr(node, "next_goal_preparer", None)
                    if preparer is not None:
                        preparer.clear()
                    node.cancel_nav_goal(timeout_s=.5)
                node.destroy_node()

    def _run_mock(self, node: BenchmarkRosNode) -> tuple[ScoreBreakdown, str]:
        start = Pose2D(0.0, 0.0, 0.0)
        people_manager = PeopleManager(self.config, self.rng)
        if self.config.scenario_file:
            goals, people = self._load_scenario(Path(self.config.scenario_file))
        else:
            people = people_manager.reset(start)
            goals = GoalSampler(self.config, self.rng).sample(start, people)
        monitor = MetricsMonitor(self.config)
        monitor.metrics.people_observed = len(people)
        monitor.metrics.people_active_initial = len(people)
        monitor.metrics.people_active_minimum = len(people)
        monitor.metrics.people_active_final = len(people)
        monitor.metrics.scan_source_observed = "not_used"
        monitor.metrics.control_people_source_observed = "mock_goal_follower_no_people_input"
        robot_pose = start
        robot_velocity = Velocity2D()
        now_s = 0.0
        previous_goal_pose = start
        scenario_path = self.writer.write_scenario(self.config, goals, people)

        for goal in goals:
            node.publish_goal(goal)
            goal_metrics = monitor.start_goal(goal)
            goal_start_time = now_s
            monitor.metrics.ideal_path_length_m += distance_xy(previous_goal_pose, goal.pose)
            previous_goal_pose = goal.pose

            while _ok():
                dt = self.config.dt_s
                now_s += dt
                if now_s > self.config.total_timeout_s:
                    monitor.complete_goal(goal_metrics, False, "benchmark_timeout", now_s - goal_start_time)
                    score = self._finish(monitor, goals, people, scenario_path)
                    return score, str(self.writer.output_path)

                if self.config.scenario_file:
                    for person in people:
                        update_person(person, dt, now_s, person.speed_mps, self.config.bounds)
                else:
                    people = people_manager.update(dt, now_s)
                monitor.metrics.people_active_minimum = min(
                    monitor.metrics.people_active_minimum, len(people)
                )
                monitor.metrics.people_active_final = len(people)
                robot_pose, robot_velocity = self._step_robot(robot_pose, goal.pose, dt)
                monitor.update(now_s, dt, robot_pose, robot_velocity, people)
                node.publish_people(people)
                node.publish_metrics({"time_s": now_s, "goal_index": goal.index, "goals_reached": monitor.metrics.goals_reached, "collisions": monitor.metrics.collisions})
                if rclpy is not None:
                    rclpy.spin_once(node, timeout_sec=0.0)

                if distance_xy(robot_pose, goal.pose) <= self.config.goal_tolerance_m:
                    monitor.complete_goal(goal_metrics, True, None, now_s - goal_start_time)
                    break
                if now_s - goal_start_time > self.config.max_goal_time_s:
                    monitor.complete_goal(goal_metrics, False, "goal_timeout", now_s - goal_start_time)
                    break
                if monitor.metrics.collisions > 0:
                    monitor.complete_goal(goal_metrics, False, "collision", now_s - goal_start_time)
                    score = self._finish(monitor, goals, people, scenario_path)
                    return score, str(self.writer.output_path)

        score = self._finish(monitor, goals, people, scenario_path)
        return score, str(self.writer.output_path)

    def _step_robot(self, pose: Pose2D, goal: Pose2D, dt_s: float) -> tuple[Pose2D, Velocity2D]:
        dx = goal.x - pose.x
        dy = goal.y - pose.y
        distance = math.hypot(dx, dy)
        if distance < 1e-9:
            return pose, Velocity2D()
        speed = min(self.config.max_speed_mps * 0.65, distance / max(dt_s, 1e-9))
        vx = speed * dx / distance
        vy = speed * dy / distance
        return Pose2D(pose.x + vx * dt_s, pose.y + vy * dt_s, math.atan2(vy, vx)), Velocity2D(vx, vy, 0.0)

    def _run_mujoco(self, node: BenchmarkRosNode) -> tuple[ScoreBreakdown, str]:
        monitor = MetricsMonitor(self.config)
        try:
            node.wait_for_live_state()
            node.wait_for_safe_start()
        except RuntimeError as exc:
            return self._finish_invalid(monitor, [], node.latest_people, str(exc))
        if self.config.people_count == 0:
            node.latest_people = []
        monitor.metrics.people_observed = node.latest_people_population or 0
        monitor.metrics.people_active_initial = len(node.latest_people)
        monitor.metrics.people_active_minimum = len(node.latest_people)
        monitor.metrics.people_active_final = len(node.latest_people)
        if len(node.latest_people) != self.config.people_count:
            return self._finish_invalid(
                monitor,
                [],
                node.latest_people,
                f"people_active_at_start_mismatch:requested={self.config.people_count},observed={len(node.latest_people)}",
            )
        start = node.latest_robot_pose or Pose2D(0.0, 0.0, 0.0)
        dynamic_random_goals = not self.config.scenario_file and self.config.goal_mode == "random"
        sampler = GoalSampler(
            self.config,
            self.rng,
            lambda start_pose, goal_pose: node.goal_has_path(
                start_pose, goal_pose, self.config.path_validation_timeout_s
            ),
        )
        if self.config.scenario_file:
            goals, _ = self._load_scenario(Path(self.config.scenario_file))
        elif dynamic_random_goals:
            goals = []
        else:
            try:
                goals = sampler.sample(start, node.goal_people())
            except RuntimeError as exc:
                return self._finish_invalid(monitor, [], node.latest_people, f"goal_sampling_failed: {exc}")
        previous_goal_pose = start
        scenario_path = self.writer.write_scenario(self.config, goals, node.latest_people)
        start_wall = time.monotonic()
        last_sample = start_wall

        prepared_next = None
        preparer = None
        if self.config.visual_session:
            from .next_goal import NextGoalPreparer
            preparer = NextGoalPreparer(node, self.rng)
            node.next_goal_preparer = preparer
        for goal_index in range(self.config.goals_count):
            if self.config.visual_session and time.monotonic() - start_wall >= self.config.total_timeout_s:
                monitor.metrics.session_end_reason = "duration_elapsed"
                monitor.metrics.total_time_s = time.monotonic() - start_wall
                break
            if preparer is not None:
                preparer.clear()
            if dynamic_random_goals:
                current_pose = node.latest_robot_pose or start
                goal = prepared_next
                prevalidated = goal is not None
                prepared_next = None
                while goal is None and _ok():
                    try:
                        goal = sampler.sample_random_goal(goal_index, current_pose, node.goal_people())
                    except RuntimeError as exc:
                        if not self.config.visual_session:
                            if not goals:
                                return self._finish_invalid(monitor, goals, node.latest_people, f"goal_sampling_failed: {exc}")
                            break
                        error = node.monitor_live_state_health()
                        if error is not None:
                            return self._finish_invalid(monitor, goals, node.latest_people, error)
                        if time.monotonic() - start_wall >= self.config.total_timeout_s:
                            monitor.metrics.session_end_reason = "duration_elapsed"
                            monitor.metrics.total_time_s = time.monotonic() - start_wall
                            break
                        node.get_logger().info("Waiting for observed free space for the next visual-session goal", throttle_duration_sec=5.)
                        until = time.monotonic() + .5
                        while _ok() and time.monotonic() < until:
                            rclpy.spin_once(node, timeout_sec=.05)
                        current_pose = node.latest_robot_pose or current_pose
                if goal is None:
                    break
                goals.append(goal)
                scenario_path = self.writer.write_scenario(self.config, goals, node.latest_people)
            else:
                goal = goals[goal_index]
                prevalidated = False
            current_pose = node.latest_robot_pose or start
            if not prevalidated and not node.wait_for_nav_path(current_pose, goal.pose):
                health_error = node.monitor_live_state_health()
                if health_error is not None:
                    return self._finish_invalid(
                        monitor, goals, node.latest_people, health_error
                    )
                if self.config.visual_session:
                    failed = monitor.start_goal(goal)
                    monitor.complete_goal(failed, False, "path_unavailable", 0.)
                    continue
                if monitor.metrics.total_time_s <= 0.0 and monitor.metrics.goals_reached == 0:
                    return self._finish_invalid(
                        monitor, goals, node.latest_people, "nav2_path_not_ready_before_scoring_start"
                    )
                return self._finish(monitor, goals, node.latest_people, scenario_path), str(self.writer.output_path)
            goal_metrics = monitor.start_goal(goal)
            monitor.metrics.ideal_path_length_m += distance_xy(previous_goal_pose, goal.pose)
            previous_goal_pose = goal.pose
            if node.robot_fallen():
                monitor.metrics.fallen = True
                monitor.complete_goal(goal_metrics, False, "fallen", 0.0)
                return self._finish(monitor, goals, node.latest_people, scenario_path), str(self.writer.output_path)
            motion_started = False
            goal_accepted = False
            goal_start_time = time.monotonic()
            if monitor.metrics.total_time_s <= 0.0:
                node.live_events.reset()
                node.human_avoidance.reset()
                if not self.config.visual_session:
                    start_wall = goal_start_time
            for attempt in range(2):
                goal_accepted = node.send_nav_goal(goal)
                if not goal_accepted:
                    break
                node.set_scoring_active(True)
                if node.wait_for_motion_cmd_after(time.monotonic()):
                    motion_started = True
                    break
                node.set_scoring_active(False)
                if attempt == 0:
                    node.cancel_nav_goal()
                    node.get_logger().warn(
                        "Accepted Nav2 goal produced no motion command; retrying once"
                    )
            if not goal_accepted:
                if monitor.metrics.total_time_s <= 0.0 and monitor.metrics.goals_reached == 0:
                    return self._finish_invalid(monitor, goals, node.latest_people, "nav2_not_ready_or_first_goal_rejected")
                monitor.complete_goal(goal_metrics, False, "nav2_goal_not_accepted", 0.0)
                return self._finish(monitor, goals, node.latest_people, scenario_path), str(self.writer.output_path)
            health_error = node.monitor_live_state_health()
            if health_error is not None:
                node.set_scoring_active(False)
                node.cancel_nav_goal()
                return self._finish_invalid(
                    monitor, goals, node.latest_people, health_error
                )
            if not motion_started:
                node.set_scoring_active(False)
                if monitor.metrics.total_time_s <= 0.0 and monitor.metrics.goals_reached == 0:
                    return self._finish_invalid(
                        monitor, goals, node.latest_people, "no_nonzero_cmd_vel_before_scoring_start"
                    )
                monitor.complete_goal(goal_metrics, False, "no_nonzero_cmd_vel_before_start", 0.0)
                return self._finish(monitor, goals, node.latest_people, scenario_path), str(self.writer.output_path)
            last_sample = time.monotonic()

            while _ok():
                rclpy.spin_once(node, timeout_sec=max(0.0, min(self.config.dt_s, 0.1)))
                now_wall = time.monotonic()
                now_s = now_wall - start_wall
                if node.frame_error() is not None:
                    node.set_scoring_active(False, now_wall)
                    node.cancel_nav_goal()
                    return self._finish_invalid(
                        monitor, goals, node.latest_people, node.frame_error() or "frame_transform_unavailable"
                    )
                health_error = node.monitor_live_state_health(now_wall)
                monitor.metrics.max_odom_age_s = node.max_odom_age_s
                monitor.metrics.max_pedestrian_age_s = node.max_pedestrian_age_s
                monitor.metrics.max_scan_age_s = node.max_scan_age_s
                monitor.metrics.max_control_people_source_age_s = (
                    node.max_control_people_source_age_s
                )
                monitor.metrics.max_odom_sample_stall_s = node.odom_progress.max_stall_s
                monitor.metrics.max_pedestrian_sample_stall_s = node.pedestrian_progress.max_stall_s
                monitor.metrics.max_scan_sample_stall_s = node.scan_progress.max_stall_s
                if health_error is not None:
                    node.set_scoring_active(False, now_wall)
                    node.cancel_nav_goal()
                    return self._finish_invalid(
                        monitor, goals, node.latest_people, health_error
                    )
                monitor.metrics.people_observed = node.latest_people_population or 0
                monitor.metrics.people_active_minimum = min(
                    monitor.metrics.people_active_minimum, len(node.latest_people)
                )
                monitor.metrics.people_active_final = len(node.latest_people)
                monitor.metrics.scan_messages_observed = node.scan_messages_observed
                monitor.metrics.scan_source_observed = node.latest_scan_source or "not_observed"
                monitor.metrics.control_people_source_observed = (
                    node.latest_control_people_source or "not_observed"
                )
                dt = max(1e-3, now_wall - last_sample)
                if dt < self.config.dt_s * 0.5:
                    continue
                last_sample = now_wall
                if now_s > self.config.total_timeout_s:
                    node.set_scoring_active(False, now_wall)
                    if self.config.visual_session:
                        monitor.metrics.session_end_reason = "duration_elapsed"
                        monitor.metrics.total_time_s = now_s
                    monitor.complete_goal(goal_metrics, False, "session_duration" if self.config.visual_session else "benchmark_timeout", now_wall - goal_start_time)
                    return self._finish(monitor, goals, node.latest_people, scenario_path), str(self.writer.output_path)

                robot_pose = node.latest_robot_pose
                if robot_pose is None:
                    continue
                if preparer is not None and goal_index + 1 < self.config.goals_count:
                    if distance_xy(robot_pose, goal.pose) <= 1.1:
                        if preparer.ready is not None and time.monotonic()-preparer.ready[2] > 4.:
                            preparer.clear()
                        preparer.prepare(goal_index + 1, goal.pose)
                people = node.latest_people[: self.config.people_count] if self.config.people_count > 0 else []
                previous_path_length = monitor.metrics.actual_path_length_m
                scoring_pose = node.evaluation_pose if self.config.onboard_only else robot_pose
                if self.config.onboard_only and (scoring_pose is None or node.evaluation_received_s is None or now_wall - node.evaluation_received_s > 1.0):
                    node.cancel_nav_goal()
                    return self._finish_invalid(monitor, goals, people, "evaluation_odometry_stale")
                scoring_velocity = node.evaluation_velocity if self.config.onboard_only else node.latest_robot_velocity
                monitor.update(now_s, dt, scoring_pose, scoring_velocity, people)
                step_length = monitor.metrics.actual_path_length_m - previous_path_length
                if node.live_events._override_active:
                    monitor.metrics.avoidance_time_s += dt
                if node.human_avoidance.state == "YIELD_TO_HUMAN" and any(
                    math.hypot(person.velocity.vx, person.velocity.vy) >= 0.10
                    for person in people
                ):
                    monitor.metrics.moving_avoidance_time_s += dt
                elif node.human_avoidance.resume_episodes > 0:
                    monitor.metrics.post_avoidance_progress_m += step_length
                monitor.metrics.emergency_stops = node.live_events.emergency_stops
                monitor.metrics.avoidance_interventions = node.live_events.avoidance_interventions
                monitor.metrics.stale_events = node.live_events.stale_events
                monitor.metrics.max_abs_safe_lateral_cmd_mps = node.max_abs_safe_lateral_cmd_mps
                previous_yields = monitor.metrics.yield_episodes
                previous_resumes = monitor.metrics.resume_episodes
                monitor.metrics.yield_episodes = node.human_avoidance.yield_episodes
                monitor.metrics.resume_episodes = node.human_avoidance.resume_episodes
                monitor.metrics.yield_time_s = node.human_avoidance.total_yield_time(now_wall)
                if monitor.metrics.yield_episodes > previous_yields:
                    monitor.metrics.events.append(SafetyEvent(now_s, "yield"))
                if monitor.metrics.resume_episodes > previous_resumes:
                    monitor.metrics.events.append(SafetyEvent(now_s, "resume"))
                node.publish_people(people)
                node.publish_metrics(
                    {
                        "time_s": now_s,
                        "goal_index": goal.index,
                        "goals_reached": monitor.metrics.goals_reached,
                        "collisions": monitor.metrics.collisions,
                        "backend": "mujoco",
                        "goal_distance_m": distance_xy(robot_pose, goal.pose),
                        "next_goal_ready": preparer is not None and preparer.ready is not None,
                        "yield_episodes": monitor.metrics.yield_episodes,
                        "resume_episodes": monitor.metrics.resume_episodes,
                        "yield_time_s": monitor.metrics.yield_time_s,
                    }
                )
                node.publish_oracle(
                    f"{goal.index + 1}/{self.config.goals_count}",
                    monitor.metrics.minimum_body_clearance_m,
                    now_wall,
                )

                if sensed_goal_reached(robot_pose, goal.pose, self.config.goal_tolerance_m,
                                       node.nav_goal_succeeded(), self.config.visual_session):
                    if preparer is not None:
                        prepared_next = preparer.take(goal_index + 1)
                        node.get_logger().info(
                            f"Waypoint reached at {distance_xy(robot_pose, goal.pose):.3f} m; "
                            f"next goal {'prepared' if prepared_next is not None else 'being planned'}")
                    node.set_scoring_active(False, now_wall)
                    node.cancel_nav_goal(timeout_s=0.0 if self.config.visual_session else 2.0)
                    if prepared_next is not None:
                        node.publish_goal(prepared_next)
                    monitor.complete_goal(goal_metrics, True, None, now_wall - goal_start_time)
                    break
                if now_wall - goal_start_time > self.config.max_goal_time_s:
                    node.set_scoring_active(False, now_wall)
                    node.cancel_nav_goal()
                    monitor.complete_goal(goal_metrics, False, "goal_timeout", now_wall - goal_start_time)
                    break
                if monitor.metrics.collisions > 0 and not self.config.visual_session:
                    node.set_scoring_active(False, now_wall)
                    monitor.complete_goal(goal_metrics, False, "collision", now_wall - goal_start_time)
                    return self._finish(monitor, goals, people, scenario_path), str(self.writer.output_path)
                if node.robot_fallen():
                    node.set_scoring_active(False, now_wall)
                    monitor.metrics.fallen = True
                    monitor.complete_goal(goal_metrics, False, "fallen", now_wall - goal_start_time)
                    return self._finish(monitor, goals, people, scenario_path), str(self.writer.output_path)
                node.retry_aborted_goal(goal, now_wall)

        score = self._finish(monitor, goals, node.latest_people, scenario_path)
        return score, str(self.writer.output_path)

    def _finish(self, monitor: MetricsMonitor, goals: list[Goal], people, scenario_path) -> ScoreBreakdown:
        if monitor.metrics.valid_trial and monitor.metrics.actual_path_length_m < 0.05 and monitor.metrics.goals_reached == 0:
            monitor.metrics.no_movement = True
        score = score_benchmark(monitor.metrics, self.config)
        self.writer.write(self.config, monitor.metrics, score, goals, people, scenario_path)
        return score

    def _finish_invalid(self, monitor: MetricsMonitor, goals: list[Goal], people, reason: str) -> tuple[ScoreBreakdown, str]:
        monitor.metrics.valid_trial = False
        monitor.metrics.invalid_reason = reason
        scenario_path = self.writer.write_scenario(self.config, goals, people)
        score = self._finish(monitor, goals, people, scenario_path)
        return score, str(self.writer.output_path)

    def _load_scenario(self, path: Path) -> tuple[list[Goal], list[PersonState]]:
        data = yaml.safe_load(path.read_text()) or {}
        if "seed" in data:
            self.config.seed = int(data["seed"])
        goals = [
            Goal(int(item["index"]), Pose2D(float(item["pose"]["x"]), float(item["pose"]["y"]), float(item["pose"].get("yaw", 0.0))))
            for item in data.get("goals", [])
        ]
        people: list[PersonState] = []
        for item in data.get("people", []):
            pose = item["pose"]
            vel = item.get("velocity", {})
            person = PersonState(
                person_id=int(item["id"]),
                pose=Pose2D(float(pose["x"]), float(pose["y"]), float(pose.get("yaw", 0.0))),
                velocity=Velocity2D(float(vel.get("x", 0.0)), float(vel.get("y", 0.0)), float(vel.get("yaw", 0.0))),
                radius_m=float(item.get("radius_m", self.config.person_radius_m)),
                behaviour=str(item.get("behaviour", "random_waypoint")),
                speed_mps=float(item.get("speed_mps", 0.5)),
                waypoints=[Pose2D(float(p["x"]), float(p["y"]), float(p.get("yaw", 0.0))) for p in item.get("waypoints", [])],
            )
            if not person.waypoints:
                person.waypoints = [person.pose]
            people.append(person)
        if not goals:
            raise ValueError(f"scenario file contains no goals: {path}")
        self.config.goals_count = len(goals)
        self.config.people_count = len(people)
        return goals, people


def _ok() -> bool:
    return True if rclpy is None else rclpy.ok()


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny_cosp, cosy_cosp)
