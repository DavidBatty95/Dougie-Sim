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
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import String
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
except ModuleNotFoundError:  # Allows CI-safe mock benchmarking without a sourced ROS Python environment.
    rclpy = None
    PoseStamped = None
    Twist = None
    Odometry = None
    ActionClient = None
    String = None
    NavigateToPose = None
    ComputePathToPose = None
    Node = object

from .config import BenchmarkConfig
from .geometry import Pose2D, Velocity2D, distance_xy
from .goal_sampler import Goal, GoalSampler
from .metrics import MetricsMonitor
from .people_manager import PeopleManager
from .person_motion_models import PersonState, update_person
from .result_writer import ResultWriter
from .scoring import ScoreBreakdown, score_benchmark


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
        self.latest_robot_pose: Pose2D | None = None
        self.latest_robot_velocity = Velocity2D()
        self._last_odom_pose: Pose2D | None = None
        self._last_odom_stamp: float | None = None
        self._derived_robot_velocity = Velocity2D()
        self.latest_people: list[PersonState] = []
        self.latest_people_stamp = None
        self.latest_raw_cmd_stamp: float | None = None
        self.latest_raw_cmd_magnitude = 0.0
        self.max_abs_safe_lateral_cmd_mps = 0.0
        self.latest_locomotion: dict | None = None
        self.nav_goal_handle = None
        if self.ros_enabled:
            super().__init__("person_avoidance_benchmark")
            self.people_pub = self.create_publisher(String, "/benchmark/people_states", 10)
            self.metrics_pub = self.create_publisher(String, "/benchmark/metrics", 10)
            self.result_pub = self.create_publisher(String, "/benchmark/result", 10)
            self.goal_pub = self.create_publisher(PoseStamped, "/benchmark/current_goal", 10)
            self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
            self.create_subscription(String, "/mujoco/pedestrians", self._people_callback, 10)
            self.create_subscription(Twist, "/cmd_vel", self._raw_cmd_callback, 10)
            self.create_subscription(Twist, "/cmd_vel_safe", self._safe_cmd_callback, 10)
            self.create_subscription(String, "/g1/locomotion_diagnostics", self._locomotion_callback, 10)
            self.create_subscription(String, "/human_collision_avoidance_debug", self._safety_debug_callback, 10)
            self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose") if NavigateToPose is not None else None
            self.path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose") if ComputePathToPose is not None else None

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

    def _odom_callback(self, msg) -> None:
        pose = msg.pose.pose
        yaw = _yaw_from_quat(pose.orientation)
        current_pose = Pose2D(float(pose.position.x), float(pose.position.y), yaw)
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp <= 0.0:
            stamp = time.monotonic()
        reported = Velocity2D(
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.angular.z),
        )
        if self._last_odom_pose is None or self._last_odom_stamp is None:
            self._last_odom_pose = current_pose
            self._last_odom_stamp = stamp
        elif stamp - self._last_odom_stamp >= 0.25:
            derived = velocity_from_pose_delta(self._last_odom_pose, current_pose, stamp - self._last_odom_stamp)
            plausible_limit = max(1.0, self.config.max_speed_mps * 2.0)
            if derived is not None and math.hypot(derived.vx, derived.vy) <= plausible_limit:
                self._derived_robot_velocity = derived
            self._last_odom_pose = current_pose
            self._last_odom_stamp = stamp
        self.latest_robot_velocity = (
            self._derived_robot_velocity
            if math.hypot(reported.vx, reported.vy) < 0.005
            else reported
        )
        self.latest_robot_pose = current_pose

    def _people_callback(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        people = []
        for item in data.get("people", []):
            person_id = int(item.get("id", len(people) + 1))
            people.append(
                PersonState(
                    person_id=person_id,
                    pose=Pose2D(float(item.get("x", 0.0)), float(item.get("y", 0.0)), 0.0),
                    velocity=Velocity2D(float(item.get("vx", 0.0)), float(item.get("vy", 0.0)), 0.0),
                    radius_m=float(item.get("radius_m", self.config.person_radius_m)),
                    behaviour="mujoco",
                    speed_mps=math.hypot(float(item.get("vx", 0.0)), float(item.get("vy", 0.0))),
                )
            )
        self.latest_people = people
        self.latest_people_stamp = self.get_clock().now()

    def _raw_cmd_callback(self, msg) -> None:
        self.latest_raw_cmd_stamp = time.monotonic()
        self.latest_raw_cmd_magnitude = (
            math.hypot(float(msg.linear.x), float(msg.linear.y)) + abs(float(msg.angular.z))
        )

    def _safe_cmd_callback(self, msg) -> None:
        self.max_abs_safe_lateral_cmd_mps = max(
            self.max_abs_safe_lateral_cmd_mps,
            abs(float(msg.linear.y)),
        )

    def _locomotion_callback(self, msg) -> None:
        try:
            self.latest_locomotion = json.loads(msg.data)
            self.live_events.update(self.latest_locomotion)
        except json.JSONDecodeError:
            return

    def _safety_debug_callback(self, msg) -> None:
        try:
            self.live_events.update(json.loads(msg.data))
        except json.JSONDecodeError:
            return

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
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            people_ready = self.config.people_count == 0 or self.latest_people_stamp is not None
            if self.latest_robot_pose is not None and people_ready:
                return
        required = "/odom" if self.config.people_count == 0 else "/odom and /mujoco/pedestrians"
        raise RuntimeError(f"Timed out waiting for live {required} state")

    def wait_for_safe_start(self, timeout_s: float = 20.0) -> None:
        if not self.ros_enabled:
            return
        deadline = time.monotonic() + timeout_s
        last_warning = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
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
        """Return true only when Nav2 produces a nonempty path."""
        if self.path_client is None or not self.path_client.wait_for_server(timeout_sec=timeout_s):
            return False
        request = ComputePathToPose.Goal()
        request.use_start = True
        request.start = self._pose_stamped(start)
        request.goal = self._pose_stamped(goal)
        future = self.path_client.send_goal_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or future.result() is None or not future.result().accepted:
            return False
        result_future = future.result().get_result_async()
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result_future.done() or result_future.result() is None:
            return False
        has_path = len(result_future.result().result.path.poses) > 0
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
        if not future.done():
            self.get_logger().warn("NavigateToPose goal was not accepted before startup timeout")
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("NavigateToPose goal was rejected")
            return False
        self.nav_goal_handle = goal_handle
        return True

    def cancel_nav_goal(self, timeout_s: float = 2.0) -> None:
        """Release the active Nav2 action before advancing benchmark goals."""
        goal_handle = self.nav_goal_handle
        self.nav_goal_handle = None
        if goal_handle is None:
            return
        future = goal_handle.cancel_goal_async()
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_motion_cmd_after(self, stamp: float, timeout_s: float = 15.0) -> bool:
        if not self.ros_enabled:
            return True
        deadline = time.monotonic() + timeout_s
        last_warning = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
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
        requested_people_count = self.config.people_count
        if requested_people_count == 0:
            node.latest_people = []
        live_people_count = len(node.latest_people)
        if requested_people_count > 0 and live_people_count != requested_people_count:
            node.get_logger().warn(
                f"Requested {self.config.people_count} people, but live MuJoCo scene publishes {live_people_count}; "
                "scoring the live scene count."
            )
            self.config.people_count = live_people_count
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
                goals = sampler.sample(start, node.latest_people)
            except RuntimeError as exc:
                return self._finish_invalid(monitor, [], node.latest_people, f"goal_sampling_failed: {exc}")
        previous_goal_pose = start
        scenario_path = self.writer.write_scenario(self.config, goals, node.latest_people)
        start_wall = time.monotonic()
        last_sample = start_wall

        for goal_index in range(self.config.goals_count):
            if dynamic_random_goals:
                current_pose = node.latest_robot_pose or start
                try:
                    goal = sampler.sample_random_goal(goal_index, current_pose, node.latest_people)
                except RuntimeError as exc:
                    if not goals:
                        return self._finish_invalid(
                            monitor, goals, node.latest_people, f"goal_sampling_failed: {exc}"
                        )
                    break
                goals.append(goal)
                scenario_path = self.writer.write_scenario(self.config, goals, node.latest_people)
            else:
                goal = goals[goal_index]
            current_pose = node.latest_robot_pose or start
            if not node.wait_for_nav_path(current_pose, goal.pose):
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
            for attempt in range(2):
                goal_accepted = node.send_nav_goal(goal)
                if not goal_accepted:
                    break
                if node.wait_for_motion_cmd_after(time.monotonic()):
                    motion_started = True
                    break
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
            if not motion_started:
                if monitor.metrics.total_time_s <= 0.0 and monitor.metrics.goals_reached == 0:
                    return self._finish_invalid(
                        monitor, goals, node.latest_people, "no_nonzero_cmd_vel_before_scoring_start"
                    )
                monitor.complete_goal(goal_metrics, False, "no_nonzero_cmd_vel_before_start", 0.0)
                return self._finish(monitor, goals, node.latest_people, scenario_path), str(self.writer.output_path)
            goal_start_time = time.monotonic()
            if monitor.metrics.total_time_s <= 0.0:
                node.live_events.reset()
                start_wall = goal_start_time
                last_sample = goal_start_time

            while _ok():
                rclpy.spin_once(node, timeout_sec=max(0.0, min(self.config.dt_s, 0.1)))
                now_wall = time.monotonic()
                now_s = now_wall - start_wall
                dt = max(1e-3, now_wall - last_sample)
                if dt < self.config.dt_s * 0.5:
                    continue
                last_sample = now_wall
                if now_s > self.config.total_timeout_s:
                    monitor.complete_goal(goal_metrics, False, "benchmark_timeout", now_wall - goal_start_time)
                    return self._finish(monitor, goals, node.latest_people, scenario_path), str(self.writer.output_path)

                robot_pose = node.latest_robot_pose
                if robot_pose is None:
                    continue
                people = node.latest_people[: self.config.people_count] if self.config.people_count > 0 else []
                previous_path_length = monitor.metrics.actual_path_length_m
                monitor.update(now_s, dt, robot_pose, node.latest_robot_velocity, people)
                step_length = monitor.metrics.actual_path_length_m - previous_path_length
                if node.live_events._override_active:
                    monitor.metrics.avoidance_time_s += dt
                    if any(math.hypot(person.velocity.vx, person.velocity.vy) >= 0.10 for person in people):
                        monitor.metrics.moving_avoidance_time_s += dt
                elif node.live_events.avoidance_interventions > 0:
                    monitor.metrics.post_avoidance_progress_m += step_length
                monitor.metrics.emergency_stops = node.live_events.emergency_stops
                monitor.metrics.avoidance_interventions = node.live_events.avoidance_interventions
                monitor.metrics.stale_events = node.live_events.stale_events
                monitor.metrics.max_abs_safe_lateral_cmd_mps = node.max_abs_safe_lateral_cmd_mps
                node.publish_people(people)
                node.publish_metrics(
                    {
                        "time_s": now_s,
                        "goal_index": goal.index,
                        "goals_reached": monitor.metrics.goals_reached,
                        "collisions": monitor.metrics.collisions,
                        "backend": "mujoco",
                    }
                )

                if distance_xy(robot_pose, goal.pose) <= self.config.goal_tolerance_m:
                    node.cancel_nav_goal()
                    monitor.complete_goal(goal_metrics, True, None, now_wall - goal_start_time)
                    break
                if now_wall - goal_start_time > self.config.max_goal_time_s:
                    monitor.complete_goal(goal_metrics, False, "goal_timeout", now_wall - goal_start_time)
                    break
                if monitor.metrics.collisions > 0:
                    monitor.complete_goal(goal_metrics, False, "collision", now_wall - goal_start_time)
                    return self._finish(monitor, goals, people, scenario_path), str(self.writer.output_path)
                if node.robot_fallen():
                    monitor.metrics.fallen = True
                    monitor.complete_goal(goal_metrics, False, "fallen", now_wall - goal_start_time)
                    return self._finish(monitor, goals, people, scenario_path), str(self.writer.output_path)

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
        monitor.metrics.goal_metrics.clear()
        monitor.metrics.ideal_path_length_m = 0.0
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
