import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from unitree_hg.msg import LowState

from .g1_constants import G1_29DOF_JOINTS

try:
    import torch
except ImportError:  # pragma: no cover - reported at runtime on systems without torch.
    torch = None

try:
    import onnxruntime as ort
except ImportError:  # pragma: no cover - optional deployment dependency.
    ort = None


LOWER_BODY_JOINTS = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]

LATEST_RELIABLE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

DEFAULT_JOINT_POS = {
    ".*_hip_pitch_joint": -0.312,
    ".*_knee_joint": 0.669,
    ".*_ankle_pitch_joint": -0.363,
    ".*_elbow_joint": 0.6,
    "left_shoulder_roll_joint": 0.2,
    "left_shoulder_pitch_joint": 0.2,
    "right_shoulder_roll_joint": -0.2,
    "right_shoulder_pitch_joint": 0.2,
}

ACTION_SCALE = {
    ".*_hip_pitch_joint": 0.5475464629598068,
    ".*_hip_yaw_joint": 0.5475464629598068,
    ".*_hip_roll_joint": 0.35066146635877904,
    ".*_knee_joint": 0.35066146635877904,
    ".*_ankle_pitch_joint": 0.43857731389829635,
    ".*_ankle_roll_joint": 0.43857731389829635,
    "waist_yaw_joint": 0.5475464629598068,
    "waist_roll_joint": 0.43857731389829635,
    "waist_pitch_joint": 0.43857731389829635,
    ".*_shoulder_pitch_joint": 0.43857731389829635,
    ".*_shoulder_roll_joint": 0.43857731389829635,
    ".*_shoulder_yaw_joint": 0.43857731389829635,
    ".*_elbow_joint": 0.43857731389829635,
    ".*_wrist_roll_joint": 0.43857731389829635,
    ".*_wrist_pitch_joint": 1.0,
    ".*_wrist_yaw_joint": 1.0,
}


def _pattern_value(name: str, patterns: dict[str, float], default: float = 0.0) -> float:
    for pattern, value in patterns.items():
        if pattern.startswith(".*") and name.endswith(pattern[2:]):
            return value
        if pattern == name:
            return value
    return default


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _parse_float_csv(value: str) -> List[float]:
    if not value:
        return []
    result = []
    for item in value.split(","):
        item = item.strip()
        if item:
            result.append(float(item))
    return result


def _parse_str_csv(value: str) -> List[str]:
    if not value:
        return []
    return [item.strip() for item in value.split(",") if item.strip()]


def _quat_conjugate(quat: Sequence[float]) -> tuple[float, float, float, float]:
    return quat[0], -quat[1], -quat[2], -quat[3]


def _quat_rotate(quat: Sequence[float], vec: Sequence[float]) -> List[float]:
    w, x, y, z = quat
    vx, vy, vz = vec
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    ]


@dataclass(frozen=True)
class PolicyInferenceCandidate:
    obs_dim: int
    action_dim: int
    observation: tuple[float, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "observation", tuple(self.observation))


@dataclass(frozen=True)
class PolicyInferenceRequest:
    generation: int
    sequence: int
    submitted_at: float
    deadline: float
    candidates: tuple[PolicyInferenceCandidate, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "candidates", tuple(self.candidates))


@dataclass(frozen=True)
class PolicyInferenceResult:
    generation: int
    sequence: int
    submitted_at: float
    completed_at: float
    obs_dim: Optional[int] = None
    action_dim: Optional[int] = None
    actions: tuple[float, ...] = ()
    error: Optional[str] = None
    deadline_missed: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "actions", tuple(self.actions))


@dataclass(frozen=True)
class PolicyInferenceDiagnostics:
    submitted: int
    started: int
    completed: int
    succeeded: int
    failed: int
    expired: int
    superseded: int
    invalidated: int
    discarded: int


class PolicyInferenceWorker:
    """Run one inference at a time without queueing stale observations."""

    def __init__(self, policy) -> None:
        self._policy = policy
        self._condition = threading.Condition()
        self._generation = 0
        self._sequence = 0
        self._pending: Optional[PolicyInferenceRequest] = None
        self._active: Optional[PolicyInferenceRequest] = None
        self._result: Optional[PolicyInferenceResult] = None
        self._stopping = False
        self._submitted = 0
        self._started = 0
        self._completed = 0
        self._succeeded = 0
        self._failed = 0
        self._expired = 0
        self._superseded = 0
        self._invalidated = 0
        self._discarded = 0
        self._thread = threading.Thread(
            target=self._run, name="g1-policy-inference", daemon=True
        )
        self._thread.start()

    @property
    def thread(self) -> threading.Thread:
        return self._thread

    def submit(
        self, candidates: Sequence[PolicyInferenceCandidate], deadline: float
    ) -> Optional[PolicyInferenceRequest]:
        with self._condition:
            if self._stopping:
                raise RuntimeError("Policy inference worker is shut down")
            if self._pending is not None or self._active is not None or self._result is not None:
                return None
            self._sequence += 1
            request = PolicyInferenceRequest(
                generation=self._generation,
                sequence=self._sequence,
                submitted_at=time.monotonic(),
                deadline=float(deadline),
                candidates=tuple(candidates),
            )
            self._pending = request
            self._submitted += 1
            self._condition.notify()
            return request

    def take_result(self) -> Optional[PolicyInferenceResult]:
        with self._condition:
            result = self._result
            self._result = None
            return result

    def deadline_missed(self, now: Optional[float] = None) -> bool:
        current = time.monotonic() if now is None else now
        with self._condition:
            requests = (self._active, self._pending)
            return any(
                request is not None
                and request.generation == self._generation
                and current > request.deadline
                for request in requests
            )

    def invalidate_expired(self, now: Optional[float] = None) -> bool:
        current = time.monotonic() if now is None else now
        with self._condition:
            requests = (self._active, self._pending)
            if not any(
                request is not None
                and request.generation == self._generation
                and current > request.deadline
                for request in requests
            ):
                return False
            self._generation += 1
            self._pending = None
            self._result = None
            self._expired += 1
            self._invalidated += 1
            return True

    def invalidate(self) -> int:
        with self._condition:
            self._generation += 1
            self._pending = None
            self._result = None
            self._invalidated += 1
            return self._generation

    def diagnostics(self) -> PolicyInferenceDiagnostics:
        with self._condition:
            return PolicyInferenceDiagnostics(
                submitted=self._submitted,
                started=self._started,
                completed=self._completed,
                succeeded=self._succeeded,
                failed=self._failed,
                expired=self._expired,
                superseded=self._superseded,
                invalidated=self._invalidated,
                discarded=self._discarded,
            )

    def shutdown(self, timeout: float = 1.0) -> bool:
        with self._condition:
            if not self._stopping:
                self._stopping = True
                self._generation += 1
                self._pending = None
                self._result = None
                self._invalidated += 1
                self._condition.notify_all()
        self._thread.join(max(0.0, timeout))
        return not self._thread.is_alive()

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._stopping:
                    self._condition.wait()
                if self._stopping:
                    return
                request = self._pending
                self._pending = None
                self._active = request
                self._started += 1

            assert request is not None
            result = self._infer(request)
            with self._condition:
                self._active = None
                self._completed += 1
                if self._stopping or request.generation != self._generation:
                    self._discarded += 1
                    continue
                if result.deadline_missed:
                    self._expired += 1
                    self._generation += 1
                    self._pending = None
                    self._invalidated += 1
                elif result.error is not None:
                    self._failed += 1
                    self._generation += 1
                    self._pending = None
                    self._invalidated += 1
                else:
                    self._succeeded += 1
                if self._result is not None:
                    self._superseded += 1
                self._result = result

    def _infer(self, request: PolicyInferenceRequest) -> PolicyInferenceResult:
        if time.monotonic() > request.deadline:
            return self._deadline_result(request)

        last_error: Optional[Exception] = None
        for candidate in request.candidates:
            if time.monotonic() > request.deadline:
                return self._deadline_result(request)
            try:
                observation = np.asarray(
                    [candidate.observation], dtype=np.float32
                )
                if observation.shape != (1, candidate.obs_dim):
                    raise ValueError(
                        f"Observation shape {observation.shape} does not match "
                        f"(1, {candidate.obs_dim})"
                    )
                actions = self._normalize_actions(
                    self._policy(observation), candidate.action_dim
                )
                completed_at = time.monotonic()
                if completed_at > request.deadline:
                    return self._deadline_result(request, completed_at)
                return PolicyInferenceResult(
                    generation=request.generation,
                    sequence=request.sequence,
                    submitted_at=request.submitted_at,
                    completed_at=completed_at,
                    obs_dim=candidate.obs_dim,
                    action_dim=candidate.action_dim,
                    actions=actions,
                )
            except Exception as exc:  # noqa: BLE001 - try the alternate shape.
                last_error = exc

        if time.monotonic() > request.deadline:
            return self._deadline_result(request)
        return PolicyInferenceResult(
            generation=request.generation,
            sequence=request.sequence,
            submitted_at=request.submitted_at,
            completed_at=time.monotonic(),
            error=str(last_error or "No policy inference candidates"),
        )

    @staticmethod
    def _deadline_result(
        request: PolicyInferenceRequest, completed_at: Optional[float] = None
    ) -> PolicyInferenceResult:
        return PolicyInferenceResult(
            generation=request.generation,
            sequence=request.sequence,
            submitted_at=request.submitted_at,
            completed_at=time.monotonic() if completed_at is None else completed_at,
            error="Policy inference deadline missed",
            deadline_missed=True,
        )

    @staticmethod
    def _normalize_actions(action, action_dim: int) -> tuple[float, ...]:
        if isinstance(action, (tuple, list)):
            action = action[0]
        if torch is not None and isinstance(action, torch.Tensor):
            values = action.detach().to("cpu").reshape(-1).tolist()
        else:
            values = np.asarray(action).reshape(-1).tolist()
        if len(values) < action_dim:
            raise ValueError(
                f"Policy returned {len(values)} actions, expected {action_dim}"
            )
        normalized = tuple(float(value) for value in values[:action_dim])
        if not all(math.isfinite(value) for value in normalized):
            raise ValueError("Policy returned non-finite actions")
        return normalized


class G1PolicyCmdVelController(Node):
    def __init__(self) -> None:
        super().__init__("g1_policy_cmd_vel_controller")

        self.declare_parameter("policy_path", "")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("body_command_topic", "/g1/body_position_commands")
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("max_lin_vel_x", 0.5)
        self.declare_parameter("max_lin_vel_y", 0.25)
        self.declare_parameter("max_ang_vel_z", 0.5)
        self.declare_parameter("cmd_accel_x", 0.5)
        self.declare_parameter("cmd_accel_y", 0.35)
        self.declare_parameter("cmd_accel_yaw", 0.5)
        self.declare_parameter("action_clip", 10.0)
        self.declare_parameter("action_rate_limit", 0.0)
        self.declare_parameter("policy_obs_dim", 0)
        self.declare_parameter("policy_action_dim", 0)
        self.declare_parameter("policy_inference_deadline_sec", 0.1)
        self.declare_parameter("policy_command_height", 0.84)
        self.declare_parameter("stand_still_on_timeout", True)
        self.declare_parameter("startup_blend_time", 0.0)
        self.declare_parameter("startup_hold_time", 0.0)
        self.declare_parameter("startup_current_pose_hold_time", 0.0)
        self.declare_parameter("startup_require_upright", False)
        self.declare_parameter("startup_upright_tilt_threshold", 0.35)
        self.declare_parameter("startup_max_recovery_time", 8.0)
        self.declare_parameter("policy_action_ramp_time", 0.0)
        self.declare_parameter("tilt_guard_enable", True)
        self.declare_parameter("tilt_guard_enter", 0.65)
        self.declare_parameter("tilt_guard_exit", 0.35)
        self.declare_parameter("tilt_guard_hold_time", 0.8)
        self.declare_parameter("pre_lowstate_stand_enable", False)
        self.declare_parameter("detect_robot_reset", True)
        self.declare_parameter("reset_joint_delta_threshold", 0.75)
        self.declare_parameter("reset_joint_total_delta_threshold", 4.0)
        self.declare_parameter("reset_fallen_tilt_threshold", 0.95)
        self.declare_parameter("reset_upright_tilt_threshold", 0.45)
        self.declare_parameter("reset_detection_cooldown", 1.0)
        self.declare_parameter("device", "cpu")

        self.device = str(self.get_parameter("device").value)
        self.policy_path = self._resolve_policy_path(str(self.get_parameter("policy_path").value))
        self.policy = self._load_policy(self.policy_path)
        self.policy_obs_dim: Optional[int] = None
        self.policy_action_dim: Optional[int] = None
        policy_obs_dim = int(self.get_parameter("policy_obs_dim").value)
        policy_action_dim = int(self.get_parameter("policy_action_dim").value)
        if policy_obs_dim in (80, 82, 97, 99):
            self.policy_obs_dim = policy_obs_dim
        if policy_action_dim in (12, 29):
            self.policy_action_dim = policy_action_dim
        self.policy_command_names: list[str] = []
        self.last_lowstate: Optional[LowState] = None
        self.previous_joint_positions: Optional[List[float]] = None
        self.last_stable_positions: Optional[List[float]] = None
        self.previous_tilt: Optional[float] = None
        self.first_lowstate_time = None
        self.initial_positions: Optional[List[float]] = None
        self.upright_since = None
        self.last_reset_time = None
        self.tilt_guard_active = False
        self.tilt_guard_clear_since = None
        self.reset_count = 0
        self.last_cmd_time = self.get_clock().now()
        self.command = [0.0, 0.0, 0.0]
        self.filtered_command = [0.0, 0.0, 0.0]
        self.last_control_time = self.get_clock().now()
        self.first_policy_time = None
        self.cmd_vel_count = 0
        self.lowstate_count = 0
        self.body_command_count = 0
        self.last_action_norm = 0.0
        self.last_active_command = [0.0, 0.0, 0.0]
        self.last_tilt = 0.0
        self.control_state = "waiting_lowstate"
        self.base_lin_vel = [0.0, 0.0, 0.0]
        self.body_joint_names = list(G1_29DOF_JOINTS)
        self.lower_joint_indexes = [
            self.body_joint_names.index(j) for j in LOWER_BODY_JOINTS
        ]
        self.default_positions = [
            _pattern_value(name, DEFAULT_JOINT_POS) for name in self.body_joint_names
        ]
        self.scales = [
            _pattern_value(name, ACTION_SCALE, 0.25) for name in self.body_joint_names
        ]
        self.last_actions = {
            12: [0.0] * len(self.lower_joint_indexes),
            29: [0.0] * len(self.body_joint_names),
        }
        self._apply_policy_metadata()
        self._log_policy_configuration()
        self.policy_worker = PolicyInferenceWorker(self.policy)
        self.policy_inference_deadline = max(
            0.001,
            float(self.get_parameter("policy_inference_deadline_sec").value),
        )

        self.body_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("body_command_topic").value),
            LATEST_RELIABLE_QOS,
        )
        self.create_subscription(
            LowState, "/lowstate", self._lowstate_callback, LATEST_RELIABLE_QOS
        )
        self.create_subscription(
            LowState, "rt/lowstate", self._lowstate_callback, LATEST_RELIABLE_QOS
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._cmd_vel_callback,
            LATEST_RELIABLE_QOS,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            LATEST_RELIABLE_QOS,
        )

        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.control_period = period
        self.create_timer(period, self._control_tick)
        self.create_timer(2.0, self._diagnostics)
        self.get_logger().info("G1 policy /cmd_vel controller ready")

    def _log_policy_configuration(self) -> None:
        command_mode = "height" if self._uses_height_command() else "velocity"
        action_scope = "lower-body" if self.policy_action_dim == 12 else "whole-body" if self.policy_action_dim == 29 else "unknown"
        self.get_logger().info(
            "Loaded G1 policy: "
            f"path={self.policy_path} obs_dim={self.policy_obs_dim} "
            f"action_dim={self.policy_action_dim} action_scope={action_scope} command_mode={command_mode} "
            f"command_names={self.policy_command_names or 'unknown'} "
            f"target_height={float(self.get_parameter('policy_command_height').value):.3f}"
        )

    def _load_policy(self, policy_path: str):
        if not policy_path:
            raise ValueError(
                "policy_path must point to a TorchScript, ONNX, or callable PyTorch policy"
            )

        path = Path(policy_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Policy file does not exist: {path}")

        if path.suffix == ".onnx":
            if ort is None:
                raise RuntimeError("onnxruntime is required to run ONNX policies")
            return OnnxPolicy(str(path))

        if torch is None:
            raise RuntimeError("PyTorch is required to run TorchScript/PyTorch policies")

        try:
            policy = torch.jit.load(str(path), map_location=self.device)
        except RuntimeError:
            policy = torch.load(str(path), map_location=self.device)
            if isinstance(policy, dict):
                for key in ("policy", "model", "actor"):
                    if key in policy and callable(policy[key]):
                        policy = policy[key]
                        break
        if not callable(policy):
            raise TypeError(
                "Policy file must be TorchScript or contain a callable policy/model/actor"
            )
        return TorchPolicy(policy, self.device)

    def _resolve_policy_path(self, policy_path: str) -> str:
        path = policy_path.strip()
        if path and path.lower() not in ("latest", "auto"):
            return path

        roots = [
            Path("/workspace/vendor/mjlab/logs/rsl_rl/g1_velocity"),
            Path.cwd() / "vendor/mjlab/logs/rsl_rl/g1_velocity",
        ]
        preferred: list[Path] = []
        fallback: list[Path] = []
        for root in roots:
            if not root.exists():
                continue
            preferred.extend(root.glob("*g1_flat_locomotion/*.onnx"))
            preferred.extend(root.glob("*flat_locomotion/*.onnx"))
            fallback.extend(root.glob("*g1_flat_lower_body_locomotion/*.onnx"))
            fallback.extend(root.glob("*lower_body_locomotion/*.onnx"))

        candidates = [candidate for candidate in preferred if candidate.is_file()]
        if not candidates:
            candidates = [candidate for candidate in fallback if candidate.is_file()]
        if candidates:
            latest = max(candidates, key=lambda candidate: candidate.stat().st_mtime)
            self.get_logger().info(f"Resolved latest G1 walking policy: {latest}")
            return str(latest)

        raise FileNotFoundError(
            "No G1 walking policy ONNX was found. Train/export a lower-body locomotion "
            "policy into vendor/mjlab/logs/rsl_rl/g1_velocity, or launch with "
            "policy_path:=/workspace/path/to/policy.onnx."
        )

    def _apply_policy_metadata(self) -> None:
        metadata = getattr(self.policy, "metadata", {})
        if not metadata:
            return

        default_joint_pos = _parse_float_csv(metadata.get("default_joint_pos", ""))
        action_scale = _parse_float_csv(metadata.get("action_scale", ""))
        joint_names = _parse_str_csv(metadata.get("joint_names", ""))
        self.policy_command_names = _parse_str_csv(metadata.get("command_names", ""))
        if joint_names and joint_names != self.body_joint_names and joint_names != LOWER_BODY_JOINTS:
            self.get_logger().warn(
                "Policy joint metadata order differs from bridge order; using bridge order",
                throttle_duration_sec=5.0,
            )
        if len(default_joint_pos) == len(self.body_joint_names):
            self.default_positions = default_joint_pos
        elif len(default_joint_pos) == len(self.lower_joint_indexes):
            if joint_names and len(joint_names) == len(default_joint_pos):
                for name, value in zip(joint_names, default_joint_pos):
                    if name in self.body_joint_names:
                        self.default_positions[self.body_joint_names.index(name)] = value
            else:
                for value, joint_index in zip(default_joint_pos, self.lower_joint_indexes):
                    self.default_positions[joint_index] = value
        if len(action_scale) == len(self.lower_joint_indexes):
            if joint_names and len(joint_names) == len(action_scale):
                for name, value in zip(joint_names, action_scale):
                    if name in self.body_joint_names:
                        self.scales[self.body_joint_names.index(name)] = value
            else:
                for action_index, joint_index in enumerate(self.lower_joint_indexes):
                    self.scales[joint_index] = action_scale[action_index]
        elif len(action_scale) == len(self.body_joint_names):
            self.scales = action_scale

        obs_dim = getattr(self.policy, "obs_dim", None)
        action_dim = getattr(self.policy, "action_dim", None)
        if self.policy_obs_dim is None and obs_dim in (80, 82, 97, 99):
            self.policy_obs_dim = int(obs_dim)
        if self.policy_action_dim is None and action_dim in (12, 29):
            self.policy_action_dim = int(action_dim)

    def _lowstate_callback(self, msg: LowState) -> None:
        if len(msg.motor_state) < len(self.body_joint_names):
            self.get_logger().warn(
                f"Ignoring short lowstate with {len(msg.motor_state)} motor states",
                throttle_duration_sec=2.0,
            )
            return
        if (
            len(msg.imu_state.rpy) < 2
            or len(msg.imu_state.quaternion) < 4
            or len(msg.imu_state.gyroscope) < 3
        ):
            self.get_logger().warn(
                "Ignoring lowstate with incomplete IMU data", throttle_duration_sec=2.0
            )
            return
        joint_positions = [float(state.q) for state in msg.motor_state[:29]]
        joint_velocities = [float(state.dq) for state in msg.motor_state[:29]]
        imu_values = [float(value) for value in msg.imu_state.rpy[:2]]
        imu_values.extend(float(value) for value in msg.imu_state.quaternion[:4])
        imu_values.extend(float(value) for value in msg.imu_state.gyroscope[:3])
        if not all(math.isfinite(value) for value in joint_positions + joint_velocities + imu_values):
            self.get_logger().warn(
                "Ignoring lowstate containing non-finite values", throttle_duration_sec=2.0
            )
            return
        self.lowstate_count += 1
        tilt = self._lowstate_tilt(msg)
        self.last_tilt = tilt
        if self._detect_robot_reset(joint_positions, tilt):
            self._reset_controller_state(joint_positions, "reset/upright transition")
        if self.first_lowstate_time is None:
            self.first_lowstate_time = self.get_clock().now()
            self.initial_positions = list(joint_positions)
        if tilt < float(self.get_parameter("startup_upright_tilt_threshold").value):
            self.last_stable_positions = list(joint_positions)
        self._update_upright_state(tilt)
        self.previous_joint_positions = list(joint_positions)
        self.previous_tilt = tilt
        self.last_lowstate = msg

    def _tilt_guard_holding(self) -> bool:
        if not bool(self.get_parameter("tilt_guard_enable").value):
            return False
        now = self.get_clock().now()
        enter = float(self.get_parameter("tilt_guard_enter").value)
        exit_threshold = float(self.get_parameter("tilt_guard_exit").value)
        hold_time = float(self.get_parameter("tilt_guard_hold_time").value)
        if self.last_tilt >= enter:
            if not self.tilt_guard_active:
                self.get_logger().warn(
                    f"Tilt guard holding stable joint pose (tilt={self.last_tilt:.2f} rad)",
                    throttle_duration_sec=1.0,
                )
                self.policy_worker.invalidate()
            self.tilt_guard_active = True
            self.tilt_guard_clear_since = None
            self.filtered_command = [0.0, 0.0, 0.0]
            self.first_policy_time = None
            self.last_actions = {
                12: [0.0] * len(self.lower_joint_indexes),
                29: [0.0] * len(self.body_joint_names),
            }
            self.last_action_norm = 0.0
            return True
        if not self.tilt_guard_active:
            return False
        if self.last_tilt <= exit_threshold:
            if self.tilt_guard_clear_since is None:
                self.tilt_guard_clear_since = now
            stable_time = (now - self.tilt_guard_clear_since).nanoseconds * 1e-9
            if stable_time >= hold_time:
                self.tilt_guard_active = False
                self.tilt_guard_clear_since = None
                self.first_policy_time = None
                return False
        else:
            self.tilt_guard_clear_since = None
        return True

    def _update_upright_state(self, tilt: float) -> None:
        if tilt < float(self.get_parameter("startup_upright_tilt_threshold").value):
            if self.upright_since is None:
                self.upright_since = self.get_clock().now()
        else:
            self.upright_since = None

    def _detect_robot_reset(self, joint_positions: Sequence[float], tilt: float) -> bool:
        if not bool(self.get_parameter("detect_robot_reset").value):
            return False
        now = self.get_clock().now()
        if self.last_reset_time is not None:
            cooldown = float(self.get_parameter("reset_detection_cooldown").value)
            if (now - self.last_reset_time).nanoseconds * 1e-9 < cooldown:
                return False

        if self.previous_tilt is not None:
            if (
                self.previous_tilt > float(self.get_parameter("reset_fallen_tilt_threshold").value)
                and tilt < float(self.get_parameter("reset_upright_tilt_threshold").value)
            ):
                return True

        if self.previous_joint_positions is None:
            return False

        deltas = [abs(current - previous) for current, previous in zip(joint_positions, self.previous_joint_positions)]
        if not deltas:
            return False
        max_delta = max(deltas)
        total_delta = sum(deltas)
        return (
            max_delta > float(self.get_parameter("reset_joint_delta_threshold").value)
            and total_delta > float(self.get_parameter("reset_joint_total_delta_threshold").value)
        )

    @staticmethod
    def _lowstate_tilt(msg: LowState) -> float:
        rpy = msg.imu_state.rpy
        roll = math.atan2(math.sin(float(rpy[0])), math.cos(float(rpy[0])))
        pitch = math.atan2(math.sin(float(rpy[1])), math.cos(float(rpy[1])))
        return max(abs(roll), abs(pitch))

    def _reset_controller_state(self, joint_positions: Sequence[float], reason: str) -> None:
        now = self.get_clock().now()
        self.policy_worker.invalidate()
        self.first_lowstate_time = now
        self.initial_positions = list(joint_positions)
        self.last_stable_positions = list(joint_positions)
        self.upright_since = None
        self.filtered_command = [0.0, 0.0, 0.0]
        self.last_active_command = [0.0, 0.0, 0.0]
        self.last_control_time = now
        self.first_policy_time = None
        self.last_actions = {
            12: [0.0] * len(self.lower_joint_indexes),
            29: [0.0] * len(self.body_joint_names),
        }
        self.last_action_norm = 0.0
        self.last_reset_time = now
        self.reset_count += 1
        self.get_logger().warn(
            f"Detected robot reset from {reason}; restarting policy startup blend",
            throttle_duration_sec=1.0,
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        values = [float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)]
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn(
                "Ignoring cmd_vel containing non-finite values", throttle_duration_sec=2.0
            )
            return
        self.cmd_vel_count += 1
        self.last_cmd_time = self.get_clock().now()
        self.command = [
            _clamp(values[0], float(self.get_parameter("max_lin_vel_x").value)),
            _clamp(values[1], float(self.get_parameter("max_lin_vel_y").value)),
            _clamp(values[2], float(self.get_parameter("max_ang_vel_z").value)),
        ]

    def _odom_callback(self, msg: Odometry) -> None:
        values = [
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z),
        ]
        if all(math.isfinite(value) for value in values):
            self.base_lin_vel = values

    def _active_command(self) -> List[float]:
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > float(self.get_parameter("cmd_vel_timeout").value):
            if bool(self.get_parameter("stand_still_on_timeout").value):
                return [0.0, 0.0, 0.0]
        return list(self.command)

    def _filtered_active_command(self, target: Sequence[float], dt: float) -> List[float]:
        limits = [
            float(self.get_parameter("cmd_accel_x").value),
            float(self.get_parameter("cmd_accel_y").value),
            float(self.get_parameter("cmd_accel_yaw").value),
        ]
        for index, limit in enumerate(limits):
            max_delta = max(0.0, limit) * dt
            delta = float(target[index]) - self.filtered_command[index]
            if delta > max_delta:
                delta = max_delta
            elif delta < -max_delta:
                delta = -max_delta
            self.filtered_command[index] += delta
        return list(self.filtered_command)

    def _control_tick(self) -> None:
        if self.last_lowstate is None:
            if bool(self.get_parameter("pre_lowstate_stand_enable").value):
                self._publish_targets(self.default_positions)
            self.get_logger().warn(
                "Waiting for /lowstate before releasing MuJoCo startup hold",
                throttle_duration_sec=2.0,
            )
            return

        now = self.get_clock().now()
        dt = max(1e-3, min(0.1, (now - self.last_control_time).nanoseconds * 1e-9))
        self.last_control_time = now
        command = self._filtered_active_command(self._active_command(), dt)
        self.last_active_command = command
        if self._startup_active():
            self.control_state = "startup_hold"
            self._publish_startup_pose()
            return
        if self._tilt_guard_holding():
            self.control_state = "tilt_guard_hold"
            self._publish_targets(self._safe_hold_positions())
            return

        result = self.policy_worker.take_result()
        if result is not None and (result.error is not None or result.deadline_missed):
            self.get_logger().error(
                result.error or "Policy inference deadline missed",
                throttle_duration_sec=2.0,
            )
            state = "policy_deadline_hold" if result.deadline_missed else "policy_error_hold"
            self._policy_safe_hold(state)
            return
        if self.policy_worker.invalidate_expired():
            self.get_logger().error(
                "Policy inference deadline missed", throttle_duration_sec=2.0
            )
            self._policy_safe_hold("policy_deadline_hold")
            return

        if result is not None:
            assert result.obs_dim is not None and result.action_dim is not None
            self.policy_obs_dim = result.obs_dim
            self.policy_action_dim = result.action_dim
            if self.first_policy_time is None:
                self.first_policy_time = now
            self.control_state = "policy"
            ramp_time = max(
                0.0, float(self.get_parameter("policy_action_ramp_time").value)
            )
            if ramp_time > 0.0:
                ramp = min(
                    1.0,
                    (now - self.first_policy_time).nanoseconds * 1e-9 / ramp_time,
                )
            else:
                ramp = 1.0
            try:
                self._publish_action(
                    np.asarray(result.actions, dtype=np.float32),
                    result.action_dim,
                    dt,
                    ramp,
                )
            except Exception as exc:  # noqa: BLE001 - hold safely on output handling errors.
                self.policy_worker.invalidate()
                self.get_logger().error(
                    f"Policy action handling failed: {exc}",
                    throttle_duration_sec=2.0,
                )
                self._policy_safe_hold("policy_error_hold")
                return

        candidates = self._build_inference_candidates(command)
        self.policy_worker.submit(
            candidates, deadline=time.monotonic() + self.policy_inference_deadline
        )
        if result is None:
            self.control_state = "policy_wait"
            if self.first_policy_time is None:
                self._publish_targets(self._safe_hold_positions())

    def _policy_safe_hold(self, state: str) -> None:
        self.control_state = state
        self.filtered_command = [0.0, 0.0, 0.0]
        self.last_actions = {
            12: [0.0] * len(self.lower_joint_indexes),
            29: [0.0] * len(self.body_joint_names),
        }
        self.last_action_norm = 0.0
        self.first_policy_time = None
        self._publish_targets(self._safe_hold_positions())

    def _build_inference_candidates(
        self, command: Sequence[float]
    ) -> tuple[PolicyInferenceCandidate, ...]:
        obs_dims = (
            (self.policy_obs_dim,)
            if self.policy_obs_dim is not None
            else (80, 82, 97, 99)
        )
        candidates = []
        for obs_dim in obs_dims:
            action_dim = self.policy_action_dim or (
                12 if obs_dim in (80, 82) else 29
            )
            observation = self._build_observation(command, action_dim, obs_dim)
            candidates.append(
                PolicyInferenceCandidate(
                    obs_dim=obs_dim,
                    action_dim=action_dim,
                    observation=tuple(float(value) for value in observation.reshape(-1)),
                )
            )
        return tuple(candidates)

    def _startup_active(self) -> bool:
        if self.first_lowstate_time is None:
            return True
        elapsed = (self.get_clock().now() - self.first_lowstate_time).nanoseconds * 1e-9
        startup_time = float(self.get_parameter("startup_current_pose_hold_time").value)
        startup_time += float(self.get_parameter("startup_blend_time").value)
        startup_time += float(self.get_parameter("startup_hold_time").value)
        if elapsed < startup_time:
            return True
        if not bool(self.get_parameter("startup_require_upright").value):
            return False
        max_recovery = float(self.get_parameter("startup_max_recovery_time").value)
        if max_recovery > 0.0 and elapsed > max_recovery:
            return False
        if self.upright_since is None:
            self.get_logger().warn(
                "Holding stable joint pose until robot is upright before enabling policy inference",
                throttle_duration_sec=1.0,
            )
            return True
        stable_time = (self.get_clock().now() - self.upright_since).nanoseconds * 1e-9
        return stable_time < float(self.get_parameter("startup_hold_time").value)

    def _publish_startup_pose(self) -> None:
        if self.first_lowstate_time is None or self.initial_positions is None:
            return
        elapsed = (self.get_clock().now() - self.first_lowstate_time).nanoseconds * 1e-9
        current_hold = max(0.0, float(self.get_parameter("startup_current_pose_hold_time").value))
        if elapsed < current_hold:
            self._publish_targets(self._safe_hold_positions())
            return
        elapsed -= current_hold
        blend_time = max(float(self.get_parameter("startup_blend_time").value), 1e-6)
        alpha = min(1.0, max(0.0, elapsed / blend_time))
        alpha = alpha * alpha * (3.0 - 2.0 * alpha)
        targets = [
            (1.0 - alpha) * start + alpha * target
            for start, target in zip(self.initial_positions, self.default_positions)
        ]
        self._publish_targets(targets)

    def _safe_hold_positions(self) -> List[float]:
        if self.last_stable_positions is not None:
            return list(self.last_stable_positions)
        if self.initial_positions is not None:
            return list(self.initial_positions)
        return list(self.default_positions)

    def _diagnostics(self) -> None:
        cmd_age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        inference = self.policy_worker.diagnostics()
        self.get_logger().info(
            "policy status: "
            f"state={self.control_state} lowstate={self.lowstate_count} cmd_vel={self.cmd_vel_count} "
            f"body_cmd={self.body_command_count} resets={self.reset_count} cmd_age={cmd_age:.2f}s "
            f"tilt={self.last_tilt:.2f} "
            f"cmd=({self.last_active_command[0]:+.2f}, "
            f"{self.last_active_command[1]:+.2f}, {self.last_active_command[2]:+.2f}) "
            f"action_norm={self.last_action_norm:.3f} "
            f"inference=(submitted={inference.submitted} started={inference.started} "
            f"completed={inference.completed} "
            f"ok={inference.succeeded} failed={inference.failed} expired={inference.expired} "
            f"superseded={inference.superseded} invalidated={inference.invalidated} "
            f"discarded={inference.discarded})",
            throttle_duration_sec=2.0,
        )

    def _build_observation(
        self, command: Sequence[float], action_dim: int, obs_dim: Optional[int] = None
    ):
        assert self.last_lowstate is not None
        joint_pos = [float(state.q) for state in self.last_lowstate.motor_state[:29]]
        joint_vel = [float(state.dq) for state in self.last_lowstate.motor_state[:29]]
        imu = self.last_lowstate.imu_state
        quat = list(imu.quaternion)
        norm = math.sqrt(sum(q * q for q in quat))
        if norm > 1e-6:
            quat = [q / norm for q in quat]
        projected_gravity = _quat_rotate(_quat_conjugate(quat), (0.0, 0.0, -1.0))
        joint_pos_rel = [q - d for q, d in zip(joint_pos, self.default_positions)]

        values: list[float] = []
        values.extend(self.base_lin_vel)
        values.extend(float(v) for v in imu.gyroscope)
        values.extend(projected_gravity)
        values.extend(joint_pos_rel)
        values.extend(joint_vel)
        values.extend(self.last_actions[action_dim])
        if self._uses_height_command(obs_dim):
            values.append(float(self.get_parameter("policy_command_height").value))
        else:
            values.extend(command)
        return np.asarray([values], dtype=np.float32)

    def _uses_height_command(self, obs_dim: Optional[int] = None) -> bool:
        if self.policy_command_names:
            return self.policy_command_names == ["height"]
        dimension = self.policy_obs_dim if obs_dim is None else obs_dim
        return dimension in (80, 97)

    def _publish_action(self, action, action_dim: int, dt: float, ramp: float = 1.0) -> None:
        if isinstance(action, (tuple, list)):
            action = action[0]
        if torch is not None and isinstance(action, torch.Tensor):
            action_values = action.detach().to("cpu").reshape(-1).tolist()
        else:
            action_values = np.asarray(action).reshape(-1).tolist()
        if len(action_values) < action_dim:
            raise ValueError(
                f"Policy returned {len(action_values)} actions, expected {action_dim}"
            )
        action_values = action_values[:action_dim]
        if not all(math.isfinite(float(value)) for value in action_values):
            raise ValueError("Policy returned non-finite actions")
        clip = float(self.get_parameter("action_clip").value)
        action_values = [_clamp(float(value), clip) for value in action_values]
        ramp = max(0.0, min(1.0, ramp))
        if ramp < 1.0:
            action_values = [value * ramp for value in action_values]
        rate_limit = max(0.0, float(self.get_parameter("action_rate_limit").value))
        if rate_limit > 0.0:
            max_delta = rate_limit * dt
            previous = self.last_actions[action_dim]
            action_values = [
                previous[index] + _clamp(value - previous[index], max_delta)
                for index, value in enumerate(action_values)
            ]
        self.last_actions[action_dim] = action_values
        self.last_action_norm = math.sqrt(sum(value * value for value in action_values))

        targets = list(self.default_positions)
        if action_dim == 12:
            for action_index, joint_index in enumerate(self.lower_joint_indexes):
                targets[joint_index] += (
                    action_values[action_index] * self.scales[joint_index]
                )
        else:
            for joint_index, value in enumerate(action_values):
                targets[joint_index] += value * self.scales[joint_index]

        self._publish_targets(targets)

    def _publish_targets(self, targets: Sequence[float]) -> None:
        msg = Float64MultiArray()
        msg.data = list(targets)
        self.body_pub.publish(msg)
        self.body_command_count += 1

    def destroy_node(self):
        if hasattr(self, "policy_worker"):
            self.policy_worker.shutdown(timeout=1.0)
        return super().destroy_node()


class OnnxPolicy:
    def __init__(self, path: str) -> None:
        assert ort is not None
        self.session = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.metadata = self.session.get_modelmeta().custom_metadata_map
        self.obs_dim = self._shape_dim(self.session.get_inputs()[0].shape, 1)
        self.action_dim = self._shape_dim(self.session.get_outputs()[0].shape, 1)

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        return self.session.run([self.output_name], {self.input_name: obs})[0]

    @staticmethod
    def _shape_dim(shape: Sequence[object], index: int) -> Optional[int]:
        if len(shape) <= index:
            return None
        value = shape[index]
        return int(value) if isinstance(value, int) else None


class TorchPolicy:
    def __init__(self, policy, device: str) -> None:
        self.policy = policy
        self.device = device
        self.metadata = {}
        if hasattr(self.policy, "eval"):
            self.policy.eval()

    def __call__(self, obs: np.ndarray):
        assert torch is not None
        obs_tensor = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        with torch.no_grad():
            return self.policy(obs_tensor)


def main() -> None:
    rclpy.init()
    node = G1PolicyCmdVelController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
