import math
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
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


class G1PolicyCmdVelController(Node):
    def __init__(self) -> None:
        super().__init__("g1_policy_cmd_vel_controller")

        self.declare_parameter("policy_path", "")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
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
        self.declare_parameter("stand_still_on_timeout", True)
        self.declare_parameter("startup_blend_time", 1.5)
        self.declare_parameter("startup_hold_time", 1.0)
        self.declare_parameter("detect_robot_reset", True)
        self.declare_parameter("reset_joint_delta_threshold", 0.75)
        self.declare_parameter("reset_joint_total_delta_threshold", 4.0)
        self.declare_parameter("reset_fallen_tilt_threshold", 0.95)
        self.declare_parameter("reset_upright_tilt_threshold", 0.45)
        self.declare_parameter("reset_detection_cooldown", 1.0)
        self.declare_parameter("device", "cpu")

        self.device = str(self.get_parameter("device").value)
        self.policy = self._load_policy(str(self.get_parameter("policy_path").value))
        self.policy_obs_dim: Optional[int] = None
        self.policy_action_dim: Optional[int] = None
        policy_obs_dim = int(self.get_parameter("policy_obs_dim").value)
        policy_action_dim = int(self.get_parameter("policy_action_dim").value)
        if policy_obs_dim in (82, 99):
            self.policy_obs_dim = policy_obs_dim
        if policy_action_dim in (12, 29):
            self.policy_action_dim = policy_action_dim
        self.last_lowstate: Optional[LowState] = None
        self.previous_joint_positions: Optional[List[float]] = None
        self.previous_tilt: Optional[float] = None
        self.first_lowstate_time = None
        self.initial_positions: Optional[List[float]] = None
        self.last_reset_time = None
        self.reset_count = 0
        self.last_cmd_time = self.get_clock().now()
        self.command = [0.0, 0.0, 0.0]
        self.filtered_command = [0.0, 0.0, 0.0]
        self.last_control_time = self.get_clock().now()
        self.cmd_vel_count = 0
        self.lowstate_count = 0
        self.body_command_count = 0
        self.last_action_norm = 0.0
        self.last_active_command = [0.0, 0.0, 0.0]
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

        self.body_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("body_command_topic").value), 10
        )
        self.create_subscription(LowState, "/lowstate", self._lowstate_callback, 10)
        self.create_subscription(LowState, "rt/lowstate", self._lowstate_callback, 10)
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value), self._cmd_vel_callback, 10
        )

        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(period, self._control_tick)
        self.create_timer(2.0, self._diagnostics)
        self.get_logger().info("G1 policy /cmd_vel controller ready")

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

    def _apply_policy_metadata(self) -> None:
        metadata = getattr(self.policy, "metadata", {})
        if not metadata:
            return

        default_joint_pos = _parse_float_csv(metadata.get("default_joint_pos", ""))
        action_scale = _parse_float_csv(metadata.get("action_scale", ""))
        joint_names = _parse_str_csv(metadata.get("joint_names", ""))
        if joint_names and joint_names != self.body_joint_names:
            self.get_logger().warn(
                "Policy joint metadata order differs from bridge order; using bridge order",
                throttle_duration_sec=5.0,
            )
        if len(default_joint_pos) == len(self.body_joint_names):
            self.default_positions = default_joint_pos
        if len(action_scale) == len(self.lower_joint_indexes):
            for action_index, joint_index in enumerate(self.lower_joint_indexes):
                self.scales[joint_index] = action_scale[action_index]
        elif len(action_scale) == len(self.body_joint_names):
            self.scales = action_scale

        obs_dim = getattr(self.policy, "obs_dim", None)
        action_dim = getattr(self.policy, "action_dim", None)
        if self.policy_obs_dim is None and obs_dim in (82, 99):
            self.policy_obs_dim = int(obs_dim)
        if self.policy_action_dim is None and action_dim in (12, 29):
            self.policy_action_dim = int(action_dim)

    def _lowstate_callback(self, msg: LowState) -> None:
        self.lowstate_count += 1
        joint_positions = [float(state.q) for state in msg.motor_state[:29]]
        tilt = self._lowstate_tilt(msg)
        if self._detect_robot_reset(joint_positions, tilt):
            self._reset_controller_state(joint_positions, "reset/upright transition")
        if self.first_lowstate_time is None:
            self.first_lowstate_time = self.get_clock().now()
            self.initial_positions = list(joint_positions)
        self.previous_joint_positions = list(joint_positions)
        self.previous_tilt = tilt
        self.last_lowstate = msg

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
        self.first_lowstate_time = now
        self.initial_positions = list(joint_positions)
        self.filtered_command = [0.0, 0.0, 0.0]
        self.last_active_command = [0.0, 0.0, 0.0]
        self.last_control_time = now
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
        self.cmd_vel_count += 1
        self.last_cmd_time = self.get_clock().now()
        self.command = [
            _clamp(float(msg.linear.x), float(self.get_parameter("max_lin_vel_x").value)),
            _clamp(float(msg.linear.y), float(self.get_parameter("max_lin_vel_y").value)),
            _clamp(float(msg.angular.z), float(self.get_parameter("max_ang_vel_z").value)),
        ]

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
            self.get_logger().warn(
                "Waiting for /lowstate before running policy", throttle_duration_sec=2.0
            )
            return

        now = self.get_clock().now()
        dt = max(1e-3, min(0.1, (now - self.last_control_time).nanoseconds * 1e-9))
        self.last_control_time = now
        command = self._filtered_active_command(self._active_command(), dt)
        self.last_active_command = command
        if self._startup_active():
            self._publish_startup_pose()
            return

        candidates = [self.policy_obs_dim] if self.policy_obs_dim is not None else [82, 99]
        last_error: Optional[Exception] = None
        for obs_dim in candidates:
            if obs_dim is None:
                continue
            action_dim = self.policy_action_dim or (12 if obs_dim == 82 else 29)
            obs = self._build_observation(command, action_dim)
            try:
                action = self.policy(obs)
                self.policy_obs_dim = obs_dim
                self._publish_action(action, action_dim, dt)
                return
            except Exception as exc:  # noqa: BLE001 - retry alternate policy shape.
                last_error = exc

        self.get_logger().error(
            f"Policy inference failed: {last_error}", throttle_duration_sec=2.0
        )

    def _startup_active(self) -> bool:
        if self.first_lowstate_time is None:
            return True
        elapsed = (self.get_clock().now() - self.first_lowstate_time).nanoseconds * 1e-9
        startup_time = float(self.get_parameter("startup_blend_time").value)
        startup_time += float(self.get_parameter("startup_hold_time").value)
        return elapsed < startup_time

    def _publish_startup_pose(self) -> None:
        if self.first_lowstate_time is None or self.initial_positions is None:
            return
        elapsed = (self.get_clock().now() - self.first_lowstate_time).nanoseconds * 1e-9
        blend_time = max(float(self.get_parameter("startup_blend_time").value), 1e-6)
        alpha = min(1.0, max(0.0, elapsed / blend_time))
        alpha = alpha * alpha * (3.0 - 2.0 * alpha)
        targets = [
            (1.0 - alpha) * start + alpha * target
            for start, target in zip(self.initial_positions, self.default_positions)
        ]
        msg = Float64MultiArray()
        msg.data = targets
        self.body_pub.publish(msg)
        self.body_command_count += 1

    def _diagnostics(self) -> None:
        cmd_age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        self.get_logger().info(
            "policy status: "
            f"lowstate={self.lowstate_count} cmd_vel={self.cmd_vel_count} "
            f"body_cmd={self.body_command_count} resets={self.reset_count} cmd_age={cmd_age:.2f}s "
            f"cmd=({self.last_active_command[0]:+.2f}, "
            f"{self.last_active_command[1]:+.2f}, {self.last_active_command[2]:+.2f}) "
            f"action_norm={self.last_action_norm:.3f}",
            throttle_duration_sec=2.0,
        )

    def _build_observation(self, command: Sequence[float], action_dim: int):
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
        values.extend([0.0, 0.0, 0.0])
        values.extend(float(v) for v in imu.gyroscope)
        values.extend(projected_gravity)
        values.extend(joint_pos_rel)
        values.extend(joint_vel)
        values.extend(self.last_actions[action_dim])
        values.extend(command)
        return np.asarray([values], dtype=np.float32)

    def _publish_action(self, action, action_dim: int, dt: float) -> None:
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
        clip = float(self.get_parameter("action_clip").value)
        action_values = [_clamp(float(value), clip) for value in action_values]
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

        msg = Float64MultiArray()
        msg.data = targets
        self.body_pub.publish(msg)
        self.body_command_count += 1


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
