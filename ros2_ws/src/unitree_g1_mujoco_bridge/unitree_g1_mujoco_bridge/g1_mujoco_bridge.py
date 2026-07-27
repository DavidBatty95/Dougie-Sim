import math
from typing import Iterable, List

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from unitree_hg.msg import HandCmd, LowCmd, LowState

from .g1_constants import (
    BRAINCO_HAND_JOINTS,
    BRAINCO_LEFT_MAX,
    BRAINCO_LEFT_MIN,
    BRAINCO_LEFT_TF_JOINTS,
    BRAINCO_RIGHT_MAX,
    BRAINCO_RIGHT_MIN,
    BRAINCO_RIGHT_TF_JOINTS,
    G1_29DOF_JOINTS,
    G1_MJLAB_BODY_KD,
    G1_MJLAB_BODY_KP,
    G1_RVIZ_JOINTS,
)


LATEST_RELIABLE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


def _clamp(value: float, lower: float, upper: float) -> float:
    if math.isnan(value):
        return 0.0
    return max(lower, min(upper, value))


def _bounded(values: Iterable[float], lower: List[float], upper: List[float]) -> List[float]:
    return [_clamp(float(value), lower[index], upper[index]) for index, value in enumerate(values)]


class G1MujocoBridge(Node):
    def __init__(self) -> None:
        super().__init__("g1_mujoco_bridge")

        self.declare_parameter("body_kp", 60.0)
        self.declare_parameter("body_kd", 2.5)
        self.declare_parameter("base_height_offset", 0.182)
        self.declare_parameter("use_mjlab_body_gains", True)
        self.declare_parameter("lock_waist", False)
        self.declare_parameter("waist_lock_kp", 120.0)
        self.declare_parameter("waist_lock_kd", 8.0)
        self.declare_parameter("balance_waist", False)
        self.declare_parameter("waist_balance_p", 0.35)
        self.declare_parameter("waist_balance_i", 0.0)
        self.declare_parameter("waist_balance_d", 0.06)
        self.declare_parameter("waist_balance_max", 0.25)
        self.declare_parameter("fall_damping_enable", True)
        self.declare_parameter("fall_damping_enter_angle", 0.95)
        self.declare_parameter("fall_damping_exit_angle", 0.45)
        self.declare_parameter("fall_damping_kd", 8.0)
        self.declare_parameter("fall_damping_waist_kd", 12.0)
        self.declare_parameter("fall_damping_reset_grace_time", 3.0)
        self.declare_parameter("startup_fall_damping_grace_time", 5.0)
        self.declare_parameter("hand_kp", 1.5)
        self.declare_parameter("hand_kd", 0.1)
        self.declare_parameter("body_command_topic", "/g1/body_position_commands")
        self.declare_parameter("left_hand_command_topic", "/g1/left_hand_position_commands")
        self.declare_parameter("right_hand_command_topic", "/g1/right_hand_position_commands")

        self.body_kp = float(self.get_parameter("body_kp").value)
        self.body_kd = float(self.get_parameter("body_kd").value)
        self.base_height_offset = float(self.get_parameter("base_height_offset").value)
        self.lock_waist = bool(self.get_parameter("lock_waist").value)
        self.waist_lock_kp = float(self.get_parameter("waist_lock_kp").value)
        self.waist_lock_kd = float(self.get_parameter("waist_lock_kd").value)
        self.balance_waist = bool(self.get_parameter("balance_waist").value)
        self.waist_balance_p = float(self.get_parameter("waist_balance_p").value)
        self.waist_balance_i = float(self.get_parameter("waist_balance_i").value)
        self.waist_balance_d = float(self.get_parameter("waist_balance_d").value)
        self.waist_balance_max = float(self.get_parameter("waist_balance_max").value)
        self.fall_damping_enable = bool(self.get_parameter("fall_damping_enable").value)
        self.fall_damping_enter_angle = float(self.get_parameter("fall_damping_enter_angle").value)
        self.fall_damping_exit_angle = float(self.get_parameter("fall_damping_exit_angle").value)
        self.fall_damping_kd = float(self.get_parameter("fall_damping_kd").value)
        self.fall_damping_waist_kd = float(self.get_parameter("fall_damping_waist_kd").value)
        self.fall_damping_reset_grace_time = float(self.get_parameter("fall_damping_reset_grace_time").value)
        self.startup_fall_damping_grace_time = float(self.get_parameter("startup_fall_damping_grace_time").value)
        self.fallen = False
        self.fall_damping_count = 0
        self.fall_damping_grace_until = None
        self.started_at = self.get_clock().now()
        self.waist_integral_roll = 0.0
        self.waist_integral_pitch = 0.0
        self.waist_joint_indexes = [
            G1_29DOF_JOINTS.index("waist_yaw_joint"),
            G1_29DOF_JOINTS.index("waist_roll_joint"),
            G1_29DOF_JOINTS.index("waist_pitch_joint"),
        ]
        self.body_kp_by_joint = [self.body_kp] * len(G1_29DOF_JOINTS)
        self.body_kd_by_joint = [self.body_kd] * len(G1_29DOF_JOINTS)
        if bool(self.get_parameter("use_mjlab_body_gains").value):
            self.body_kp_by_joint = list(G1_MJLAB_BODY_KP)
            self.body_kd_by_joint = list(G1_MJLAB_BODY_KD)
        if self.lock_waist:
            for index in self.waist_joint_indexes:
                self.body_kp_by_joint[index] = self.waist_lock_kp
                self.body_kd_by_joint[index] = self.waist_lock_kd
        self.hand_kp = float(self.get_parameter("hand_kp").value)
        self.hand_kd = float(self.get_parameter("hand_kd").value)
        self.body_position = [0.0] * len(G1_29DOF_JOINTS)
        self.body_velocity = [0.0] * len(G1_29DOF_JOINTS)
        self.body_effort = [0.0] * len(G1_29DOF_JOINTS)
        self.base_height = 0.793
        self.imu_rpy = [0.0, 0.0, 0.0]
        self.imu_gyro = [0.0, 0.0, 0.0]
        self.left_hand_position = [0.0] * len(BRAINCO_LEFT_TF_JOINTS)
        self.right_hand_position = [0.0] * len(BRAINCO_RIGHT_TF_JOINTS)
        self.lowstate_count = 0
        self.body_command_count = 0

        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", LATEST_RELIABLE_QOS
        )
        self.lowcmd_pub = self.create_publisher(LowCmd, "/lowcmd", LATEST_RELIABLE_QOS)
        self.lowcmd_rt_pub = self.create_publisher(LowCmd, "rt/lowcmd", LATEST_RELIABLE_QOS)
        self.left_hand_pub = self.create_publisher(
            HandCmd, "/dex3/left/cmd", LATEST_RELIABLE_QOS
        )
        self.right_hand_pub = self.create_publisher(
            HandCmd, "/dex3/right/cmd", LATEST_RELIABLE_QOS
        )

        self.create_subscription(
            LowState, "/lowstate", self._lowstate_callback, LATEST_RELIABLE_QOS
        )
        self.create_subscription(
            LowState, "rt/lowstate", self._lowstate_callback, LATEST_RELIABLE_QOS
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("body_command_topic").value),
            self._body_command_callback,
            LATEST_RELIABLE_QOS,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("left_hand_command_topic").value),
            lambda msg: self._hand_command_callback(msg, "left"),
            LATEST_RELIABLE_QOS,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("right_hand_command_topic").value),
            lambda msg: self._hand_command_callback(msg, "right"),
            LATEST_RELIABLE_QOS,
        )
        self.create_timer(1.0 / 60.0, self._publish_joint_states)
        self.create_timer(2.0, self._diagnostics)

        self.get_logger().info("G1 Mujoco ROS 2 bridge ready")

    def _lowstate_callback(self, msg: LowState) -> None:
        if len(msg.motor_state) < len(G1_29DOF_JOINTS):
            self.get_logger().warn(
                f"Ignoring short lowstate with {len(msg.motor_state)} motor states",
                throttle_duration_sec=2.0,
            )
            return
        if len(msg.imu_state.rpy) < 3 or len(msg.imu_state.gyroscope) < 3:
            self.get_logger().warn(
                "Ignoring lowstate with incomplete IMU data", throttle_duration_sec=2.0
            )
            return

        positions = [float(state.q) for state in msg.motor_state[: len(G1_29DOF_JOINTS)]]
        velocities = [float(state.dq) for state in msg.motor_state[: len(G1_29DOF_JOINTS)]]
        efforts = [float(state.tau_est) for state in msg.motor_state[: len(G1_29DOF_JOINTS)]]
        imu_rpy = [float(value) for value in msg.imu_state.rpy[:3]]
        imu_gyro = [float(value) for value in msg.imu_state.gyroscope[:3]]
        state_values = positions + velocities + efforts + imu_rpy + imu_gyro
        if not all(math.isfinite(value) for value in state_values):
            self.get_logger().warn(
                "Ignoring lowstate containing non-finite values", throttle_duration_sec=2.0
            )
            return

        self.lowstate_count += 1
        self.body_position = positions
        self.body_velocity = velocities
        self.body_effort = efforts
        self.base_height = self._estimate_base_height(self.body_position)
        self.imu_rpy = imu_rpy
        self.imu_gyro = imu_gyro
        self._update_fallen_state()

    def _publish_joint_states(self) -> None:
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = G1_RVIZ_JOINTS
        joint_state.position = [self.base_height] + self.body_position + self.left_hand_position + self.right_hand_position
        joint_state.velocity = [0.0] + self.body_velocity + [0.0] * (len(BRAINCO_LEFT_TF_JOINTS) + len(BRAINCO_RIGHT_TF_JOINTS))
        joint_state.effort = [0.0] + self.body_effort + [0.0] * (len(BRAINCO_LEFT_TF_JOINTS) + len(BRAINCO_RIGHT_TF_JOINTS))
        self.joint_state_pub.publish(joint_state)

    def _estimate_base_height(self, positions: List[float]) -> float:
        left_height = self._leg_height(
            positions[0],
            positions[3],
            positions[4],
        )
        right_height = self._leg_height(
            positions[6],
            positions[9],
            positions[10],
        )
        return _clamp(max(left_height, right_height) + self.base_height_offset, 0.35, 0.95)

    @staticmethod
    def _leg_height(hip_pitch: float, knee: float, ankle_pitch: float) -> float:
        hip_to_knee = 0.300
        knee_to_ankle = 0.300
        ankle_to_ground = 0.045
        thigh_angle = hip_pitch
        shank_angle = hip_pitch + knee
        foot_angle = shank_angle + ankle_pitch
        height = hip_to_knee * math.cos(thigh_angle)
        height += knee_to_ankle * math.cos(shank_angle)
        height += ankle_to_ground * math.cos(foot_angle)
        return height

    def _body_command_callback(self, msg: Float64MultiArray) -> None:
        self.body_command_count += 1
        if len(msg.data) != len(G1_29DOF_JOINTS):
            self.get_logger().warn(
                f"Expected {len(G1_29DOF_JOINTS)} body positions, got {len(msg.data)}",
                throttle_duration_sec=2.0,
            )
            return
        if not all(math.isfinite(float(value)) for value in msg.data):
            self.get_logger().warn(
                "Ignoring body command containing non-finite values",
                throttle_duration_sec=2.0,
            )
            return

        cmd = LowCmd()
        cmd.mode_pr = 0
        cmd.mode_machine = 0
        if self.fallen:
            self._fill_fall_damping_command(cmd)
            self.lowcmd_pub.publish(cmd)
            self.lowcmd_rt_pub.publish(cmd)
            self.fall_damping_count += 1
            return

        targets = list(msg.data)
        if self.balance_waist:
            self._apply_waist_balance(targets)
        elif self.lock_waist:
            for index in self.waist_joint_indexes:
                targets[index] = 0.0

        for index, target in enumerate(targets):
            motor = cmd.motor_cmd[index]
            motor.mode = 1
            motor.q = float(target)
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = self.body_kp_by_joint[index]
            motor.kd = self.body_kd_by_joint[index]
        self.lowcmd_pub.publish(cmd)
        self.lowcmd_rt_pub.publish(cmd)

    def _update_fallen_state(self) -> None:
        if not self.fall_damping_enable:
            self.fallen = False
            return
        now = self.get_clock().now()
        if (now - self.started_at).nanoseconds * 1e-9 < self.startup_fall_damping_grace_time:
            self.fallen = False
            return
        if self.fall_damping_grace_until is not None:
            if (self.fall_damping_grace_until - now).nanoseconds > 0:
                self.fallen = False
                return
            self.fall_damping_grace_until = None
        roll = abs(self._wrap_pi(self.imu_rpy[0]))
        pitch = abs(self._wrap_pi(self.imu_rpy[1]))
        tilt = max(roll, pitch)
        if self.fallen:
            if tilt < self.fall_damping_exit_angle:
                self.fallen = False
                self.fall_damping_grace_until = now + Duration(seconds=self.fall_damping_reset_grace_time)
                self.get_logger().info(
                    f"Robot reset/upright; allowing {self.fall_damping_reset_grace_time:.1f}s for stand controller"
                )
        elif tilt > self.fall_damping_enter_angle:
            self.fallen = True
            self.waist_integral_roll = 0.0
            self.waist_integral_pitch = 0.0
            self.get_logger().warn(
                f"Robot appears fallen (roll={self.imu_rpy[0]:+.2f}, pitch={self.imu_rpy[1]:+.2f}); damping joints",
                throttle_duration_sec=1.0,
            )

    def _fill_fall_damping_command(self, cmd: LowCmd) -> None:
        for index, current in enumerate(self.body_position):
            motor = cmd.motor_cmd[index]
            motor.mode = 1
            motor.q = float(current)
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = 0.0
            motor.kd = self.fall_damping_waist_kd if index in self.waist_joint_indexes else self.fall_damping_kd

    @staticmethod
    def _wrap_pi(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def _apply_waist_balance(self, targets: List[float]) -> None:
        _, roll_index, pitch_index = self.waist_joint_indexes
        dt = 1.0 / 100.0
        roll = self.imu_rpy[0]
        pitch = self.imu_rpy[1]
        roll_rate = self.imu_gyro[0]
        pitch_rate = self.imu_gyro[1]

        self.waist_integral_roll = _clamp(
            self.waist_integral_roll + roll * dt,
            -self.waist_balance_max,
            self.waist_balance_max,
        )
        self.waist_integral_pitch = _clamp(
            self.waist_integral_pitch + pitch * dt,
            -self.waist_balance_max,
            self.waist_balance_max,
        )

        targets[self.waist_joint_indexes[0]] = 0.0
        targets[roll_index] = _clamp(
            -self.waist_balance_p * roll
            - self.waist_balance_i * self.waist_integral_roll
            - self.waist_balance_d * roll_rate,
            -self.waist_balance_max,
            self.waist_balance_max,
        )
        targets[pitch_index] = _clamp(
            -self.waist_balance_p * pitch
            - self.waist_balance_i * self.waist_integral_pitch
            - self.waist_balance_d * pitch_rate,
            -self.waist_balance_max,
            self.waist_balance_max,
        )

    def _diagnostics(self) -> None:
        if self.lowstate_count == 0:
            self.get_logger().warn(
                "No /lowstate received from Mujoco yet; RViz TF will stay at the neutral pose.",
                throttle_duration_sec=5.0,
            )
        else:
            self.get_logger().info(
                f"Mujoco lowstate packets: {self.lowstate_count}, body commands: {self.body_command_count}, fallen={self.fallen}, damping_cmd={self.fall_damping_count}",
                throttle_duration_sec=10.0,
            )

    def _hand_command_callback(self, msg: Float64MultiArray, side: str) -> None:
        if len(msg.data) != len(BRAINCO_HAND_JOINTS):
            self.get_logger().warn(
                f"Expected {len(BRAINCO_HAND_JOINTS)} {side} hand positions, got {len(msg.data)}",
                throttle_duration_sec=2.0,
            )
            return
        if not all(math.isfinite(float(value)) for value in msg.data):
            self.get_logger().warn(
                f"Ignoring {side} hand command containing non-finite values",
                throttle_duration_sec=2.0,
            )
            return

        lower = BRAINCO_LEFT_MIN if side == "left" else BRAINCO_RIGHT_MIN
        upper = BRAINCO_LEFT_MAX if side == "left" else BRAINCO_RIGHT_MAX
        targets = _bounded(msg.data, lower, upper)
        if side == "left":
            self.left_hand_position = self._brainco_tf_positions(targets, "left")
        else:
            self.right_hand_position = self._brainco_tf_positions(targets, "right")

        cmd = HandCmd()
        cmd.motor_cmd = []
        for index, target in enumerate(targets):
            motor = cmd.motor_cmd.add() if hasattr(cmd.motor_cmd, "add") else None
            if motor is None:
                from unitree_hg.msg import MotorCmd

                motor = MotorCmd()
                cmd.motor_cmd.append(motor)
            motor.mode = (index & 0x0F) | (0x01 << 4)
            motor.q = target
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = self.hand_kp
            motor.kd = self.hand_kd

        if side == "left":
            self.left_hand_pub.publish(cmd)
        else:
            self.right_hand_pub.publish(cmd)

    @staticmethod
    def _brainco_tf_positions(targets: List[float], side: str) -> List[float]:
        thumb_meta = targets[0]
        thumb_prox = targets[1]
        index = targets[3]
        middle = targets[4]
        ring = targets[5]
        pinky = targets[6]

        if side == "right":
            thumb_meta = -thumb_meta

        return [
            thumb_meta,
            thumb_prox,
            thumb_prox,
            0.0,
            index,
            1.155 * index,
            1.0,
            middle,
            1.155 * middle,
            1.0,
            ring,
            1.155 * ring,
            1.0,
            pinky,
            1.155 * pinky,
            1.0,
        ]


def main() -> None:
    rclpy.init()
    node = G1MujocoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
