import math
from typing import Optional, Sequence


# Ordered to match G1_29DOF_JOINTS. Values come from the deployment G1 MJCF.
G1_JOINT_LIMITS = (
    (-2.5307, 2.8798),
    (-0.5236, 2.9671),
    (-2.7576, 2.7576),
    (-0.087267, 2.8798),
    (-0.87267, 0.5236),
    (-0.2618, 0.2618),
    (-2.5307, 2.8798),
    (-2.9671, 0.5236),
    (-2.7576, 2.7576),
    (-0.087267, 2.8798),
    (-0.87267, 0.5236),
    (-0.2618, 0.2618),
    (-2.618, 2.618),
    (-0.52, 0.52),
    (-0.52, 0.52),
    (-3.0892, 2.6704),
    (-1.5882, 2.2515),
    (-2.618, 2.618),
    (-1.0472, 2.0944),
    (-1.97222, 1.97222),
    (-1.61443, 1.61443),
    (-1.61443, 1.61443),
    (-3.0892, 2.6704),
    (-2.2515, 1.5882),
    (-2.618, 2.618),
    (-1.0472, 2.0944),
    (-1.97222, 1.97222),
    (-1.61443, 1.61443),
    (-1.61443, 1.61443),
)


def sample_age(now: float, arrival_time: Optional[float]) -> float:
    if arrival_time is None:
        return math.inf
    return max(0.0, now - arrival_time)


def cleared_motion_state() -> tuple[list[float], None, None]:
    return [0.0, 0.0, 0.0], None, None


def apply_command_deadband(
    command: Sequence[float], stopped: bool, threshold: float, hysteresis: float
) -> tuple[list[float], bool]:
    enter = max(0.0, threshold)
    exit_threshold = enter + max(0.0, hysteresis)
    magnitude = max((abs(float(value)) for value in command), default=0.0)
    stopped = magnitude <= (exit_threshold if stopped else enter)
    if stopped:
        return [0.0] * len(command), True
    return [float(value) for value in command], False


def filter_command(
    current: Sequence[float],
    target: Sequence[float],
    acceleration_limits: Sequence[float],
    dt: float,
    immediate_stop: bool,
) -> list[float]:
    if immediate_stop:
        return [0.0] * len(target)
    result = list(current)
    for index, limit in enumerate(acceleration_limits):
        max_delta = max(0.0, float(limit)) * max(0.0, dt)
        delta = float(target[index]) - result[index]
        result[index] += max(-max_delta, min(max_delta, delta))
    return result


def axis_isolated_command(
    command: Sequence[float], yaw_threshold: float
) -> tuple[list[float], bool]:
    values = [float(value) for value in command]
    turning = len(values) >= 3 and abs(values[2]) >= max(0.0, yaw_threshold)
    if turning:
        return [0.0, 0.0, values[2]], True
    if len(values) >= 3:
        values[2] = 0.0
    return values, False


def ensure_forward_turning_arc(
    command: Sequence[float],
    yaw_threshold: float,
    minimum_forward: float,
    minimum_yaw: float = 0.0,
) -> list[float]:
    values = [float(value) for value in command]
    if len(values) >= 3 and abs(values[2]) >= max(0.0, yaw_threshold):
        if values[0] < max(0.0, minimum_forward):
            values[0] = max(0.0, minimum_forward)
        if abs(values[2]) < max(0.0, minimum_yaw):
            values[2] = math.copysign(max(0.0, minimum_yaw), values[2])
    return values


def advance_gait_phase(
    phase: float,
    command: Sequence[float],
    dt: float,
    cycle_time: float,
    command_threshold: float = 0.05,
) -> float:
    """Advance a normalized gait clock, resetting it for stopped commands."""
    if max((abs(float(value)) for value in command), default=0.0) <= max(
        0.0, command_threshold
    ):
        return 0.0
    if cycle_time <= 0.0:
        raise ValueError("cycle_time must be positive")
    return (float(phase) + max(0.0, dt) / cycle_time) % 1.0


def clamp_joint_targets(
    targets: Sequence[float], soft_limit_factor: float = 0.9
) -> list[float]:
    if len(targets) != len(G1_JOINT_LIMITS):
        raise ValueError(
            f"Expected {len(G1_JOINT_LIMITS)} G1 joint targets, got {len(targets)}"
        )
    factor = max(0.0, min(1.0, soft_limit_factor))
    result = []
    for target, (lower, upper) in zip(targets, G1_JOINT_LIMITS):
        midpoint = 0.5 * (lower + upper)
        half_range = 0.5 * (upper - lower) * factor
        result.append(
            max(midpoint - half_range, min(midpoint + half_range, float(target)))
        )
    return result


def validate_metadata_joint_order(
    joint_names: Sequence[str],
    full_order: Sequence[str],
    lower_order: Sequence[str],
    action_dim: Optional[int] = None,
) -> None:
    names = list(joint_names)
    full = list(full_order)
    lower = list(lower_order)
    valid_orders = [full]
    if action_dim != len(full):
        valid_orders.append(lower)
    if names and names not in valid_orders:
        raise ValueError(
            "Policy joint_names metadata does not match the deployment joint order"
        )


def startup_active(
    elapsed: float,
    require_upright: bool,
    upright_duration: Optional[float],
    current_pose_hold: float,
    blend_time: float,
    stand_hold: float,
) -> bool:
    minimum_startup = sum(
        max(0.0, value) for value in (current_pose_hold, blend_time, stand_hold)
    )
    if elapsed < minimum_startup:
        return True
    if not require_upright:
        return False
    if upright_duration is None:
        return True
    return upright_duration < max(0.0, stand_hold)


def pending_target(
    last_safe_target: Optional[Sequence[float]], hold_target: Sequence[float]
) -> list[float]:
    return list(last_safe_target if last_safe_target is not None else hold_target)
