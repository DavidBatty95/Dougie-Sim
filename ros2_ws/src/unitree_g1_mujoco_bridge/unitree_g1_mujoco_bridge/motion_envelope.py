"""Shared executable command bounds and response-aware planar rollouts.

Response defaults are conservative engineering assumptions, not a fitted G1
response certificate. Keep them explicit so calibration can replace them.
"""

import math
from dataclasses import dataclass

from .policy_safety import apply_command_deadband, axis_isolated_command, ensure_forward_turning_arc


@dataclass(frozen=True)
class MotionEnvelope:
    max_x: float = 0.35
    max_y: float = 0.12
    max_yaw: float = 0.35
    accel_x: float = 0.22
    accel_y: float = 0.16
    accel_yaw: float = 0.35
    deadband: float = 0.05
    hysteresis: float = 0.01
    arc_forward: float = 0.0
    arc_yaw: float = 0.0
    yaw_threshold: float = 0.05
    axis_exclusive: bool = False
    allow_reverse: bool = True
    forward_gain: float = 1.0
    lateral_gain: float = 1.0
    yaw_gain: float = 1.0

    def normalize(self, command, stopped: bool = False) -> tuple[float, float, float]:
        if not all(math.isfinite(value) for value in command):
            return (0.0, 0.0, 0.0)
        limits = (self.max_x, self.max_y, self.max_yaw)
        values = [max(-limit, min(limit, float(value))) for value, limit in zip(command, limits)]
        if not self.allow_reverse:
            values[0] = max(0.0, values[0])
        values, stopped = apply_command_deadband(values, stopped, self.deadband, self.hysteresis)
        if stopped:
            return (0.0, 0.0, 0.0)
        if self.axis_exclusive:
            values, _ = axis_isolated_command(values, self.yaw_threshold)
        else:
            values = ensure_forward_turning_arc(values, self.yaw_threshold, self.arc_forward, self.arc_yaw)
        # Arc floors must never override actuator/policy bounds.
        return tuple(max(-limit, min(limit, value)) for value, limit in zip(values, limits))

    def rollout(self, pose, command, velocity, horizon: float, dt: float, delay: float = 0.15, command_duration: float = math.inf, next_command=None, filtered_command=None):
        x, y, yaw = pose
        velocity = list(velocity)
        gains = (self.forward_gain, self.lateral_gain, self.yaw_gain)
        requested = self.normalize(command)
        next_requested = self.normalize(next_command) if next_command is not None else (0.0, 0.0, 0.0)
        target = tuple(value * gain for value, gain in zip(requested, gains))
        following = tuple(value * gain for value, gain in zip(next_requested, gains))
        filtered = None if filtered_command is None else list(filtered_command)
        acceleration = (self.accel_x, self.accel_y, self.accel_yaw)
        steps = max(1, math.ceil(horizon / max(0.02, dt)))
        step_dt = horizon / steps
        response_fraction = (1.0 - math.exp(-step_dt / 0.25), 1.0 - math.exp(-step_dt / 0.25), 1.0 - math.exp(-step_dt / 0.40))
        trajectory = [(x, y, yaw, 0.0)]
        for step in range(steps):
            # Integrate midpoint velocity; zero target still has a stopping path.
            previous = velocity[:]
            active_dt = max(0.0, (step + 1) * step_dt - max(step * step_dt, delay))
            step_target = target if step * step_dt < command_duration else following
            if filtered is None:
                for axis, limit in enumerate(acceleration):
                    change = max(0.0, limit) * active_dt
                    velocity[axis] += max(-change, min(change, step_target[axis] - velocity[axis]))
            else:
                step_command = requested if step * step_dt < command_duration else next_requested
                immediate_stop = active_dt > 0.0 and not any(step_command)
                for axis, limit in enumerate(acceleration):
                    change = max(0.0, limit) * active_dt
                    if immediate_stop:
                        filtered[axis] = 0.0  # Policy stop bypasses command slew.
                    else:
                        filtered[axis] += max(-change, min(change, step_command[axis] - filtered[axis]))
                    velocity[axis] += (filtered[axis] * gains[axis] - velocity[axis]) * response_fraction[axis]
            vx, vy, wz = [(old + new) * 0.5 for old, new in zip(previous, velocity)]
            heading = yaw + wz * step_dt * 0.5
            x += (math.cos(heading) * vx - math.sin(heading) * vy) * step_dt
            y += (math.sin(heading) * vx + math.cos(heading) * vy) * step_dt
            yaw += wz * step_dt
            trajectory.append((x, y, yaw, (step + 1) * step_dt))
        return trajectory


_PARAMETERS = {
    "max_lin_vel_x": ("max_x", 0.35),
    "max_lin_vel_y": ("max_y", 0.12),
    "max_ang_vel_z": ("max_yaw", 0.35),
    "cmd_accel_x": ("accel_x", 0.22),
    "cmd_accel_y": ("accel_y", 0.16),
    "cmd_accel_yaw": ("accel_yaw", 0.35),
    "cmd_vel_deadband": ("deadband", 0.05),
    "cmd_vel_deadband_hysteresis": ("hysteresis", 0.01),
    "turning_arc_min_forward_speed": ("arc_forward", 0.0),
    "turning_arc_min_yaw_speed": ("arc_yaw", 0.0),
    "axis_exclusive_yaw_threshold": ("yaw_threshold", 0.05),
    "axis_exclusive_cmd_vel": ("axis_exclusive", False),
    "allow_reverse_cmd_vel": ("allow_reverse", True),
    "response_forward_gain": ("forward_gain", 1.0),
    "response_lateral_gain": ("lateral_gain", 1.0),
    "response_yaw_gain": ("yaw_gain", 1.0),
}


def load_motion_envelope(node) -> MotionEnvelope:
    values = {}
    for name, (field, default) in _PARAMETERS.items():
        if not node.has_parameter(name):
            node.declare_parameter(name, default)
        values[field] = node.get_parameter(name).value
    return MotionEnvelope(**values)
