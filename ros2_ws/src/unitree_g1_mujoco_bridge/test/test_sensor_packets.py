import struct
from typing import Any

import pytest

from unitree_g1_mujoco_bridge.g1_sim_sensors import (
    G1SimSensors,
    world_to_body_velocity,
)

from unitree_g1_mujoco_bridge.sensor_packets import (
    CAMERA_MAGIC_V1,
    CAMERA_MAGIC_V2,
    LIDAR_MAGIC_V1,
    LIDAR_MAGIC_V2,
    decode_camera_packet,
    decode_lidar_packet,
)


def test_lidar_v2_preserves_both_timestamps() -> None:
    payload = struct.pack("<ffff", 1.0, 2.0, 3.0, 4.0)
    packet = struct.pack(
        "<IIdq3f4f", LIDAR_MAGIC_V2, 1, 12.25, 1_721_234_567_890_123_456,
        5.0, 6.0, 7.0, 1.0, 0.0, 0.0, 0.0,
    ) + payload
    decoded = decode_lidar_packet(packet)
    assert decoded is not None
    assert decoded.simulation_time == 12.25
    assert decoded.acquisition_time_ns == 1_721_234_567_890_123_456
    assert decoded.payload == payload


def test_lidar_legacy_keeps_receipt_time_fallback_signal() -> None:
    packet = struct.pack(
        "<IId3f4f", LIDAR_MAGIC_V1, 0, 3.5, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    )
    decoded = decode_lidar_packet(packet)
    assert decoded is not None
    assert decoded.simulation_time == 3.5
    assert decoded.acquisition_time_ns is None


def test_camera_v2_preserves_timestamp_and_payload() -> None:
    payload = bytes(range(10))
    packet = struct.pack(
        "<IIIdq3f4f", CAMERA_MAGIC_V2, 2, 1, 9.75, 1_721_000_000_000_000_001,
        1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0,
    ) + payload
    decoded = decode_camera_packet(packet)
    assert decoded is not None
    assert decoded.acquisition_time_ns == 1_721_000_000_000_000_001
    assert decoded.payload == payload
    assert decoded.origin == (1.0, 2.0, 3.0)


def test_camera_legacy_header_without_pose_remains_supported() -> None:
    payload = bytes(range(5))
    packet = struct.pack("<IIId", CAMERA_MAGIC_V1, 1, 1, 2.0) + payload
    decoded = decode_camera_packet(packet)
    assert decoded is not None
    assert decoded.acquisition_time_ns is None
    assert decoded.origin == ()
    assert decoded.payload == payload


def test_world_velocity_is_rotated_into_policy_body_frame() -> None:
    rotation = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    body_velocity = world_to_body_velocity(rotation, (0.0, 1.0, 0.25))
    assert body_velocity == pytest.approx((1.0, 0.0, 0.25))


def test_exact_odometry_state_preserves_vertical_body_velocity() -> None:
    class Clock:
        def now(self) -> object:
            return object()

    class SensorState:
        current_odom_twist = (0.0, 0.0, 0.0, 0.4)
        last_odom_pose = None
        last_odom_time = None

        def get_clock(self) -> Clock:
            return Clock()

        def _stable_base_yaw(self, _rotation: object) -> float:
            return 0.0

    sensor: Any = SensorState()
    rotation = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    G1SimSensors._update_odometry_state(
        sensor,
        (0.0, 0.0, 0.84),
        rotation,
        (0.1, -0.2, 0.3),
        None,
    )
    assert sensor.current_odom_twist == pytest.approx((0.1, -0.2, 0.3, 0.4))
