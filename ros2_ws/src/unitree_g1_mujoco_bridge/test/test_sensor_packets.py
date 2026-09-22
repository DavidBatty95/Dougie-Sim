import struct
from typing import Any

import pytest
from types import SimpleNamespace
from rclpy.time import Time

from unitree_g1_mujoco_bridge.g1_sim_sensors import (
    G1SimSensors,
    world_to_body_velocity,
)

from unitree_g1_mujoco_bridge.sensor_packets import (
    CAMERA_MAGIC_V1,
    CAMERA_MAGIC_V2,
    LIDAR_MAGIC_V1,
    LIDAR_MAGIC_V2,
    LIDAR_MAGIC_V3,
    PEDESTRIAN_MAGIC_V3,
    decode_camera_packet,
    decode_lidar_packet,
    decode_pedestrian_packet,
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


def test_onboard_payload_is_invariant_to_simulator_pose_and_velocity():
    truths = []
    sensor = SimpleNamespace(
        onboard_only=True, latest_mujoco_lidar_sim_time=None,
        last_base_pose="untouched", last_mujoco_pose="untouched",
        get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=2_000_000_000)),
        _quat_wxyz_to_matrix=G1SimSensors._quat_wxyz_to_matrix,
        _acquisition_stamp=G1SimSensors._acquisition_stamp,
        odom_pub=SimpleNamespace(publish=truths.append))
    payload = struct.pack("<ffff", 1., 2., 3., 4.)
    for offset in (0., 100.):
        packet = struct.pack("<IIdq3f4f3f4f3f3f", LIDAR_MAGIC_V3, 1, 1., 1_000_000_000,
                             offset, offset, 1., 1., 0., 0., 0.,
                             offset, offset, .8, 1., 0., 0., 0.,
                             offset, offset, offset, offset, offset, offset) + payload
        assert G1SimSensors._decode_mujoco_lidar_packet(sensor, packet) == payload
        assert sensor.last_base_pose == sensor.last_mujoco_pose == "untouched"
    assert len(truths) == 2 and truths[1].pose.pose.position.x == 100.
    assert sensor.odom_source == "external_onboard_estimator"


def test_onboard_sensor_loss_does_not_fabricate_geometry():
    sensor = SimpleNamespace(onboard_only=True,
        get_clock=lambda: SimpleNamespace(now=lambda: Time()),
        _latest_mujoco_lidar_packet=lambda: None)
    # Any access to fallback geometry or publishers would fail on this stub.
    G1SimSensors._publish_lidar(sensor)


def test_lidar_v3_keeps_exact_root_state_separate_from_sensor_pose() -> None:
    fmt = "<IIdq3f4f3f4f3f3f"
    assert struct.calcsize(fmt) == 104
    payload = struct.pack("<ffff", 1.0, 2.0, 3.0, 4.0)
    packet = struct.pack(fmt, LIDAR_MAGIC_V3, 1, 12.25, 1_721_234_567_890_123_456,
                         5.0, 6.0, 7.0, 0.7071, 0.0, 0.0, 0.7071,
                         1.0, 2.0, 0.8, 1.0, 0.0, 0.0, 0.0,
                         0.3, -0.2, 0.1, 0.01, 0.02, 0.4) + payload
    decoded = decode_lidar_packet(packet)
    assert decoded is not None
    assert decoded.origin == (5.0, 6.0, 7.0)
    assert decoded.root_origin == pytest.approx((1.0, 2.0, 0.8))
    assert decoded.root_quaternion_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert decoded.root_world_velocity == pytest.approx((0.3, -0.2, 0.1))
    assert decoded.root_body_angular_velocity == pytest.approx((0.01, 0.02, 0.4))
    assert decoded.payload == payload
    assert decode_lidar_packet(packet[:-1]) is None


def test_lidar_v3_unavailable_root_state_preserves_legacy_fallback():
    packet = struct.pack("<IIdq3f4f3f4f3f3f", LIDAR_MAGIC_V3, 0, 1.0, 10,
                         0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, *([0.0] * 13))
    decoded = decode_lidar_packet(packet)
    assert decoded is not None and decoded.root_origin is None


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


def test_pedestrian_v3_separates_population_from_active_people() -> None:
    packet = struct.pack(
        "<IIIdqiddddd",
        PEDESTRIAN_MAGIC_V3,
        1,
        4,
        12.5,
        1_721_000_000_000_000_001,
        3,
        1.0,
        2.0,
        0.1,
        0.2,
        0.3,
    )
    decoded = decode_pedestrian_packet(packet)
    assert decoded is not None
    assert decoded.population_count == 4
    assert len(decoded.people) == 1
    assert decoded.people[0][0] == 3


def test_pedestrian_v3_rejects_population_below_active_count() -> None:
    packet = struct.pack("<IIIdq", PEDESTRIAN_MAGIC_V3, 2, 1, 0.0, 1)
    packet += struct.pack("<iddddd", 1, 0.0, 0.0, 0.0, 0.0, 0.3) * 2
    assert decode_pedestrian_packet(packet) is None


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
