import dataclasses
import struct
from typing import Optional


LIDAR_MAGIC_V1 = 0x524C4A4D
LIDAR_MAGIC_V2 = 0x324C4A4D
CAMERA_MAGIC_V1 = 0x52434A4D
CAMERA_MAGIC_V2 = 0x32434A4D


@dataclasses.dataclass(frozen=True)
class SensorPacket:
    simulation_time: float
    acquisition_time_ns: Optional[int]
    origin: tuple
    quaternion_wxyz: tuple
    payload: bytes
    count: int = 0
    width: int = 0
    height: int = 0


def decode_lidar_packet(packet: bytes) -> Optional[SensorPacket]:
    if len(packet) < 4:
        return None
    magic = struct.unpack_from("<I", packet)[0]
    if magic == LIDAR_MAGIC_V2:
        header_format = "<IIdq3f4f"
        acquisition_index = 3
        pose_index = 4
    elif magic == LIDAR_MAGIC_V1:
        header_format = "<IId3f4f"
        acquisition_index = None
        pose_index = 3
    else:
        return None
    header_size = struct.calcsize(header_format)
    if len(packet) < header_size:
        return None
    values = struct.unpack_from(header_format, packet)
    count = int(values[1])
    payload_size = count * 16
    if len(packet) < header_size + payload_size:
        return None
    acquisition_ns = values[acquisition_index] if acquisition_index is not None else None
    return SensorPacket(
        simulation_time=values[2],
        acquisition_time_ns=acquisition_ns,
        origin=tuple(values[pose_index : pose_index + 3]),
        quaternion_wxyz=tuple(values[pose_index + 3 : pose_index + 7]),
        payload=packet[header_size : header_size + payload_size],
        count=count,
    )


def decode_camera_packet(packet: bytes) -> Optional[SensorPacket]:
    if len(packet) < 4:
        return None
    magic = struct.unpack_from("<I", packet)[0]
    if magic == CAMERA_MAGIC_V2:
        header_format = "<IIIdq3f4f"
        acquisition_index = 4
        pose_index = 5
    elif magic == CAMERA_MAGIC_V1:
        pose_format = "<IIId3f4f"
        base_format = "<IIId"
        if len(packet) < struct.calcsize(base_format):
            return None
        _, width, height, simulation_time = struct.unpack_from(base_format, packet)
        payload_size = int(width) * int(height) * 5
        header_format = pose_format if len(packet) >= struct.calcsize(pose_format) + payload_size else base_format
        acquisition_index = None
        pose_index = 4 if header_format == pose_format else None
    else:
        return None
    header_size = struct.calcsize(header_format)
    if len(packet) < header_size:
        return None
    values = struct.unpack_from(header_format, packet)
    width = int(values[1])
    height = int(values[2])
    payload_size = width * height * 5
    if len(packet) < header_size + payload_size:
        return None
    acquisition_ns = values[acquisition_index] if acquisition_index is not None else None
    origin = tuple(values[pose_index : pose_index + 3]) if pose_index is not None else ()
    quaternion = tuple(values[pose_index + 3 : pose_index + 7]) if pose_index is not None else ()
    return SensorPacket(
        simulation_time=values[3],
        acquisition_time_ns=acquisition_ns,
        origin=origin,
        quaternion_wxyz=quaternion,
        payload=packet[header_size : header_size + payload_size],
        width=width,
        height=height,
    )
