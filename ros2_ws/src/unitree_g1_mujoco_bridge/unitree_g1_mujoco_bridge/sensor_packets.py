import dataclasses
import math
import struct
from typing import Optional


LIDAR_MAGIC_V1 = 0x524C4A4D
LIDAR_MAGIC_V2 = 0x324C4A4D
LIDAR_MAGIC_V3 = 0x334C4A4D
CAMERA_MAGIC_V1 = 0x52434A4D
CAMERA_MAGIC_V2 = 0x32434A4D
PEDESTRIAN_MAGIC_V1 = 0x31444550
PEDESTRIAN_MAGIC_V2 = 0x32444550
PEDESTRIAN_MAGIC_V3 = 0x33444550
IMU_MAGIC_V1 = 0x31494A4D


@dataclasses.dataclass(frozen=True)
class ImuPacket:
    simulation_time: float
    acquisition_time_ns: int
    camera: Optional[tuple]
    lidar: Optional[tuple]


def decode_imu_packet(packet: bytes) -> Optional[ImuPacket]:
    if len(packet)!=struct.calcsize('<IIdq12f'):
        return None
    magic,valid,simulation_time,stamp,*vectors = struct.unpack('<IIdq12f',packet)
    if magic!=IMU_MAGIC_V1 or valid not in (1,2,3) or stamp<=0:
        return None
    if not math.isfinite(simulation_time) or simulation_time<0. or not all(map(math.isfinite,vectors)):
        return None
    return ImuPacket(simulation_time,stamp,tuple(vectors[:6]) if valid&1 else None,
                     tuple(vectors[6:]) if valid&2 else None)


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
    root_origin: Optional[tuple] = None
    root_quaternion_wxyz: Optional[tuple] = None
    root_world_velocity: Optional[tuple] = None
    root_body_angular_velocity: Optional[tuple] = None


@dataclasses.dataclass(frozen=True)
class PedestrianPacket:
    simulation_time: float
    acquisition_time_ns: Optional[int]
    population_count: Optional[int]
    people: tuple[tuple[int, float, float, float, float, float], ...]


def decode_lidar_packet(packet: bytes) -> Optional[SensorPacket]:
    if len(packet) < 4:
        return None
    magic = struct.unpack_from("<I", packet)[0]
    if magic == LIDAR_MAGIC_V3:
        header_format = "<IIdq3f4f3f4f3f3f"
        acquisition_index = 3
        pose_index = 4
    elif magic == LIDAR_MAGIC_V2:
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
    root = {}
    if magic == LIDAR_MAGIC_V3:
        state = values[11:24]
        if not all(math.isfinite(value) for value in state):
            return None
        # A zero root quaternion denotes unavailable root state on other models.
        if sum(value * value for value in values[14:18]) > 0.5:
            root = dict(root_origin=tuple(values[11:14]), root_quaternion_wxyz=tuple(values[14:18]),
                        root_world_velocity=tuple(values[18:21]), root_body_angular_velocity=tuple(values[21:24]))
    return SensorPacket(
        simulation_time=values[2],
        acquisition_time_ns=acquisition_ns,
        origin=tuple(values[pose_index : pose_index + 3]),
        quaternion_wxyz=tuple(values[pose_index + 3 : pose_index + 7]),
        payload=packet[header_size : header_size + payload_size],
        count=count,
        **root,
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


def decode_pedestrian_packet(packet: bytes) -> Optional[PedestrianPacket]:
    if len(packet) < 8:
        return None
    magic, count = struct.unpack_from("<II", packet)
    if magic == PEDESTRIAN_MAGIC_V3:
        header_format = "<IIIdq"
        population_index = 2
        simulation_index = 3
        acquisition_index = 4
    elif magic == PEDESTRIAN_MAGIC_V2:
        header_format = "<IIdq"
        population_index = None
        simulation_index = 2
        acquisition_index = 3
    elif magic == PEDESTRIAN_MAGIC_V1:
        header_format = "<IId"
        population_index = None
        simulation_index = 2
        acquisition_index = None
    else:
        return None
    header_size = struct.calcsize(header_format)
    entry_format = "<iddddd"
    entry_size = struct.calcsize(entry_format)
    if count > 64 or len(packet) < header_size + count * entry_size:
        return None
    values = struct.unpack_from(header_format, packet)
    people = tuple(
        struct.unpack_from(entry_format, packet, header_size + index * entry_size)
        for index in range(count)
    )
    population = int(values[population_index]) if population_index is not None else None
    if population is not None and population < count:
        return None
    acquisition = int(values[acquisition_index]) if acquisition_index is not None else None
    return PedestrianPacket(
        simulation_time=float(values[simulation_index]),
        acquisition_time_ns=acquisition,
        population_count=population,
        people=people,
    )
