"""Unitree HG LowCmd CRC over the SDK's zero-padded little-endian ABI.

Layout and polynomial follow the pinned unitree_ros2 motor_crc_hg example.
This is not the CRC of ROS CDR serialization (which has different padding).
"""
import struct
from functools import lru_cache


def _table():
    result = []
    for byte in range(256):
        crc = byte << 24
        for _ in range(8):
            crc = ((crc << 1) ^ (0x04C11DB7 if crc & 0x80000000 else 0)) & 0xffffffff
        result.append(crc)
    return tuple(result)


_CRC_TABLE = _table()
_MOTOR = struct.Struct('<B3x5fI')


def lowcmd_payload(command):
    if len(command.motor_cmd)!=35 or len(command.reserve)!=4:
        raise ValueError('HG LowCmd requires 35 motors and four reserve words')
    payload = bytearray(struct.pack('<BB2x',command.mode_pr,command.mode_machine))
    for motor in command.motor_cmd:
        payload.extend(_MOTOR.pack(motor.mode,motor.q,motor.dq,motor.tau,motor.kp,motor.kd,motor.reserve))
    payload.extend(struct.pack('<4I',*command.reserve))
    return bytes(payload)


def lowcmd_crc(command):
    return _payload_crc(lowcmd_payload(command))


@lru_cache(maxsize=2)
def _payload_crc(payload):
    crc = 0xffffffff
    for (word,) in struct.iter_unpack('<I',payload):
        for shift in (24,16,8,0):
            crc = ((crc << 8) & 0xffffffff) ^ _CRC_TABLE[((crc >> 24) ^ (word >> shift)) & 255]
    return crc
