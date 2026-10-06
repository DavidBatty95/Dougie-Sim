import struct
import math
from types import SimpleNamespace

from std_msgs.msg import Header

from unitree_g1_mujoco_bridge.sensor_packets import IMU_MAGIC_V1,decode_imu_packet
from unitree_g1_mujoco_bridge.g1_sim_sensors import G1SimSensors
from unitree_g1_mujoco_bridge.sensor_geometry import matrix_to_quat_xyzw


def test_native_imu_packet_validity_and_no_fabricated_orientation():
    stamp = 10000000000
    wire = struct.pack('<IIdq12f',IMU_MAGIC_V1,3,1.,stamp,*range(12))
    decoded = decode_imu_packet(wire)
    assert decoded.camera == (0.,1.,2.,3.,4.,5.)
    assert decoded.lidar == (6.,7.,8.,9.,10.,11.)
    assert decode_imu_packet(wire[:-1]) is None
    assert decode_imu_packet(wire+b'bad') is None
    assert decode_imu_packet(struct.pack('<IIdq12f',IMU_MAGIC_V1,3,1.,stamp,*([float('nan')]*12))) is None
    packets = [wire]
    def receive(_):
        if packets:
            return packets.pop(0)
        raise BlockingIOError
    streams = {name:[] for name in ('imu','accel','gyro','livox_imu')}
    node = SimpleNamespace(mujoco_imu_socket_rx=SimpleNamespace(recv=receive),last_imu_published_ns=0,
        native_imu_count=0,imu_frame='camera_imu_optical_frame',lidar_frame='livox_frame',
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=stamp+10000000)),
        _header=lambda frame,stamp:Header(frame_id=frame,stamp=stamp),
        **{name+'_pub':SimpleNamespace(publish=messages.append) for name,messages in streams.items()})
    G1SimSensors._publish_imu(node)
    assert streams['imu'][0].orientation_covariance[0] == -1.
    assert streams['imu'][0].linear_acceleration.z == 5.
    assert streams['livox_imu'][0].angular_velocity.z == 8.
    assert streams['accel'][0].angular_velocity_covariance[0] == -1.
    assert streams['gyro'][0].linear_acceleration_covariance[0] == -1.
    packets.append(wire)
    G1SimSensors._publish_imu(node)
    assert len(streams['imu'])==1


def test_camera_info_has_valid_rectification_matrix():
    info = G1SimSensors._camera_info(SimpleNamespace(width=192,height=108),Header())
    assert list(info.r)==[1.,0.,0.,0.,1.,0.,0.,0.,1.]
    assert info.k[0]>0. and info.k[4]>0.
    assert info.width==192 and info.height==108


def test_measured_sensor_rotations_round_trip_including_inverted_lidar():
    import numpy as np
    from unitree_g1_mujoco_bridge.onboard_lidar_odometry import axis_rotation
    for axis in range(3):
        for angle in (0.,.4,math.pi):
            rotation = axis_rotation(axis,angle)
            x,y,z,w = matrix_to_quat_xyzw(rotation)
            actual = G1SimSensors._quat_wxyz_to_matrix((w,x,y,z))
            np.testing.assert_allclose(actual,rotation,atol=1e-12)
