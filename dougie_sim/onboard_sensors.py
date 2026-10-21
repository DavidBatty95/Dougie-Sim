"""Simulator sensor front-end. Only detached body-frame measurements leave here.

Physics state is used inside LiDAR ray generation, just as a renderer uses scene
geometry. No world pose, framequat, framepos, framevel, source or shield metadata
is exposed to the estimator. Noise is seeded independently of detector counts.
"""
import numpy as np

from .odometry import Proprioception
from .policy import JOINTS
from .sensing import RangePacket


class OnboardSensors:
    def __init__(self, seed=0, lidar_dropout_after=None):
        self.rng = np.random.default_rng(seed)
        self.gyro_bias = self.rng.normal(0., .0005, 3)
        self.accel_bias = self.rng.normal(0., .015, 3)
        if lidar_dropout_after is not None and (not np.isfinite(lidar_dropout_after) or lidar_dropout_after < 0):
            raise ValueError('invalid LiDAR fault time')
        self.lidar_dropout_after = lidar_dropout_after

    def proprioception(self, sim):
        def read(name):
            return sim.data.sensor(name).data.copy()
        return Proprioception(float(sim.data.time),
            read('imu_gyro')+self.gyro_bias+self.rng.normal(0., .001, 3),
            read('imu_acc')+self.accel_bias+self.rng.normal(0., .02, 3),
            np.array([read(name.removesuffix('_joint')+'_pos')[0] for name in JOINTS])+self.rng.normal(0., .0002, 12),
            np.array([read(name.removesuffix('_joint')+'_vel')[0] for name in JOINTS])+self.rng.normal(0., .002, 12),
            np.maximum(0., np.array([read(leg+'_contact')[0] for leg in ('FR', 'FL', 'RR', 'RL')])+self.rng.normal(0., .3, 4)))

    def scan(self, sim):
        packet = sim.lidar_packet(body_frame=True)
        distance = packet.distances.copy()
        hit = packet.valid
        distance[hit] += self.rng.normal(0., .002, hit.sum())
        # Zero means missing/invalid. -1 is a valid max-range no-hit beam in the
        # occupancy mapper; using it for dropout would invent clear space.
        distance[self.rng.random(distance.shape) < .005] = 0.
        if self.lidar_dropout_after is not None and packet.stamp_s >= self.lidar_dropout_after:
            distance[:] = 0.
        return RangePacket(packet.stamp_s, packet.sequence, packet.origin, packet.directions,
                           distance, packet.max_range_m, np.zeros(distance.shape, dtype=bool))
