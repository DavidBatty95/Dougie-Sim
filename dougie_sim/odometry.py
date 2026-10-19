"""IMU/stance-leg error-state filter with observed-cloud registration.

No simulator imports, world poses, map files, source positions or object IDs.
Coordinates are an arbitrary gravity-aligned frame established at first stance.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .sensing import RangePacket, support_mask


@dataclass(frozen=True)
class Proprioception:
    stamp_s: float
    gyro: np.ndarray
    acceleration: np.ndarray
    joints: np.ndarray
    joint_velocity: np.ndarray
    foot_force: np.ndarray

    def __post_init__(self):
        for name, shape in (('gyro', (3,)), ('acceleration', (3,)), ('joints', (12,)),
                            ('joint_velocity', (12,)), ('foot_force', (4,))):
            value = np.array(getattr(self, name), dtype=float, copy=True)
            if value.shape != shape or not np.isfinite(value).all() or not math.isfinite(self.stamp_s):
                raise ValueError('invalid proprioceptive observation')
            value.setflags(write=False)
            object.__setattr__(self, name, value)


def leg_kinematics(joints, velocities):
    """Go2 URDF link dimensions; FR, FL, RR, RL. Returns body-frame feet/Jdq."""
    q, dq = np.asarray(joints).reshape(4, 3), np.asarray(velocities).reshape(4, 3)
    h, t, c = q.T
    side = np.array([-1., 1., -1., 1.])
    x = -.213*(np.sin(t)+np.sin(t+c))
    z = -.213*(np.cos(t)+np.cos(t+c))
    y = side*.0955
    feet = np.column_stack((x+np.array([1., 1., -1., -1.])*.1934,
        np.cos(h)*y-np.sin(h)*z+side*.0465, np.sin(h)*y+np.cos(h)*z))
    dx = -.213*(np.cos(t)*dq[:, 1]+np.cos(t+c)*(dq[:, 1]+dq[:, 2]))
    dz = .213*(np.sin(t)*dq[:, 1]+np.sin(t+c)*(dq[:, 1]+dq[:, 2]))
    velocity = np.column_stack((dx, (-np.sin(h)*y-np.cos(h)*z)*dq[:, 0]-np.sin(h)*dz,
        (np.cos(h)*y-np.sin(h)*z)*dq[:, 0]+np.cos(h)*dz))
    return feet, velocity


class FusedOdometry:
    def __init__(self, map_capacity=20000):
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.rotation = np.eye(3)
        self.gyro_bias = np.zeros(3)
        self.covariance = np.eye(6)*.01
        self.drift_variance = .03**2
        self.stamp = None
        self.initialized = False
        self.anchors = {}
        self.feet = np.zeros((4, 3))
        self.map = {}
        self.map_capacity = map_capacity
        self.scan_updates = self.scan_rejections = 0
        self.scan_rmse = None
        self.last_scan_stamp = None
        self.last_contact_stamp = None

    def update(self, observation):
        t = observation.stamp_s
        if self.stamp is not None and t < self.stamp:
            raise ValueError('out-of-order IMU/encoder observation')
        dt = 0. if self.stamp is None else t-self.stamp
        if dt > .1:
            raise ValueError('IMU observation gap exceeds 100 ms')
        self.stamp = t
        self.drift_variance += dt*1e-5
        feet, foot_velocity = leg_kinematics(observation.joints, observation.joint_velocity)
        self.feet = feet
        contact = observation.foot_force > 12.
        omega = observation.gyro-self.gyro_bias
        acceleration = observation.acceleration
        # Gravity tilt correction is gated against strongly dynamic acceleration.
        norm = np.linalg.norm(acceleration)
        if 8.8 < norm < 10.8:
            measured_up = acceleration/norm
            expected_up = self.rotation.T@np.array([0., 0., 1.])
            omega = omega+.8*np.cross(measured_up, expected_up)
        self.rotation = self.rotation@Rotation.from_rotvec(omega*dt).as_matrix()
        if t < 1. and np.count_nonzero(contact) >= 3 and np.linalg.norm(observation.joint_velocity) < .15 and np.linalg.norm(observation.gyro) < .03:
            self.gyro_bias += min(1., dt*.2)*(observation.gyro-self.gyro_bias)
        if not self.initialized:
            if np.count_nonzero(contact) < 2:
                return
            self.position[2] = .022-float(np.median((feet@self.rotation.T)[contact, 2]))
            self.initialized = True
        world_acceleration = self.rotation@acceleration+np.array([0., 0., -9.81])
        self.position += self.velocity*dt+.5*world_acceleration*dt*dt
        self.velocity += world_acceleration*dt
        F = np.eye(6)
        F[:3, 3:] = np.eye(3)*dt
        self.covariance = F@self.covariance@F.T+np.diag([1e-5]*3+[.08]*3)*dt
        if np.any(contact):
            self.last_contact_stamp = t
            estimates = -(foot_velocity+np.cross(observation.gyro-self.gyro_bias, feet))@self.rotation.T
            measured = np.median(estimates[contact], axis=0)
            disagreement = float(np.mean(np.sum((estimates[contact]-measured)**2, axis=1)))
            self._correct(measured-self.velocity, np.c_[np.zeros((3, 3)), np.eye(3)], .0025+disagreement)
        for leg in list(self.anchors):
            if not contact[leg]:
                del self.anchors[leg]
        for leg in np.flatnonzero(contact):
            if leg not in self.anchors:
                self.anchors[leg] = self.position+self.rotation@feet[leg]
        if self.anchors:
            estimates = np.array([anchor-self.rotation@feet[leg] for leg, anchor in self.anchors.items()])
            measured = np.median(estimates, axis=0)
            disagreement = float(np.mean(np.sum((estimates-measured)**2, axis=1)))
            self._correct(measured-self.position, np.c_[np.eye(3), np.zeros((3, 3))], .0004+disagreement)

    def _correct(self, residual, H, variance):
        R = np.eye(len(residual))*variance
        gain = np.linalg.solve(H@self.covariance@H.T+R, H@self.covariance).T
        correction = gain@residual
        self.position += correction[:3]
        self.velocity += correction[3:]
        I = np.eye(6)-gain@H
        self.covariance = I@self.covariance@I.T+gain@R@gain.T

    @property
    def yaw(self):
        return math.atan2(self.rotation[1, 0], self.rotation[0, 0])

    @property
    def sensor_position(self):
        return self.position+self.rotation@np.array([0., 0., .12])

    @property
    def support_position(self):
        feet = self.feet@self.rotation.T+self.position
        return np.r_[self.position[:2], np.mean(np.sort(feet[:, 2])[:2])-.022]

    def observe_scan(self, local):
        """Point-to-plane local-map update; unobservable directions retain odometry."""
        if self.last_scan_stamp is not None and local.stamp_s <= self.last_scan_stamp:
            raise ValueError('out-of-order LiDAR observation')
        self.last_scan_stamp = local.stamp_s
        if not self.initialized:
            return
        points = local.origin+local.directions*local.distances[..., None]
        a = np.roll(points, 1, axis=1)-np.roll(points, -1, axis=1)
        b = np.roll(points, 1, axis=0)-np.roll(points, -1, axis=0)
        normals = np.cross(a, b)
        lengths = np.linalg.norm(normals, axis=2)
        valid = local.valid & np.roll(local.valid, 1, axis=0) & np.roll(local.valid, -1, axis=0)
        valid &= np.roll(local.valid, 1, axis=1) & np.roll(local.valid, -1, axis=1)
        valid &= (lengths > 1e-5) & (np.linalg.norm(a, axis=2) < .6) & (np.linalg.norm(b, axis=2) < .6)
        valid[[0, -1]] = False
        points, normals = points[valid], normals[valid]/lengths[valid, None]
        if len(points) < 40:
            self.scan_rejections += 1
            return
        _, indices = np.unique(np.floor(points/.12).astype(int), axis=0, return_index=True)
        indices = indices[::max(1, math.ceil(len(indices)/1500))]
        points, normals = points[indices], normals[indices]
        if len(self.map) >= 40:
            stored = np.asarray(list(self.map.values()))
            target, target_normals = stored[:, :3], stored[:, 3:]
            tree = cKDTree(target)
            accepted = False
            for _ in range(5):
                rotated = points@self.rotation.T
                world = rotated+self.position
                distance, nearest = tree.query(world)
                keep = distance < .25
                if keep.sum() < 40:
                    break
                n = target_normals[nearest[keep]]
                residual = np.sum(n*(target[nearest[keep]]-world[keep]), axis=1)
                robust = np.minimum(1., .04/np.maximum(np.abs(residual), 1e-8))
                A = np.column_stack((n, np.cross(rotated[keep], n)))
                H = A.T@(robust[:, None]*A)
                eigenvalues, vectors = np.linalg.eigh(H)
                inverse = np.where(eigenvalues > max(2., eigenvalues[-1]*.015), 1./np.maximum(eigenvalues, 1e-8), 0.)
                delta = vectors@(inverse*(vectors.T@(A.T@(robust*residual))))
                if np.linalg.norm(delta[:3]) > .10 or np.linalg.norm(delta[3:]) > .08 or np.sqrt(np.mean(residual**2)) > .12:
                    break
                self.position += delta[:3]
                self.rotation = Rotation.from_rotvec(delta[3:]).as_matrix()@self.rotation
                self.scan_rmse = float(np.sqrt(np.mean(residual**2)))
                accepted = True
                if np.linalg.norm(delta) < 1e-4:
                    break
            if accepted:
                self.scan_updates += 1
                # Re-anchor stance feet to the corrected estimate, avoiding a
                # false opposing leg innovation on the next high-rate update.
                self.anchors = {leg: self.position+self.rotation@self.feet[leg] for leg in self.anchors}
            else:
                self.scan_rejections += 1
        world, normals = points@self.rotation.T+self.position, normals@self.rotation.T
        for point, normal in zip(world, normals):
            key = tuple(np.floor(point/.12).astype(int))
            self.map[key] = np.r_[point, normal]
        if len(self.map) > self.map_capacity:
            # Local registration map only; mission support/radiation maps are not evicted.
            keys = sorted(self.map, key=lambda k: np.linalg.norm(self.map[k][:3]-self.position))[:self.map_capacity]
            self.map = {key: self.map[key] for key in keys}

    def world_packet(self, local):
        if self.stamp is not None and abs(local.stamp_s-self.stamp) > .015:
            raise ValueError('LiDAR must be transformed at its capture timestamp')
        directions = local.directions@self.rotation.T
        origin = self.position+self.rotation@local.origin
        return RangePacket(local.stamp_s, local.sequence, origin, directions, local.distances,
                           local.max_range_m, support_mask(origin, directions, local.distances, local.max_range_m))

    def metrics(self):
        return dict(source='imu_leg_lidar_fusion', initialized=self.initialized,
            position=self.position.tolist(), velocity=self.velocity.tolist(), yaw=self.yaw,
            position_std_m=np.sqrt(np.diag(self.covariance)[:3]+self.drift_variance).tolist(),
            conditional_position_std_m=np.sqrt(np.diag(self.covariance)[:3]).tolist(),
            gyro_bias=self.gyro_bias.tolist(), scan_updates=self.scan_updates,
            scan_rejections=self.scan_rejections, scan_rmse_m=self.scan_rmse, local_map_cells=len(self.map))
