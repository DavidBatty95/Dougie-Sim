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
    def __init__(self, map_capacity=20000, robust=True, slam_backend='distributed', loop_search_radius=1.25,
                 keyframe_retention='recent'):
        if slam_backend not in ('distributed', 'pose-graph') or (not robust and slam_backend != 'distributed'):
            raise ValueError('pose-graph localization requires the robust navigation profile')
        if not math.isfinite(loop_search_radius) or not .5 <= loop_search_radius <= 3.:
            raise ValueError('loop search radius must be between .5 and 3 metres')
        self.loop_search_radius = float(loop_search_radius)
        self.loop_search = dict(attempts=0, candidates=0, last_reason='not_attempted')
        if keyframe_retention not in ('recent', 'coverage'):
            raise ValueError('unknown keyframe retention policy')
        self.keyframe_retention, self.keyframe_evictions = keyframe_retention, 0
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
        self.robust = robust
        self.keyframes = []
        self.corrections = []
        self.registration_status = 'initializing'
        self.registration_rank = 0
        self.inlier_fraction = 0.
        self.rejection_streak = 0
        self.slipping_legs = 0
        self.last_good_scan = None
        self.last_loop_attempt = -100.
        self.last_loop_stamp = 0.
        self.loop_candidate = None
        self.loop_closure_enabled = True
        self.anchor_revalidations = 0
        self.slam_backend = slam_backend
        from .pose_graph import PoseGraph
        self.pose_graph = PoseGraph() if slam_backend == 'pose-graph' else None
        from .pose_uncertainty import PoseUncertaintyScale
        self.uncertainty_scale = PoseUncertaintyScale()

    def update(self, observation):
        t = observation.stamp_s
        if self.stamp is not None and t < self.stamp:
            raise ValueError('out-of-order IMU/encoder observation')
        dt = 0. if self.stamp is None else t-self.stamp
        if dt > .1:
            raise ValueError('IMU observation gap exceeds 100 ms')
        self.stamp = t
        self.drift_variance += dt*1e-5
        if self.robust:
            self.drift_variance += np.linalg.norm(self.velocity)*dt*.0008
        feet, foot_velocity = leg_kinematics(observation.joints, observation.joint_velocity)
        self.feet = feet
        contact = observation.foot_force > 12.
        self.slipping_legs = 0
        if self.robust and np.count_nonzero(contact) >= 3:
            estimates = -(foot_velocity+np.cross(observation.gyro-self.gyro_bias, feet))@self.rotation.T
            median = np.median(estimates[contact], axis=0)
            residual = np.linalg.norm(estimates-median, axis=1)
            bad = contact & (residual > max(.08, 3.*float(np.median(residual[contact]))))
            self.slipping_legs = int(bad.sum())
            contact &= ~bad
            self.drift_variance += self.slipping_legs*dt*.0002
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
        if self.robust:
            from .scan_geometry import registration_cloud
            points, normals = registration_cloud(local)
        else:
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
            self.rejection_streak += 1
            self.registration_status = 'insufficient_range_structure'
            return
        _, indices = np.unique(np.floor(points/.12).astype(int), axis=0, return_index=True)
        indices = indices[::max(1, math.ceil(len(indices)/1500))]
        points, normals = points[indices], normals[indices]
        if self.robust:
            self._observe_submaps(points, normals, local.stamp_s)
            return
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

    @property
    def map_revision(self):
        return len(self.corrections)

    @property
    def uncalibrated_position_std(self):
        return np.sqrt(np.diag(self.covariance)[:3]+self.drift_variance)

    @property
    def position_std(self):
        return self.uncertainty_scale.apply(self.uncalibrated_position_std)

    def _observe_submaps(self, points, normals, stamp):
        from .registration import Keyframe, register
        if self.keyframes:
            recent = self.keyframes[-4:]
            worlds = [frame.world() for frame in recent]
            target, target_normals = np.concatenate([x[0] for x in worlds]), np.concatenate([x[1] for x in worlds])
            fit = register(points, normals, target, target_normals, self.position, self.rotation)
            self.registration_status = fit.reason
            self.registration_rank, self.inlier_fraction = fit.observable_rank, fit.inlier_fraction
            self.scan_rmse = fit.rmse if math.isfinite(fit.rmse) else None
            if not fit.accepted:
                self.scan_rejections += 1
                self.rejection_streak += 1
                self.drift_variance += .0001
                return  # Rejected scans must not contaminate future registration.
            self.position, self.rotation = fit.position, fit.rotation
            self.scan_updates += 1
            self.rejection_streak = 0
            self.last_good_scan = stamp
            self.drift_variance += min(.0002, fit.translation_m**2*.05)
            self.anchors = {leg: self.position+self.rotation@self.feet[leg] for leg in self.anchors}
            self._try_loop(points, normals, stamp)
        else:
            self.registration_status, self.last_good_scan = 'initial_submap', stamp
        last = self.keyframes[-1] if self.keyframes else None
        if (last is None or np.linalg.norm(self.position-last.position) > .6
                or Rotation.from_matrix(self.rotation@last.rotation.T).magnitude() > .35 or stamp-last.stamp > 12.):
            self.keyframes.append(Keyframe(stamp, self.position.copy(), self.rotation.copy(), points.copy(), normals.copy(), self.drift_variance))
            if self.pose_graph is not None:
                self.pose_graph.add_pose(stamp, self.position, self.rotation)
            if len(self.keyframes) > 64:
                self._prune_keyframes()
        # Compatibility/diagnostics only; registration uses immutable keyframes.
        world, n = self.keyframes[-1].world()
        self.map = {tuple(np.floor(p/.12).astype(int)): np.r_[p, normal] for p, normal in zip(world, n)}

    def _prune_keyframes(self):
        if len(self.keyframes) <= 64:
            return
        victim = 8  # Historical policy: initial eight plus recent submaps.
        if self.keyframe_retention == 'coverage':
            positions = np.asarray([frame.position for frame in self.keyframes])
            up = np.asarray([frame.rotation[:, 2] for frame in self.keyframes])
            distance = (np.linalg.norm(positions[:, None]-positions[None, :], axis=2)
                        +.5*np.linalg.norm(up[:, None]-up[None, :], axis=2))
            np.fill_diagonal(distance, np.inf)
            nearest = distance.min(axis=1)
            pending = None if self.loop_candidate is None else self.loop_candidate[0]
            eligible = [i for i in range(8, len(self.keyframes)-4) if self.keyframes[i].stamp != pending]
            victim = min(eligible, key=lambda i: (nearest[i], -self.keyframes[i].variance, self.keyframes[i].stamp))
        del self.keyframes[victim]
        self.keyframe_evictions += 1

    def _try_loop(self, points, normals, stamp):
        from .registration import register
        from .map_corrections import MapCorrection
        if not self.loop_closure_enabled or len(self.corrections) >= 16 or stamp-self.last_loop_stamp < 30. or stamp-self.last_loop_attempt < 2.:
            return
        self.last_loop_attempt = stamp
        self.loop_search['attempts'] += 1
        self.loop_search['stamp_s'] = float(stamp)
        candidates = [frame for frame in self.keyframes[:-4]
                      if stamp-frame.stamp > 20. and np.linalg.norm(frame.position-self.position) < self.loop_search_radius]
        self.loop_search['candidates'] = len(candidates)
        if not candidates:
            self.loop_search.update(last_reason='no_eligible_anchor', anchor_stamp_s=None,
                anchor_distance_m=None, rank=None, inlier_fraction=None, residual_m=None)
            self.loop_candidate = None
            return
        anchor = min(candidates, key=lambda frame: frame.stamp)
        target, target_normals = anchor.world()
        fit = register(points, normals, target, target_normals, self.position, self.rotation, .4, .15)
        self.loop_search.update(anchor_stamp_s=anchor.stamp,
            anchor_distance_m=float(np.linalg.norm(anchor.position-self.position)),
            rank=fit.observable_rank, inlier_fraction=fit.inlier_fraction,
            residual_m=fit.rmse if math.isfinite(fit.rmse) else None)
        if not fit.accepted or fit.observable_rank < 6 or fit.inlier_fraction < .55 or fit.rmse > .025:
            self.loop_search['last_reason'] = 'registration_quality_rejected'
            self.loop_candidate = None
            return
        R = fit.rotation@self.rotation.T
        t = fit.position-R@self.position
        rv = Rotation.from_matrix(R).as_rotvec()
        small = fit.translation_m < .025 and np.linalg.norm(rv) < .015
        previous = self.loop_candidate
        self.loop_candidate = anchor.stamp, rv.copy(), t.copy(), stamp
        if (previous is None or previous[0] != anchor.stamp or stamp-previous[3] > 5.
                or np.linalg.norm(previous[1]-rv) > .03 or np.linalg.norm(previous[2]-t) > .08):
            self.loop_search['last_reason'] = 'awaiting_confirmation'
            return
        if small and self.pose_graph is None:
            # A confirmed near-zero loop match is still useful evidence against
            # accumulated drift. Do not force a pointless pose/map correction,
            # but do not ignore the old anchor merely because pose already agrees.
            self.drift_variance = min(self.drift_variance, max(.03**2, anchor.variance+fit.rmse**2))
            self.last_loop_stamp, self.loop_candidate = stamp, None
            self.anchor_revalidations += 1
            self.loop_search['last_reason'] = 'anchor_revalidated'
            return
        if self.pose_graph is not None:
            event = self.pose_graph.close_loop(anchor.stamp, stamp, self.position, self.rotation,
                fit.position, fit.rotation, self.map_revision+1)
            if event is None:
                self.loop_search['last_reason'] = 'graph_rejected'
                self.loop_candidate = None
                return
            self.position = self.pose_graph.poses[-1].position.copy()
            self.rotation = self.pose_graph.poses[-1].rotation.copy()
            R, _ = event.transform(stamp)
            if small:
                self.anchor_revalidations += 1
        else:
            event = MapCorrection(self.map_revision+1, max(anchor.stamp, self.last_loop_stamp), stamp, tuple(rv), tuple(t))
            self.position, self.rotation = fit.position, fit.rotation
        self.velocity = R@self.velocity
        transform = np.zeros((6, 6))
        transform[:3, :3] = transform[3:, 3:] = R
        self.covariance = transform@self.covariance@transform.T
        for frame in self.keyframes:
            frame.correct(event)
        self.anchors = {leg: self.position+self.rotation@self.feet[leg] for leg in self.anchors}
        self.corrections.append(event)
        self.loop_search['last_reason'] = 'map_corrected'
        self.last_loop_stamp, self.loop_candidate = stamp, None
        self.drift_variance = max(.03**2, anchor.variance+fit.rmse**2)
        if self.pose_graph is not None:
            marginal = self.pose_graph.last_result.get('position_marginal_covariance')
            if marginal is not None:
                self.drift_variance = max(self.drift_variance, float(np.max(np.diag(marginal))))

    def world_packet(self, local):
        if self.stamp is not None and abs(local.stamp_s-self.stamp) > .015:
            raise ValueError('LiDAR must be transformed at its capture timestamp')
        directions = local.directions@self.rotation.T
        origin = self.position+self.rotation@local.origin
        return RangePacket(local.stamp_s, local.sequence, origin, directions, local.distances,
                           local.max_range_m, support_mask(origin, directions, local.distances, local.max_range_m), self.map_revision)

    def metrics(self):
        return dict(source='imu_leg_lidar_fusion', initialized=self.initialized,
            position=self.position.tolist(), velocity=self.velocity.tolist(), yaw=self.yaw,
            position_std_m=self.position_std.tolist(),
            uncalibrated_position_std_m=self.uncalibrated_position_std.tolist(),
            uncertainty_calibration=self.uncertainty_scale.metrics(),
            conditional_position_std_m=np.sqrt(np.diag(self.covariance)[:3]).tolist(),
            gyro_bias=self.gyro_bias.tolist(), scan_updates=self.scan_updates,
            scan_rejections=self.scan_rejections, scan_rmse_m=self.scan_rmse, local_map_cells=len(self.map),
            robust=self.robust, map_revision=self.map_revision, keyframes=len(self.keyframes),
            anchor_revalidations=self.anchor_revalidations,
            loop_search_radius_m=self.loop_search_radius, loop_search=dict(self.loop_search),
            keyframe_retention=self.keyframe_retention, keyframe_evictions=self.keyframe_evictions,
            slam_backend=self.slam_backend, pose_graph=self.pose_graph.metrics() if self.pose_graph is not None else None,
            registration_status=self.registration_status, observable_rank=self.registration_rank,
            inlier_fraction=self.inlier_fraction, rejection_streak=self.rejection_streak, slipping_legs=self.slipping_legs,
            last_good_scan_s=self.last_good_scan,
            uncertainty_scope='conditional filter plus path/innovation drift allowance; not a calibrated probability guarantee')
