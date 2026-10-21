"""Bounded six-DoF pose graph from onboard relative-pose observations.

The first pose fixes gauge. Sequential scan/leg-odometry and confirmed loop
factors are optimized jointly with robust sparse least squares. No world map,
physical pose or radiation truth enters this module. Factor scales are modelling
assumptions, not calibrated posterior covariance.
"""
from dataclasses import dataclass
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.sparse import eye as sparse_eye
from scipy.sparse.linalg import splu
from scipy.spatial.transform import Rotation

from .map_corrections import MapCorrection


@dataclass(frozen=True)
class Pose:
    stamp: float
    position: np.ndarray
    rotation: np.ndarray


@dataclass(frozen=True)
class Factor:
    first: int
    second: int
    translation: np.ndarray
    rotation: np.ndarray
    sigma: np.ndarray
    kind: str


def relative(first, second):
    return first.rotation.T@(second.position-first.position), first.rotation.T@second.rotation


def se3_error(rotations, translations):
    """Log of batched SE(3) errors: translation tangent followed by rotation."""
    omega = Rotation.from_matrix(rotations).as_rotvec()
    angle = np.linalg.norm(omega, axis=1)
    coefficient = np.full(len(angle), 1./12.)
    large = angle > 1e-4
    coefficient[large] = 1./angle[large]**2-1./(2.*angle[large]*np.tan(angle[large]/2.))
    cross = np.cross(omega, translations)
    tangent = translations-.5*cross+coefficient[:, None]*np.cross(omega, cross)
    return np.column_stack((tangent, omega))


class PoseGraph:
    def __init__(self, capacity=512):
        if capacity < 2:
            raise ValueError('pose graph capacity must be at least two')
        self.capacity = capacity
        self.poses, self.factors = [], []
        self.saturated = False
        self.accepted_loops = self.rejected_loops = 0
        self.last_result = {}

    def add_pose(self, stamp, position, rotation):
        position, rotation = np.asarray(position, dtype=float), np.asarray(rotation, dtype=float)
        if (not np.isfinite(stamp) or position.shape != (3,) or rotation.shape != (3, 3)
                or not np.isfinite(position).all() or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-6) or np.linalg.det(rotation) < .999):
            raise ValueError('invalid graph pose')
        if self.poses and stamp <= self.poses[-1].stamp:
            if stamp == self.poses[-1].stamp:
                if not np.allclose(position, self.poses[-1].position) or not np.allclose(rotation, self.poses[-1].rotation):
                    raise ValueError('conflicting graph pose at same timestamp')
                return len(self.poses)-1
            raise ValueError('graph poses must be chronological')
        if len(self.poses) >= self.capacity:
            self.saturated = True
            return None
        pose = Pose(float(stamp), position.copy(), rotation.copy())
        if self.poses:
            translation, delta = relative(self.poses[-1], pose)
            sigma = np.r_[np.full(3, .015+.02*np.linalg.norm(translation)),
                          np.full(3, .01+.01*Rotation.from_matrix(delta).magnitude())]
            self.factors.append(Factor(len(self.poses)-1, len(self.poses), translation, delta, sigma, 'odometry'))
        self.poses.append(pose)
        return len(self.poses)-1

    @staticmethod
    def _residual(poses, factors):
        first = np.array([f.first for f in factors])
        second = np.array([f.second for f in factors])
        R = np.asarray([p.rotation for p in poses])
        t = np.asarray([p.position for p in poses])
        measured_R = np.asarray([f.rotation for f in factors])
        predicted_R = np.swapaxes(R[first], 1, 2)@R[second]
        predicted_t = np.einsum('nij,nj->ni', np.swapaxes(R[first], 1, 2), t[second]-t[first])
        error_R = np.swapaxes(measured_R, 1, 2)@predicted_R
        error_t = np.einsum('nij,nj->ni', np.swapaxes(measured_R, 1, 2),
                            predicted_t-np.asarray([f.translation for f in factors]))
        return se3_error(error_R, error_t)/np.asarray([f.sigma for f in factors])

    def close_loop(self, anchor_stamp, stamp, position, rotation, matched_position, matched_rotation, revision):
        """Transactional optimization; reject inconsistent loops without moving history."""
        started = time.perf_counter()
        candidate = PoseGraph(self.capacity)
        candidate.poses, candidate.factors = list(self.poses), list(self.factors)
        anchor = next((i for i, pose in enumerate(candidate.poses) if pose.stamp == anchor_stamp), None)
        current = candidate.add_pose(stamp, position, rotation)
        if anchor is None or current is None or anchor == current:
            self.saturated |= candidate.saturated
            self.rejected_loops += 1
            self.last_result = dict(accepted=False, reason='missing_anchor_or_capacity')
            return None
        matched = Pose(stamp, np.asarray(matched_position), np.asarray(matched_rotation))
        translation, delta = relative(candidate.poses[anchor], matched)
        loop = Factor(anchor, current, translation, delta, np.array([.03]*3+[.02]*3), 'loop')
        factors = candidate.factors+[loop]
        initial = self._residual(candidate.poses, factors)
        if not np.isfinite(initial).all() or np.linalg.norm(initial[-1]) > 12.:
            self.rejected_loops += 1
            self.last_result = dict(accepted=False, reason='loop_innovation_limit')
            return None
        original = list(candidate.poses)
        n = len(original)
        x0 = np.column_stack(([p.position for p in original[1:]],
                              Rotation.from_matrix([p.rotation for p in original[1:]]).as_rotvec())).reshape(-1)

        def unpack(x):
            values = x.reshape(-1, 6)
            rotations = Rotation.from_rotvec(values[:, 3:]).as_matrix()
            return [original[0]]+[Pose(p.stamp, value[:3].copy(), R) for p, value, R in zip(original[1:], values, rotations)]

        sparsity = lil_matrix((len(factors)*6, (n-1)*6), dtype=int)
        for row, factor in enumerate(factors):
            for index in (factor.first, factor.second):
                if index:
                    sparsity[row*6:(row+1)*6, (index-1)*6:index*6] = 1
        result = least_squares(lambda x: self._residual(unpack(x), factors).reshape(-1), x0,
            jac_sparsity=sparsity.tocsr(), method='trf', loss='soft_l1', f_scale=1.,
            max_nfev=60, ftol=1e-7, xtol=1e-7, gtol=1e-7)
        optimized = unpack(result.x)
        residual = self._residual(optimized, factors)
        cost = lambda r: float(np.sum(2.*(np.sqrt(1.+r*r)-1.)))
        moves = np.array([np.linalg.norm(a.position-b.position) for a, b in zip(original, optimized)])
        angles = np.array([Rotation.from_matrix(b.rotation@a.rotation.T).magnitude() for a, b in zip(original, optimized)])
        accepted = bool(result.success and np.isfinite(residual).all() and cost(residual) <= cost(initial)+1e-8
                        and np.linalg.norm(residual[-1]) <= 4. and np.max(np.linalg.norm(residual, axis=1)) <= 6.
                        and moves.max() <= .5 and angles.max() <= .2)
        self.last_result = dict(accepted=accepted, reason='optimized' if accepted else 'graph_consistency_rejected',
            initial_cost=cost(initial), final_cost=cost(residual), evaluations=result.nfev,
            max_translation_m=float(moves.max()), max_rotation_rad=float(angles.max()),
            loop_residual_norm=float(np.linalg.norm(residual[-1])), wall_ms=(time.perf_counter()-started)*1000.)
        if not accepted:
            self.rejected_loops += 1
            return None
        # Conditional Gauss-Newton marginal in the optimized parameter chart.
        # It is not a calibrated SLAM posterior: association and factor-scale
        # uncertainty still require held-out episode checks.
        H = (result.jac.T@result.jac).tocsc()+sparse_eye(len(x0), format='csc')*1e-9
        columns = np.zeros((len(x0), 3))
        columns[-6:-3, :] = np.eye(3)
        marginal = splu(H).solve(columns)[-6:-3, :]
        marginal = (marginal+marginal.T)*.5
        if not np.isfinite(marginal).all() or np.linalg.eigvalsh(marginal).min() < -1e-8:
            self.rejected_loops += 1
            self.last_result.update(accepted=False, reason='invalid_graph_marginal')
            return None
        self.last_result['position_marginal_covariance'] = marginal.tolist()
        self.last_result['wall_ms'] = (time.perf_counter()-started)*1000.
        knots = []
        for old, new in zip(original, optimized):
            R = new.rotation@old.rotation.T
            t = new.position-R@old.position
            knots.append((old.stamp, *Rotation.from_matrix(R).as_rotvec(), *t))
        self.poses, self.factors = optimized, factors
        self.accepted_loops += 1
        return MapCorrection(revision, original[0].stamp, original[-1].stamp,
            tuple(knots[-1][1:4]), tuple(knots[-1][4:]), tuple(knots))

    def metrics(self):
        return dict(nodes=len(self.poses), factor_count=len(self.factors), capacity=self.capacity,
            saturated=self.saturated, accepted_loops=self.accepted_loops, rejected_loops=self.rejected_loops,
            last_optimization=self.last_result, method='root-anchored SE3 graph, robust sparse least squares')

    def archive(self):
        return dict(schema_version=1, **self.metrics(),
            poses=[dict(stamp_s=p.stamp, position=p.position.tolist(), rotation=p.rotation.tolist()) for p in self.poses],
            factors=[dict(first=f.first, second=f.second, translation=f.translation.tolist(), rotation=f.rotation.tolist(),
                          sigma=f.sigma.tolist(), kind=f.kind) for f in self.factors])
