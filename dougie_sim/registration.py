"""Quality-gated point-to-plane registration on detached observed submaps."""
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


@dataclass
class Registration:
    position: np.ndarray
    rotation: np.ndarray
    accepted: bool
    reason: str
    rmse: float
    inlier_fraction: float
    observable_rank: int
    translation_m: float
    rotation_rad: float


def register(points, normals, target, target_normals, position, rotation, max_translation=.18, max_rotation=.12):
    """Atomic fit using sensor-facing normals; reject opposite observed faces."""
    original_p, original_R = np.asarray(position).copy(), np.asarray(rotation).copy()
    p, R = original_p.copy(), original_R.copy()
    tree = cKDTree(target)
    rank, fraction, rmse = 0, 0., float('inf')
    reason = 'insufficient_correspondences'
    if min(len(points), len(target)) >= 40:
        for _ in range(8):
            rotated = points@R.T
            world = rotated+p
            distance, nearest = tree.query(world)
            agreement = np.sum((normals@R.T)*target_normals[nearest], axis=1)
            keep = (distance < .3) & (agreement > .8)
            fraction = float(keep.mean())
            if keep.sum() < 40 or fraction < .35:
                break
            n = target_normals[nearest[keep]]
            residual = np.sum(n*(target[nearest[keep]]-world[keep]), axis=1)
            rmse = float(np.sqrt(np.mean(residual**2)))
            robust = np.minimum(1., .025/np.maximum(np.abs(residual), 1e-8))
            A = np.column_stack((n, np.cross(rotated[keep], n)))
            H = A.T@(robust[:, None]*A)
            # Translation and rotation columns have different lever-arm scales.
            # Normalize before testing rank so a large floor does not erase a
            # smaller but informative wall direction. Truly unsupported columns
            # and correlated/degenerate directions still remain unobservable.
            supported = np.diag(H) > 2.
            scale = np.sqrt(np.maximum(np.diag(H), 1.))
            normalized = H/np.outer(scale, scale)
            normalized[~supported, :] = 0.
            normalized[:, ~supported] = 0.
            eigenvalues, vectors = np.linalg.eigh(normalized)
            observable = eigenvalues > .015
            rank = int(observable.sum())
            if rank < 3:
                reason = 'degenerate_registration'
                break
            gradient = A.T@(robust*residual)/scale
            gradient[~supported] = 0.
            delta = (vectors@(np.where(observable, 1./np.maximum(eigenvalues, 1e-8), 0.)*(vectors.T@gradient)))/scale
            p += delta[:3]
            R = Rotation.from_rotvec(delta[3:]).as_matrix()@R
            if np.linalg.norm(p-original_p) > max_translation or Rotation.from_matrix(R@original_R.T).magnitude() > max_rotation:
                reason = 'registration_innovation_limit'
                break
            if rmse > .08:
                reason = 'registration_residual_limit'
                break
            reason = 'registered' if rank == 6 else 'weak_geometry'
            if np.linalg.norm(delta) < 1e-4:
                break
        # Score the final pose rather than the pre-update residual.
        if reason in ('registered', 'weak_geometry'):
            world = points@R.T+p
            distance, nearest = tree.query(world)
            keep = (distance < .3) & (np.sum((normals@R.T)*target_normals[nearest], axis=1) > .8)
            fraction = float(keep.mean())
            rmse = float(np.sqrt(np.mean(np.sum(target_normals[nearest[keep]]*(target[nearest[keep]]-world[keep]), axis=1)**2))) if keep.any() else float('inf')
            if keep.sum() < 40 or fraction < .35 or rmse > .06:
                reason = 'registration_final_validation'
    accepted = reason in ('registered', 'weak_geometry')
    return Registration(p if accepted else original_p, R if accepted else original_R, accepted, reason,
        rmse, fraction, rank, float(np.linalg.norm(p-original_p)), float(Rotation.from_matrix(R@original_R.T).magnitude()))


@dataclass
class Keyframe:
    stamp: float
    position: np.ndarray
    rotation: np.ndarray
    points: np.ndarray
    normals: np.ndarray
    variance: float

    def world(self):
        return self.points@self.rotation.T+self.position, self.normals@self.rotation.T

    def correct(self, event):
        R, t = event.transform(self.stamp)
        self.position = R@self.position+t
        self.rotation = R@self.rotation
