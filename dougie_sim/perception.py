"""Shared sparse subset of the existing ideal LiDAR for perceptive policies.

No overhead height-map oracle: 288 occluded rays originate at the actual scanner.
Layout: proprio45, normalized ranges288, hit-validity288, scan age1 = 622 values.
"""
from dataclasses import dataclass
import math

import numpy as np

ELEVATION_INDICES = (1, 3, 5, 9, 14, 24, 39, 49, 50, 51, 52, 53)
AZIMUTH_STRIDE = 10
RANGE_M = 6.
PERIOD_S = .1
RAY_COUNT = len(ELEVATION_INDICES)*24
OBSERVATIONS = 45+2*RAY_COUNT+1
SCHEMA = 'go2-proprio-lidar-622-v2'
SENSOR_CONTRACT = dict(elevation_indices=list(ELEVATION_INDICES), azimuth_stride=AZIMUTH_STRIDE,
                       max_range_m=RANGE_M, period_s=PERIOD_S, origin_z_m=.12,
                       alignment='gravity_aligned_yaw', features='ranges_then_validity_then_age_over_0.2')


def beam_angles():
    azimuth = np.linspace(-math.pi, math.pi, 240, endpoint=False)[::AZIMUTH_STRIDE]
    elevation = np.r_[-np.arctan2(.45, np.arange(.12, 6.1, .12)), [0., .15, .35, .6]][list(ELEVATION_INDICES)]
    return azimuth, elevation


def range_features(distances, age_s):
    distances = np.asarray(distances).reshape(-1)
    if distances.shape != (RAY_COUNT,) or not np.isfinite(age_s) or age_s < 0:
        raise ValueError('invalid sparse LiDAR observation')
    valid = np.isfinite(distances) & (distances > .05) & (distances <= RANGE_M)
    ranges = np.where(valid, distances/RANGE_M, 1.)
    return np.r_[ranges, valid.astype(float), min(age_s/.2, 1.)].astype(np.float32)


@dataclass
class SparseLidarPattern:
    """mjlab structural ray-pattern interface, using exactly the deployment rays."""
    def generate_rays(self, mj_model, device):
        import torch
        azimuth, elevation = beam_angles()
        a, e = np.meshgrid(azimuth, elevation)
        directions = np.stack((np.cos(e)*np.cos(a), np.cos(e)*np.sin(a), np.sin(e)), axis=-1).reshape(-1, 3)
        offsets = np.zeros_like(directions)
        offsets[:, 2] = .12  # yaw alignment preserves this vertical offset
        return (torch.tensor(offsets, dtype=torch.float32, device=device),
                torch.tensor(directions, dtype=torch.float32, device=device))


class TrainingLidarObservation:
    """10 Hz sample-and-hold, reset-aware, with training-only noise/dropout."""
    def __init__(self, cfg, env):
        import torch
        self.cached = torch.ones((env.num_envs, 2*RAY_COUNT), device=env.device)
        self.cached[:, RAY_COUNT:] = 0.
        self.stamp = torch.full((env.num_envs,), -1e6, device=env.device)

    def reset(self, env_ids=None):
        self.stamp[slice(None) if env_ids is None else env_ids] = -1e6

    def __call__(self, env, corrupt=True):
        import torch
        now = env.episode_length_buf.float()*env.step_dt
        due = (now-self.stamp >= PERIOD_S-1e-5) | (now < self.stamp)
        if corrupt:
            due &= (torch.rand_like(now) > .03) | (self.stamp < 0.)
        raw = env.scene['policy_lidar'].data.distances
        valid = torch.isfinite(raw) & (raw > .05) & (raw <= RANGE_M)
        if corrupt:
            valid &= torch.rand_like(raw) > .03
            raw = (raw+torch.randn_like(raw)*.01).clamp(.05, RANGE_M)
        features = torch.cat((torch.where(valid, raw/RANGE_M, 1.), valid.float()), dim=1)
        self.cached[due] = features[due]
        self.stamp[due] = now[due]
        age = ((now-self.stamp)/.2).clamp(0., 1.)[:, None]
        return torch.cat((self.cached, age), dim=1)


class CoherentActorObservation:
    """Assemble one timestamped actor vector before applying a shared delay.

    Independent delays on positions, velocities and previous actions create
    physically inconsistent state vectors. The observation manager delays this
    whole vector together, matching the native delayed-policy qualification.
    """
    def __init__(self, cfg, env):
        self.lidar = TrainingLidarObservation(cfg, env)

    def reset(self, env_ids=None):
        self.lidar.reset(env_ids)

    def __call__(self, env, asset_cfg, perceptive=True):
        import torch
        data = env.scene[asset_cfg.name].data
        ids = asset_cfg.joint_ids
        proprio = torch.cat((data.root_link_ang_vel_b*.25, data.projected_gravity_b,
            env.command_manager.get_command('twist'),
            data.joint_pos[:, ids]-data.default_joint_pos[:, ids], data.joint_vel[:, ids]*.05,
            env.action_manager.action), dim=1).clamp(-100., 100.)
        return torch.cat((proprio, self.lidar(env)), dim=1) if perceptive else proprio
