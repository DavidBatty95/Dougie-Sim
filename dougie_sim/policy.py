"""Versioned deployment contracts for stateless, TorchScript Go2 policies."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import torch

from .assets import ASSETS, ROOT

JOINTS = tuple(f'{leg}_{joint}_joint' for leg in ('FR', 'FL', 'RR', 'RL')
               for joint in ('hip', 'thigh', 'calf'))
DEFAULT_POLICY = ROOT/'config/dougie_policy.json'


class PolicyContract:
    """Fail closed on incompatible schemas, model assets, or changed weights.

    v1 is proprioceptive; v2 adds a defined subset of the existing LiDAR. Neither
    schema permits privileged height maps or unversioned observation changes.
    """
    def __init__(self, path=None):
        self.path = Path(path or DEFAULT_POLICY).resolve()
        payload = self.path.read_bytes()
        self.manifest_hash = hashlib.sha256(payload).hexdigest()
        self.config = json.loads(payload)
        c = self.config
        from .perception import OBSERVATIONS, SCHEMA, SENSOR_CONTRACT
        schema = (c['schema_version'], c['observation_schema'])
        if schema not in ((1, 'go2-proprio-45-v1'), (2, SCHEMA)) or tuple(c['joint_order']) != JOINTS:
            raise ValueError('unsupported Go2 policy schema or joint ordering')
        self.perceptive = schema[0] == 2
        self.observations = OBSERVATIONS if self.perceptive else 45
        if self.perceptive and c.get('range_sensor') != SENSOR_CONTRACT:
            raise ValueError('perceptive policy sensor contract does not match deployment')
        for name in ('policy_period_s', 'kp', 'kd', 'torque_limit_nm', 'observation_clip', 'action_clip'):
            if not math.isfinite(c[name]) or c[name] <= 0:
                raise ValueError(f'invalid policy parameter: {name}')
        if not math.isclose(c['policy_period_s'], .02) or c['torque_limit_nm'] > 23.5:
            raise ValueError('policy exceeds the current 50 Hz / 23.5 Nm deployment contract')
        for name in ('default_leg_pose', 'leg_action_scale'):
            values = np.asarray(c[name], dtype=float)
            if values.shape != (3,) or not np.isfinite(values).all():
                raise ValueError(f'invalid policy vector: {name}')
        if np.any(np.asarray(c['leg_action_scale']) <= 0):
            raise ValueError('action scales must be positive')
        self.default = np.tile(c['default_leg_pose'], 4)
        self.action_scale = np.tile(c['leg_action_scale'], 4)
        self.artifact = (self.path.parent/c['artifact']).resolve()

    def load(self):
        for path, expected in ((self.artifact, self.config['sha256']),
                               (ASSETS/'go2/go2.xml', self.config['model_sha256'])):
            if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                raise ValueError(f'policy/model checksum mismatch: {path}')
        actor = torch.jit.load(str(self.artifact), map_location='cpu').eval()
        with torch.inference_mode():
            action = actor(torch.zeros(1, self.observations))
        if action.shape != (1, 12) or not torch.isfinite(action).all():
            raise ValueError(f'policy must produce twelve finite actions from {self.observations} observations')
        return actor

    def identity(self):
        return dict(name=self.config['name'], schema=self.config['observation_schema'],
                    sha256=self.config['sha256'], manifest_sha256=self.manifest_hash,
                    qualification=self.config.get('qualification', 'unqualified'))
