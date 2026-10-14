"""Bounded deterministic gait corrections for optional GPU policy training."""
import math

import torch


class BoundedResidual(torch.nn.Module):
    """Freeze the reference gait; bound the mean correction in policy action units.

    PPO exploration noise is applied by the trainer after this deterministic mean.
    The bound is not a bound on sampled training actions or closed-loop motion.
    """
    def __init__(self, baseline, correction, scale):
        super().__init__()
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError('residual scale must be positive and finite')
        self.baseline = baseline.eval()
        for parameter in self.baseline.parameters():
            parameter.requires_grad_(False)
        self.correction = correction
        self.scale = float(scale)

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        return self.baseline(observation[..., :45])+self.scale*torch.tanh(self.correction(observation))
