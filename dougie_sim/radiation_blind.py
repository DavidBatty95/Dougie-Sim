"""Explicit radiation ablation: geometry/time remain available, exposure does not."""
from dataclasses import replace

import numpy as np

from .gp import PoissonOnlineGP


def blind_sample(sample):
    return replace(sample, counts=0, rate_usv_h=0., variance_usv_h2=0.,
                   dose_usv=0., dose_upper_usv=0.)


class BlindGP(PoissonOnlineGP):
    """Zero risk and zero radiation information gain, independent of observations."""
    def predict(self, query):
        return tuple(np.zeros(len(query)) for _ in range(3))

    def update_counts(self, *args, **kwargs):
        pass

    def update_counts_window(self, *args, **kwargs):
        pass
