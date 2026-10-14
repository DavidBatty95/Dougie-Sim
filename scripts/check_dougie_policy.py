#!/usr/bin/env python3
"""Check a training export against saved actions in the native CPU runtime."""
import argparse
from pathlib import Path

import numpy as np
import torch

from dougie_sim.policy import PolicyContract


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    args = parser.parse_args()
    contract = PolicyContract(args.manifest)
    actor = contract.load()
    with np.load(args.manifest.parent/'export_parity.npz') as saved:
        with torch.inference_mode():
            actual = actor(torch.from_numpy(saved['observations'])).numpy()
        np.testing.assert_allclose(actual, saved['actions'], rtol=1e-4, atol=1e-5)
        print(f"CPU export parity passed: {len(actual)} observations; {contract.config['name']}")
    print('Action parity is not physical locomotion qualification.')


if __name__ == '__main__':
    main()
