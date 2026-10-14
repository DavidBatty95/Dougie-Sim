#!/usr/bin/env python3
"""Install the optional CUDA Go2 trainer in an isolated checkout-local venv."""
from pathlib import Path
import subprocess
import sys
import venv

ROOT = Path(__file__).resolve().parents[1]


def main():
    target = ROOT/'.dougie/training'
    if not (target/'bin/python').exists():
        venv.EnvBuilder(with_pip=True).create(target)
    subprocess.run([str(target/'bin/python'), '-m', 'pip', 'install', '-r',
                    str(ROOT/'config/dougie_training_requirements.txt')], check=True)
    subprocess.run([str(target/'bin/python'), '-c',
        "import torch; assert torch.cuda.is_available(), 'CUDA unavailable'; print(torch.cuda.get_device_name())"], check=True)
    print('Ready: .dougie/training/bin/python -m dougie_sim.training --smoke --num-envs 16 --iterations 2')


if __name__ == '__main__':
    main()
