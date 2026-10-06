#!/usr/bin/env python3
"""Profile RGB-D geometry on a retained transport snapshot (ROS environment)."""
import argparse
import cProfile
import json
import math
from pathlib import Path
import pstats

import numpy as np

from unitree_g1_mujoco_bridge.arm_kinematics import rpy_matrix
from unitree_g1_mujoco_bridge.transport_perception import colour_detections


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    args = parser.parse_args()
    snapshot = json.loads((args.run/'perception_snapshot.json').read_text())
    images = np.load(args.run/'rgbd.npz')
    # Geometry timing does not depend on the exact waist pose. Use the recorded
    # torso rotation when available, otherwise this is a timing-only replay.
    torso = np.asarray(snapshot.get('torso_rotation',rpy_matrix(snapshot['rpy'])))
    optical = np.array(((0.,0.,1.),(-1.,0.,0.),(0.,-1.,0.)))
    rotation = torso@rpy_matrix((0.,math.radians(snapshot['camera_pitch_deg']),0.))@optical
    profile = cProfile.Profile()
    profile.enable()
    detections = colour_detections(images['rgb'],images['depth'],images['K'],np.asarray(snapshot['origin']),rotation)
    profile.disable()
    print(json.dumps(detections,indent=2))
    pstats.Stats(profile).sort_stats('cumulative').print_stats(20)


if __name__=='__main__':
    main()
