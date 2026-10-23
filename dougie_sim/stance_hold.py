"""Experimental quasi-static support torques from the known Go2 leg model.

No world pose, terrain model or simulator state is accepted. This load-support
approximation is not full inverse dynamics: limb gravity and disturbances are
not explicitly modelled. It is screened offline before any mission integration.
"""
import numpy as np

from .odometry import leg_kinematics


def support_torques(joints, body_up, robot_mass_kg):
    joints, up = np.asarray(joints, dtype=float), np.asarray(body_up, dtype=float)
    if (joints.shape != (12,) or up.shape != (3,) or not np.isfinite(joints).all()
            or not np.isfinite(up).all() or np.linalg.norm(up) < .5
            or not np.isfinite(robot_mass_kg) or not 0 < robot_mass_kg <= 30.):
        raise ValueError('invalid stance model inputs')
    up = up/np.linalg.norm(up)
    feet, _ = leg_kinematics(joints, np.zeros(12))
    # Nonnegative gravity-aligned forces balance total weight and moment about
    # the base origin (an approximate COM). Refuse unsupported load distributions.
    balance = np.vstack((np.tile(up[:, None], (1, 4)), np.cross(feet, up).T))
    target = np.r_[robot_mass_kg*9.81*up, np.zeros(3)]
    loads, _, rank, _ = np.linalg.lstsq(balance, target, rcond=None)
    if rank < 3 or np.min(loads) < 5. or np.linalg.norm(balance@loads-target) > 1e-6*np.linalg.norm(target):
        raise ValueError('gravity projection is outside the supported stance')
    forces = loads[:, None]*up
    torque = np.empty(12)
    for index, velocity in enumerate(np.eye(12)):
        _, foot_velocity = leg_kinematics(joints, velocity)
        torque[index] = -float(np.sum(foot_velocity*forces))
    if np.max(np.abs(torque)) > 12.:
        raise ValueError('stance feedforward exceeds the experimental torque limit')
    return torque, loads
