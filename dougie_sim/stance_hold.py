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


class GravityStanceHold:
    """One bounded entry attempt; loss of valid support permanently refuses it.

    This returns policy-format joint targets, leaving existing PD gains, action
    clipping and motor torque limits in the actuator path unchanged.
    """
    def __init__(self, stamp, mass, default, action_scale, kp):
        self.requested_s, self.mass = float(stamp), float(mass)
        self.default, self.scale, self.kp = np.asarray(default).copy(), np.asarray(action_scale).copy(), float(kp)
        if (self.default.shape != (12,) or self.scale.shape != (12,)
                or not np.isfinite(np.r_[self.requested_s, self.mass, self.default, self.scale, self.kp]).all()
                or self.mass <= 0 or self.kp <= 0 or np.any(self.scale <= 0)):
            raise ValueError('invalid gravity stance configuration')
        self.phase, self.reason = 'waiting', None
        self.quiet_since = self.started_s = None
        self.joints = self.entry_action = None
        self.maximum_support_torque = 0.

    def update(self, measured, rotation, last_action, stamp):
        if self.phase == 'refused':
            return None
        fresh = 0 <= stamp-measured.stamp_s <= .05
        if self.phase == 'waiting':
            if stamp-self.requested_s > 3.:
                self.phase, self.reason = 'refused', 'stance_entry_deadline'
                return None
            quiet = (fresh and stamp-self.requested_s >= .5 and np.all(measured.foot_force > 5.)
                     and np.linalg.norm(measured.gyro) <= .15 and np.max(np.abs(measured.joint_velocity)) <= .5)
            if not quiet:
                self.quiet_since = None
                return None
            if self.quiet_since is None:
                self.quiet_since = stamp
            if stamp-self.quiet_since < .2:
                return None
            self.started_s, self.phase = float(stamp), 'holding'
            self.joints, self.entry_action = measured.joints.copy(), np.asarray(last_action).copy()
        try:
            if not fresh or np.any(measured.foot_force <= 5.) or np.linalg.norm(measured.gyro) > .3:
                raise ValueError('stance lost fresh quiet four-foot support')
            torque, _ = support_torques(measured.joints, np.asarray(rotation).T@np.array([0., 0., 1.]), self.mass)
            self.maximum_support_torque = max(self.maximum_support_torque, float(np.max(np.abs(torque))))
            desired = (self.joints+torque/self.kp-self.default)/self.scale
            blend = min(1., (stamp-self.started_s)/.5)
            return (1.-blend)*self.entry_action+blend*desired
        except ValueError as exc:
            self.phase, self.reason = 'refused', str(exc)
            return None

    def metrics(self):
        return dict(phase=self.phase, reason=self.reason, requested_s=self.requested_s,
                    started_s=self.started_s, maximum_support_torque_nm=self.maximum_support_torque,
                    robot_model_mass_kg=self.mass, scope='experimental proprioceptive gravity stance; no position feedback')
