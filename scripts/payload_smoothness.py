"""Fluid-carry-inspired rigid-payload metrics, not a liquid/slosh simulation.

Only rewards and evaluation use these simulator measurements. No new privileged
actor input is introduced. Derivatives reset independently for each environment.
"""
import math

import torch


def rotate_vector(quaternion,vector):
    """Apply a world-frame wxyz quaternion to a local vector."""
    xyz = quaternion[...,1:]
    cross = 2.*torch.cross(xyz,vector,dim=-1)
    return vector+quaternion[...,:1]*cross+torch.cross(xyz,cross,dim=-1)


class PayloadMotion:
    def __init__(self,count,device,dt,up_axis=(0.,0.,1.),filter_tau=.08):
        self.dt = dt
        self.alpha = 1.-math.exp(-dt/filter_tau)
        self.up_axis = torch.tensor(up_axis,device=device,dtype=torch.float32).expand(count,-1)
        self.velocity = torch.zeros(count,3,device=device)
        self.acceleration = torch.zeros_like(self.velocity)
        self.omega = torch.zeros_like(self.velocity)
        self.count = torch.zeros(count,device=device,dtype=torch.long)

    def reset(self,ids=None):
        ids = slice(None) if ids is None else ids
        self.count[ids] = 0
        self.velocity[ids] = 0.
        self.acceleration[ids] = 0.
        self.omega[ids] = 0.

    def update(self,velocity,omega,quaternion):
        initialized = self.count>0
        raw_accel = torch.where(initialized[:,None],(velocity-self.velocity)/self.dt,0.)
        filtered_accel = self.acceleration+self.alpha*(raw_accel-self.acceleration)
        jerk = torch.where((self.count>1)[:,None],(filtered_accel-self.acceleration)/self.dt,0.)
        angular_accel = torch.where(initialized[:,None],(omega-self.omega)/self.dt,0.)
        up = rotate_vector(quaternion,self.up_axis)
        apparent_up = filtered_accel.clone()
        apparent_up[:,2] += 9.81
        normal = (apparent_up*up).sum(dim=-1)
        tangent = torch.linalg.vector_norm(apparent_up-normal[:,None]*up,dim=-1)
        effective_angle = torch.atan2(tangent,normal)
        result = dict(
            tilt_rad=torch.acos(up[:,2].clamp(-1.,1.)),
            effective_tilt_rad=effective_angle,
            raw_acceleration_m_s2=torch.linalg.vector_norm(raw_accel,dim=-1),
            acceleration_m_s2=torch.linalg.vector_norm(filtered_accel,dim=-1),
            lateral_acceleration_m_s2=torch.linalg.vector_norm(filtered_accel[:,:2],dim=-1),
            jerk_m_s3=torch.linalg.vector_norm(jerk,dim=-1),
            angular_acceleration_rad_s2=torch.linalg.vector_norm(angular_accel,dim=-1),
            tilt_rate_rad_s=torch.linalg.vector_norm(omega[:,:2],dim=-1),
            normal_specific_force_m_s2=normal,
            derivative_valid=self.count>1,
        )
        self.velocity.copy_(velocity)
        self.acceleration.copy_(filtered_accel)
        self.omega.copy_(omega)
        self.count += 1
        return result


class FluidCarryPenalty:
    """A bounded physical-motion cost, keeping velocity tracking rewards active."""
    def __init__(self,cfg,env):
        self.robot = env.scene['robot']
        ids,_ = self.robot.find_bodies('carried_payload')
        self.body = ids[0]
        self.motion = PayloadMotion(env.num_envs,env.device,env.step_dt,
                                    up_axis=cfg.params.get('up_axis',(0.,0.,1.)))

    def reset(self,env_ids=None):
        self.motion.reset(env_ids)

    def __call__(self,env,up_axis=(0.,0.,1.)):
        del up_axis
        data = self.robot.data
        measured = self.motion.update(data.body_link_lin_vel_w[:,self.body],
            data.body_link_ang_vel_w[:,self.body],data.body_link_quat_w[:,self.body])
        def quadratic(value,scale):
            return (value/scale).square().clamp_max(10.)
        cost = (.4*quadratic(measured['tilt_rad'],.15)+
                .6*quadratic(measured['effective_tilt_rad'],.20)+
                .15*quadratic(measured['lateral_acceleration_m_s2'],3.)+
                .03*quadratic(measured['jerk_m_s3'],20.)+
                .05*quadratic(measured['angular_acceleration_rad_s2'],3.)+
                .10*quadratic(measured['tilt_rate_rad_s'],.3)+
                .30*quadratic(torch.relu(5.-measured['normal_specific_force_m_s2']),5.))
        action = env.action_manager.get_term('joint_pos')
        active = action.held & (env.episode_length_buf*env.step_dt>5.) & measured['derivative_valid']
        return torch.where(active,cost,0.)


LOADED_TURN_SCHEDULE = (
    ('settle',6.,(0.,0.,0.)),
    ('forward',4.,(.3,0.,0.)),('stop_forward',3.,(0.,0.,0.)),
    ('left_pivot',4.,(0.,0.,.6)),('stop_left',3.,(0.,0.,0.)),
    ('right_pivot',4.,(0.,0.,-.9)),('stop_right',3.,(0.,0.,0.)),
    ('left_arc',4.,(.3,0.,.35)),('right_arc',4.,(.3,0.,-.35)),
    ('left_restart',3.,(0.,0.,.6)),('yaw_reversal',3.,(0.,0.,-.9)),
    ('final_stop',4.,(0.,0.,0.)),
)
