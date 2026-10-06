import math

import numpy as np

from unitree_g1_mujoco_bridge.human_collision_avoidance import prepare_predictions,time_aligned_prediction_risk
from unitree_g1_mujoco_bridge.motion_envelope import MotionEnvelope


def test_preparation_preserves_risk_over_commands_ages_horizons_and_covariances():
    rng = np.random.default_rng(44)
    tracks = []
    for _ in range(4):
        x,y,vx,vy = rng.normal(size=4)
        tracks.append([(float(t),x+vx*t,y+vy*t,.002+.05*t,.003+.08*t,.9)
                       for t in np.linspace(0.,6.,31)])
    prepared = prepare_predictions(tracks,.1)
    envelope = MotionEnvelope(max_x=.65,max_y=.45,max_yaw=.9)
    for age in (0.,.15,.4):
        for command in ((0.,0.,0.),(.5,.2,0.),(-.4,.3,.5)):
            for horizon in (1.,3.,6.):
                rollout = envelope.rollout((0.,0.,.4),command,(.2,0.,0.),horizon,.1)
                args = (age,(0.,0.,.4),command,horizon,.1,math.inf,.5,.4,rollout)
                assert time_aligned_prediction_risk(prepared,*args) == time_aligned_prediction_risk(tracks,*args)


def test_invalid_and_out_of_order_points_do_not_create_a_cached_phantom():
    tracks = [[(0.,3.,0.,0.,0.,1.),(0.,0.,0.,0.,0.,1.),
               (.1,float('nan'),0.,0.,0.,1.),(.2,3.,0.,0.,0.,1.)]]
    prepared = prepare_predictions(tracks,.1)
    args = (0.,(0.,0.,0.),(0.,0.,0.),.2,.1,1.,0.,.5)
    assert time_aligned_prediction_risk(prepared,*args) is None
    tracks[0][0] = (0.,0.,0.,0.,0.,1.)
    assert time_aligned_prediction_risk(prepared,*args) is None
    assert time_aligned_prediction_risk(tracks,*args) is not None


def test_age_and_sampling_resolution_are_not_cached_with_the_message():
    tracks = [[(0.,-1.,0.,0.,0.,1.),(1.,1.,0.,0.,0.,1.)]]
    prepared = prepare_predictions(tracks,.5)
    args = ((0.,0.,0.),(0.,0.,0.),.4,.1,.25,0.,.5)
    assert time_aligned_prediction_risk(prepared,.3,*args) == time_aligned_prediction_risk(tracks,.3,*args)
    assert time_aligned_prediction_risk(prepared,.3,*args) is not None
    assert time_aligned_prediction_risk(prepared,.9,*args) is None
