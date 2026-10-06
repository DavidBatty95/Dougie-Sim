import math

import pytest

@pytest.fixture
def api():
    torch = pytest.importorskip('torch')
    from payload_smoothness import PayloadMotion
    return torch,PayloadMotion


def test_stationary_upright_and_constant_velocity_have_no_false_acceleration(api):
    torch,PayloadMotion = api
    motion = PayloadMotion(1,'cpu',.02)
    q = torch.tensor([[1.,0.,0.,0.]])
    v = torch.tensor([[.3,0.,0.]])
    for _ in range(4):
        sample = motion.update(v,torch.zeros_like(v),q)
    assert sample['tilt_rad'].item() == 0.
    assert sample['effective_tilt_rad'].item() == 0.
    assert sample['acceleration_m_s2'].item() == 0.


def test_tilt_and_acceleration_are_both_visible_to_fluid_proxy(api):
    torch,PayloadMotion = api
    motion = PayloadMotion(1,'cpu',.02)
    angle = math.radians(20)
    q = torch.tensor([[math.cos(angle/2),0.,math.sin(angle/2),0.]])
    sample = motion.update(torch.zeros(1,3),torch.zeros(1,3),q)
    assert sample['tilt_rad'].item() == pytest.approx(angle,abs=1e-6)
    assert sample['effective_tilt_rad'].item() == pytest.approx(angle,abs=1e-6)
    motion.reset()
    q = torch.tensor([[1.,0.,0.,0.]])
    for step in range(30):
        sample = motion.update(torch.tensor([[step*.02*3.,0.,0.]]),torch.zeros(1,3),q)
    assert sample['effective_tilt_rad'].item() == pytest.approx(math.atan2(3.,9.81),abs=.001)


def test_partial_episode_reset_does_not_turn_teleportation_into_jerk(api):
    torch,PayloadMotion = api
    motion = PayloadMotion(2,'cpu',.02)
    q = torch.tensor([[1.,0.,0.,0.]]).repeat(2,1)
    for _ in range(3):
        motion.update(torch.zeros(2,3),torch.zeros(2,3),q)
    motion.reset(torch.tensor([0]))
    sample = motion.update(torch.tensor([[10.,0.,0.],[.1,0.,0.]]),torch.zeros(2,3),q)
    assert sample['raw_acceleration_m_s2'][0] == 0.
    assert not sample['derivative_valid'][0]
    assert sample['raw_acceleration_m_s2'][1] == pytest.approx(5.)
