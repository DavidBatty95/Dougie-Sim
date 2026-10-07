import math

import numpy as np
import pytest

from dougie_sim.assets import ensure_assets
from dougie_sim.simulation import DEFAULT, JOINTS, Simulation, rotation


@pytest.fixture(scope='session', autouse=True)
def assets():
    ensure_assets()


def test_go2_joint_contract_uses_named_joints_not_qpos_order():
    sim = Simulation()
    assert sim.model.nu == 12 and len(JOINTS) == 12
    np.testing.assert_allclose(sim.data.qpos[sim.qadr], DEFAULT)
    assert len(set(sim.qadr)) == 12
    assert sim.observation((0, 0, 0)).shape == (45,)
    for joint, actuator in zip(sim.joint_ids, sim.actuators):
        assert sim.model.actuator_trnid[actuator, 0] == joint
    np.testing.assert_allclose(sim.observation((.2, .1, .3))[6:9], [.2, .1, .3])


@pytest.mark.parametrize('controller', ['stand', 'policy'])
def test_stays_upright_under_physics(controller):
    sim = Simulation(controller=controller)
    for _ in range(2000):
        sim.step()
    assert .2 < sim.data.qpos[2] < .45
    assert rotation(sim.data.qpos[3:7])[2, 2] > .9
    assert np.linalg.norm(sim.data.qpos[:2]) < .5


def test_policy_walks_forward_without_root_actuation():
    sim = Simulation()
    for _ in range(1200):
        sim.step((.3, 0., 0.))
    assert sim.data.qpos[0] > .6
    assert sim.model.nmocap == 0
    assert not np.any(sim.data.xfrc_applied)
    assert np.max(np.abs(sim.data.ctrl)) <= 23.5


def test_policy_turns_in_both_directions():
    headings = []
    for command in (-.5, .5):
        sim = Simulation()
        for _ in range(800):
            sim.step((0., 0., command))
        matrix = rotation(sim.data.qpos[3:7])
        headings.append(math.atan2(matrix[1, 0], matrix[0, 0]))
    assert headings[0] < -.2 and headings[1] > .2


def test_virtual_scan_sees_walls_and_moving_people():
    room = Simulation('room')
    ranges = room.scan()
    assert len(ranges) == 180
    assert ranges[90] == pytest.approx(5.9, abs=.05)
    people = Simulation('room', people=1)
    for _ in range(200):
        people.step()
    assert min(people.scan()) < 3.


def test_invalid_commands_cannot_reach_policy():
    sim = Simulation()
    with pytest.raises(ValueError):
        sim.step((float('nan'), 0., 0.))
    with pytest.raises(ValueError):
        sim.step((1., 2.))
