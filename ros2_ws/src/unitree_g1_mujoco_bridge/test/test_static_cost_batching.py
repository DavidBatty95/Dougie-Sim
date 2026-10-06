import math

import pytest

from unitree_g1_mujoco_bridge.bridge_safety import trajectory_static_cost


@pytest.mark.parametrize('obstacle,clearance',[( (.8,.5),.3),((.4,0.),.3),((3.,4.),5.)])
@pytest.mark.parametrize('yaw',[0.,.7,-2.])
def test_scan_multiplicity_preserves_scalar_clearance_and_cost(obstacle,clearance,yaw):
    c,s = math.cos(yaw),math.sin(yaw)
    path = [(3.+c*x,-2.+s*x,yaw) for x in (0.,.1,.2,.3)]
    scalar = trajectory_static_cost(path,[obstacle],clearance,6.)
    batched = trajectory_static_cost(path,[obstacle]*2200,clearance,6.)
    assert batched[1] == scalar[1]
    assert batched[0] == pytest.approx(scalar[0],rel=1e-12)


def test_collision_in_later_obstacle_and_trajectory_blocks_is_not_missed():
    path = [(float(x),0.,0.) for x in range(70)]
    obstacles = [(0.,100.)]*2100+[(68.,.1)]
    assert trajectory_static_cost(path,obstacles,.2,1.) == (math.inf,True)


def test_batched_clearance_boundary_remains_strict():
    assert trajectory_static_cost([(0.,0.,0.)],[(3.,4.)]*200,5.,6.) == pytest.approx((1./36.,False))
