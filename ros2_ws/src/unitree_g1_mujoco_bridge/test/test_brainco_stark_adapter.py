import pytest

from unitree_g1_mujoco_bridge.brainco_stark_adapter import stark_command


def test_normalized_finger_order_and_speed_units():
    positions,speeds = stark_command([0.,.1,.2,.3,.4,1.],[1.]*6)
    assert positions==[0,100,200,300,400,1000]
    assert speeds==[1000]*6


def test_zero_speed_holds_measurement_instead_of_sending_new_target():
    with pytest.raises(ValueError,match='fresh'):
        stark_command([1.]*6,[0.]*6)
    positions,speeds = stark_command([1.]*6,[0.]*6,[.25]*6)
    assert positions==[250]*6 and speeds==[1]*6


@pytest.mark.parametrize('value',[-.1,1.1,float('nan'),float('inf')])
def test_invalid_units_are_rejected(value):
    with pytest.raises(ValueError):
        stark_command([value]*6,[1.]*6)
