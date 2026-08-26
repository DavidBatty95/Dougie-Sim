from types import SimpleNamespace

from g1_task_orchestration.mission_core import MissionState, validate_goal


def _goal(**overrides):
    values = {
        "object_id": "red_cube",
        "surface_object_id": "drop_pad",
        "arm": "right",
        "pickup_staging_pose": SimpleNamespace(
            header=SimpleNamespace(frame_id="map")
        ),
        "delivery_staging_pose": SimpleNamespace(
            header=SimpleNamespace(frame_id="map")
        ),
        "navigation_timeout_s": 0.0,
        "approach_timeout_s": 0.0,
        "retreat_distance_m": 0.0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_valid_goal_accepts_server_defaults():
    assert validate_goal(_goal()) == ""


def test_goal_validation_rejects_missing_identity_frame_and_bad_limits():
    assert "object_id" in validate_goal(_goal(object_id=""))
    assert "surface_object_id" in validate_goal(_goal(surface_object_id=""))
    assert "arm" in validate_goal(_goal(arm="both"))
    assert "pickup" in validate_goal(
        _goal(pickup_staging_pose=SimpleNamespace(header=SimpleNamespace(frame_id="")))
    )
    assert "non-negative" in validate_goal(_goal(retreat_distance_m=-0.1))


def test_authority_is_not_releasable_while_object_may_be_held():
    state = MissionState(authority_active=True)
    assert state.can_release_authority

    state.object_may_be_held = True
    assert not state.can_release_authority

    state.object_may_be_held = False
    assert state.can_release_authority
