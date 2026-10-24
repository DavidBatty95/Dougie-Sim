from dataclasses import replace

import numpy as np
import pytest

from dougie_sim.localization_checkpoint import LocalizationCheckpoints
from dougie_sim.map_corrections import MapCorrection
from dougie_sim.radiation import RadiationSample
from dougie_sim.sensing import RangePacket, support_mask


def floor_packet(stamp=.5, slope=0.):
    x, y = np.meshgrid(np.linspace(-1., 1., 21), np.linspace(-1., 1., 21))
    origin = np.array([0., 0., .4])
    delta = np.stack((x, y, slope*x), axis=-1)-origin
    distance = np.linalg.norm(delta, axis=2)
    directions = delta/distance[..., None]
    return RangePacket(stamp, 0, origin, directions, distance, 6., support_mask(origin, directions, distance, 6.))


def localization(stamp, std=.05, revision=1, rank=6):
    return dict(initialized=True, position_std_m=[std]*3, conditional_position_std_m=[.003]*3,
                observable_rank=rank, rejection_streak=0, scan_rmse_m=.003,
                last_good_scan_s=stamp, map_revision=revision, anchor_revalidations=0)


def test_checkpoint_requires_home_new_confirmation_and_low_uncertainty():
    checkpoint = LocalizationCheckpoints(1)
    checkpoint.request(localization(1., std=.19, revision=0))
    assert not checkpoint.step(2., localization(2.), False, True, 'localization_return_reserve')
    assert not checkpoint.step(3., localization(3., revision=0), True, True, 'localization_return_reserve')
    assert not checkpoint.step(4., localization(4., std=.19), True, True, 'localization_return_reserve')
    assert checkpoint.step(5., localization(5.), True, True, 'localization_return_reserve')
    assert checkpoint.used == 1 and not checkpoint.waiting
    checkpoint.request(localization(6., std=.19))
    assert checkpoint.origin is None  # Never exceed the declared number of resumptions.


def test_checkpoint_keeps_quality_budget_and_timeout_gates():
    checkpoint = LocalizationCheckpoints(1)
    checkpoint.request(localization(1., revision=0))
    assert not checkpoint.step(10., localization(10., rank=5), True, True, 'localization_return_reserve')
    assert not checkpoint.step(50., localization(50.), True, True, 'localization_return_reserve')
    assert checkpoint.outcome == 'revalidation_timeout'
    checkpoint.request(localization(51., revision=0))
    assert not checkpoint.step(52., localization(52.), True, False, 'localization_return_reserve')
    assert checkpoint.outcome == 'not_admissible'
    checkpoint.request(localization(53., revision=0))
    assert not checkpoint.step(54., localization(54.), True, True, 'measured_dose_reserve')
    assert checkpoint.origin is None


def test_worker_checkpoint_preserves_history_budget_and_deadline():
    from dougie_sim import planning_worker as worker
    worker.initialize((10., 100., 32, 600., 'poisson'), 'clustered', 'layered', options=dict(
        consistent_mapping=True, recovery_enabled=True, navigation_profile='robust',
        path_mode='grid', display_prediction=False, localization_checkpoints=1))
    sample = RadiationSample(.5, (0., 0., .4), 10, .5, 1.44, .1, .001, .002)
    _, state, metrics, _ = worker.plan([sample], floor_packet(), np.zeros(3), .5, False,
        localization=localization(.5, std=.19, revision=0))
    assert state == 'returning_checkpoint' and metrics['localization_checkpoints']['used'] == 0
    event = MapCorrection(1, 0., 1., (0., 0., 0.), (0., 0., 0.))
    sample = replace(sample, stamp_s=1.5, dose_usv=.004, dose_upper_usv=.005, map_revision=1)
    _, state, metrics, _ = worker.plan([sample], event.packet(floor_packet(1.5)), np.zeros(3), 1.5, False,
        corrections=(event,), localization=localization(1.5))
    assert state == 'checkpoint_revalidated' and metrics['localization_checkpoints']['used'] == 1
    result = worker.finalize([])
    assert not result.returning and result.sample.dose_usv == .004
    assert result.budget == 10. and result.mission_duration == 600.
    assert result.gp.updates == 2 and len(worker._journal.scans) == 2


@pytest.mark.parametrize('dose_upper,slope', [(8.1, 0.), (.002, .05)])
def test_worker_does_not_wait_for_checkpoint_with_high_dose_or_sloping_home(dose_upper, slope):
    from dougie_sim import planning_worker as worker
    worker.initialize((10., 100., 32, 600., 'poisson'), 'clustered', 'layered', options=dict(
        consistent_mapping=True, recovery_enabled=True, navigation_profile='robust',
        path_mode='grid', display_prediction=False, localization_checkpoints=1))
    sample = RadiationSample(.5, (0., 0., .4), 10, .5, 1.44, .1, .001, dose_upper)
    _, state, metrics, _ = worker.plan([sample], floor_packet(slope=slope), np.zeros(3), .5, False,
        localization=localization(.5, std=.19, revision=0))
    assert state == 'returned'
    assert metrics['localization_checkpoints']['outcome'] == 'not_admissible'
    assert metrics['localization_checkpoints']['used'] == 0
