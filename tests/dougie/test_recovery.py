from dataclasses import replace
from types import SimpleNamespace as S

import numpy as np

from dougie_sim.recovery import BudgetedRecovery, failure_reason
from dougie_sim.radiation import RadiationSample
from dougie_sim.sensing import RangePacket


def step(recovery, reason, stamp, valid=False, measured=.01, budget=2., reserve=.3, rate=10.):
    return recovery.step(reason, stamp, measured, measured+.001, rate, budget, reserve, 600., 20., valid)


def test_changing_failure_reason_does_not_restart_recovery_or_hide_exposure():
    recovery = BudgetedRecovery()
    assert step(recovery, 'missing_local_support', 0.).phase == 'resensing'
    assert step(recovery, 'stale_range', 5., measured=.015).phase == 'checked_retreat'
    assert step(recovery, 'return_route_disconnected', 12., measured=.02).terminal
    assert recovery.episodes == 1 and recovery.measured_dose == .01
    assert step(recovery, None, 20., valid=True, measured=.03).terminal


def test_wait_exposure_consumes_return_margin_and_forces_immediate_retreat():
    recovery = BudgetedRecovery()
    result = step(recovery, 'motion_stalled', 0., valid=True, budget=.2, reserve=.15, rate=100.)
    assert result.phase == 'return_now' and result.force_return
    assert abs(recovery.wait_upper_dose-100.*3./3600.) < 1e-12
    impossible = step(BudgetedRecovery(), 'missing_local_support', 0., budget=.2, reserve=.15, rate=100.)
    assert impossible.terminal


def test_fresh_route_recovers_and_retreat_attempts_are_bounded():
    recovery = BudgetedRecovery()
    step(recovery, 'motion_stalled', 0.)
    assert step(recovery, None, 4., valid=True).phase == 'recovered'
    assert recovery.started is None
    step(recovery, 'motion_stalled', 10.)
    assert recovery.episodes == 2
    assert recovery.allow_attempt(14.)
    assert not recovery.allow_attempt(15.)
    assert recovery.allow_attempt(21.)
    assert not recovery.allow_attempt(28.)


def test_failure_diagnostics_separate_localization_counts_ranges_and_support():
    sample = RadiationSample(1., (0., 0., .4), 5, .5, .72, .1, .001, .002)
    explorer = S(sample=sample, state='waiting_for_support', path=[])
    packet = S(stamp_s=1.)
    assert failure_reason(explorer, packet, 1., {}, False, None) == 'missing_local_support'
    assert failure_reason(explorer, packet, 2., {}, False, None) == 'stale_range'
    assert failure_reason(explorer, packet, 3., {}, False, None) == 'stale_radiation'
    assert failure_reason(explorer, packet, 1., dict(initialized=True, rejection_streak=5), False, None) == 'localization_unreliable'


def test_worker_never_moves_without_support_and_eventually_reports_terminal_stop():
    from dougie_sim import planning_worker as worker
    worker.initialize((2., 100., 32, 600., 'poisson'), 'clustered', 'conservative',
                      options=dict(recovery_enabled=True, display_prediction=False))
    sample = RadiationSample(1., (0., 0., .4), 5, .5, .72, .1, .001, .002)
    for stamp in (1., 3., 5., 7., 9., 11., 13.):
        packet = RangePacket(stamp, 0, np.array([0., 0., .4]), np.ones((3, 8, 3)),
                             np.full((3, 8), -1.), 6., np.zeros((3, 8), dtype=bool))
        path, state, metrics, _ = worker.plan([replace(sample, stamp_s=stamp)], packet, np.zeros(3), stamp, False)
        assert not path
    assert state == 'stopped_unrecoverable'
    assert metrics['recovery']['terminal']
    assert metrics['recovery']['reason'] == 'missing_local_support'


def test_lidar_fault_is_missing_measurements_not_fake_clear_space():
    from dougie_sim.onboard_sensors import OnboardSensors
    from dougie_sim.simulation import Simulation
    sim = Simulation('room')
    packet = OnboardSensors(lidar_dropout_after=0.).scan(sim)
    assert not packet.valid.any() and not packet.support_mask.any()
    from dougie_sim.layered_terrain import VoxelEvidence
    occupancy = VoxelEvidence()
    occupancy.update(packet)
    assert not occupancy.evidence


def test_detector_windows_are_deferred_until_available_not_misclassified_as_stale():
    from dougie_sim.radiation import Detector, available_samples
    detector = Detector(3)
    for i in range(100):
        sample = detector.integrate([0., 0., .4], 10., .005, (i+1)*.005)
    ready, pending = available_samples([sample], .495)
    assert not ready and pending == [sample]
    ready, pending = available_samples(pending, .5)
    assert ready == [sample] and not pending
    explorer = S(sample=ready[0], state='exploring', path=[np.zeros(3)])
    assert failure_reason(explorer, S(stamp_s=.4), .5, {}, False, 'tracking') is None
