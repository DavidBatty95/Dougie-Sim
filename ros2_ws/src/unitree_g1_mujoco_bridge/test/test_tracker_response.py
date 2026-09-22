"""Motion-onset and stop latency regressions for measured-only tracking."""
import math
import random

from unitree_g1_mujoco_bridge.dynamic_obstacle_tracker import DynamicObstacleTracker, Track


def tracker_and_track():
    # The numerical filter does not require a ROS graph or node initialization.
    tracker = object.__new__(DynamicObstacleTracker)
    tracker.tracking_process_noise = 1.2
    tracker.measurement_noise_variance = .004
    track = Track(0., 0., 0., 0., 0.)
    return tracker, track


def test_walking_onset_after_a_long_idle_is_not_filtered_away():
    tracker, track = tracker_and_track()
    dt = 1/15
    for _ in range(150):
        tracker._kf_update(track, 0., 0., dt)
    for step in range(1, 10):
        t = step*dt
        tracker._kf_update(track, .5*1.2*t*t, 0., dt)
    # Actual speed is .72 m/s. More than .2 m/s lag here can hide an imminent
    # crossing after adding sensor transport and the locomotion response delay.
    assert track.vx >= .52


def test_stationary_centroid_noise_does_not_become_walking_speed():
    tracker, track = tracker_and_track()
    rng = random.Random(17)
    speeds = []
    for step in range(300):
        tracker._kf_update(track, rng.gauss(0., .025), rng.gauss(0., .025), 1/15)
        if step > 30:
            speeds.append(math.hypot(track.vx, track.vy))
    assert sorted(speeds)[int(.95*len(speeds))] < .20
