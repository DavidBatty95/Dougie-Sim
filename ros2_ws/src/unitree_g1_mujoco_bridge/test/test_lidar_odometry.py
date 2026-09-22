import numpy as np

from unitree_g1_mujoco_bridge.lidar_odometry import ScanOdometry, rotation


def room():
    axis = np.linspace(-4, 4, 500)
    return np.concatenate((np.column_stack((axis, np.full_like(axis, 4))),
                           np.column_stack((np.full_like(axis, 4), axis)),
                           np.column_stack((axis, np.full_like(axis, -4))),
                           np.column_stack((np.full_like(axis, -4), axis))))


def test_estimates_translation_with_heading_and_moving_outliers():
    rng = np.random.default_rng(6)
    world = room()
    odom = ScanOdometry()
    assert odom.update(world, 0.7)
    for index in range(1, 20):
        xy = np.array((index * 0.04, index * -0.025))
        yaw = index * 0.008
        local = (world - xy) @ rotation(yaw)
        local += rng.normal(0, 0.007, local.shape)
        local = np.concatenate((local, rng.uniform(-3, 3, (80, 2))))
        assert odom.update(local, 0.7 + yaw)
        assert np.linalg.norm(odom.xy - xy) < 0.12


def test_failed_overlap_cannot_update_pose_or_reference():
    odom = ScanOdometry()
    assert odom.update(room(), 0.)
    previous = odom.reference.copy()
    assert not odom.update(room() + 50, 0.)
    np.testing.assert_array_equal(odom.xy, (0., 0.))
    np.testing.assert_array_equal(odom.reference, previous)
    assert not odom.update(np.empty((0, 2)), 0.)
