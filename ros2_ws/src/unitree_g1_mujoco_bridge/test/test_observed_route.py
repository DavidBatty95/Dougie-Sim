from types import SimpleNamespace as S

from unitree_g1_mujoco_bridge.observed_route import route_is_free, people_clear_corridor


def test_shortcuts_preserve_people_detours():
    assert not people_clear_corridor((0,0), (3,0), [(1.5,0,0,0,.42)], .38)
    assert not people_clear_corridor((0,0), (3,0), [(1.5,3,0,-1,.42)], .38)
    assert people_clear_corridor((0,0), (3,0), [(-3,0,-1,0,.42)], .38)


def fixture():
    grid = S(header=S(frame_id="map"), info=S(resolution=.1, width=100, height=100,
        origin=S(position=S(x=0., y=0.), orientation=S(x=0., y=0., z=0., w=1.))), data=[0] * 10000)
    path = S(header=S(frame_id="map"), poses=[S(pose=S(position=S(x=x, y=2.))) for x in (1., 4.)])
    return grid, path


def test_route_revalidation_checks_segment_interior_and_footprint():
    grid, path = fixture()
    assert route_is_free(grid, path, .35)
    grid.data[22 * 100 + 25] = 100
    assert not route_is_free(grid, path, .35)
    grid.data[22 * 100 + 25] = -1
    assert not route_is_free(grid, path, .35)


def test_route_revalidation_rejects_frame_mismatch_and_unobserved_space():
    grid, path = fixture()
    path.header.frame_id = "world"
    assert not route_is_free(grid, path, .35)
    path.header.frame_id = "map"
    path.poses[0].pose.position.x = -.2
    assert not route_is_free(grid, path, .35)
