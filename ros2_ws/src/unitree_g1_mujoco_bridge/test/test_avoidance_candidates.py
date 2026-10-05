import math
from unitree_g1_mujoco_bridge.avoidance_candidates import crossing_escape_directions, velocity_obstacle_command


def test_approach_from_behind_proposes_perpendicular_escape():
    directions = crossing_escape_directions((0,0,0), [(-2,0,1,0)])
    assert len(directions) == 2
    assert all(abs(x) < 1e-9 and abs(y) == 1 for x,y in directions)


def test_receding_and_parallel_clear_people_do_not_trigger():
    assert crossing_escape_directions((0,0,0), [(2,0,1,0),(-2,3,1,0)]) == []


def test_escape_is_transformed_to_robot_heading():
    directions = crossing_escape_directions((0,0,math.pi/2), [(-2,0,1,0)])
    assert all(abs(x) > .99 and abs(y) < 1e-9 for x,y in directions)


def test_velocity_obstacle_takes_full_responsibility_for_a_head_on_person():
    command = velocity_obstacle_command((0,0,0),(0,0,0),(.6,0),[(3.,0,-1.,0,.9)],(.65,.45,.45),delay=0.)
    assert command is not None
    vx,vy = command
    t = max(0.,min(3.,3*(vx+1)/((vx+1)**2+vy*vy)))
    assert math.hypot(3-(vx+1)*t,-vy*t) >= .9-1e-8
    assert -.45 <= vx <= .65 and abs(vy) <= .45


def test_velocity_obstacle_preserves_an_unobstructed_preference():
    assert velocity_obstacle_command((0,0,0),(0,0,0),(.4,.1),[],(.65,.45,.45)) == (.4,.1)
