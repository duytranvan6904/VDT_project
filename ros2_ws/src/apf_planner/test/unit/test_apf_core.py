import math

import numpy as np
import pytest

from apf_planner.apf_core import APFCore, APFParams, make_follow_goal


def v3(*a):
    return np.array(a, dtype=float)


def test_follow_goal_3d_standoff():
    goal = make_follow_goal(v3(0, 0, 3), v3(10, 0, 0), 5.0, cruise_altitude=3.0)
    assert goal == pytest.approx([6.0, 0.0, 3.0])


def test_follow_goal_planar_standoff():
    goal = make_follow_goal(v3(0, 0, 3), v3(10, 0, 0), 5.0, is_3d_distance=False)
    assert goal == pytest.approx([5.0, 0.0, 3.0])


def test_follow_goal_distance_not_larger_than_dz_collapses_to_target_xy():
    goal = make_follow_goal(v3(0, 0, 3), v3(10, 2, 0), 2.0, cruise_altitude=3.0)
    assert goal == pytest.approx([10.0, 2.0, 3.0])


def test_follow_goal_without_altitude_hold_keeps_target_z():
    goal = make_follow_goal(v3(0, 0, 3), v3(10, 0, 0), 5.0, hold_altitude=False)
    assert goal == pytest.approx([6.0, 0.0, 0.0])


def test_follow_goal_drone_above_target_uses_default_direction():
    goal = make_follow_goal(v3(10, 0, 3), v3(10, 0, 0), 5.0, cruise_altitude=3.0)
    assert goal == pytest.approx([6.0, 0.0, 3.0])


def test_follow_goal_direction_follows_bearing():
    goal = make_follow_goal(v3(0, 0, 3), v3(0, 10, 0), 5.0, cruise_altitude=3.0)
    assert goal == pytest.approx([0.0, 6.0, 3.0])


def test_follow_goal_rejects_bad_shapes():
    with pytest.raises(ValueError):
        make_follow_goal(np.zeros(2), np.zeros(3), 3.0)
    with pytest.raises(ValueError):
        make_follow_goal(np.zeros(3), np.zeros(4), 3.0)


def test_follow_goal_does_not_mutate_inputs():
    drone, target = v3(0, 0, 3), v3(10, 0, 0)
    make_follow_goal(drone, target, 5.0)
    assert drone.tolist() == [0, 0, 3]
    assert target.tolist() == [10, 0, 0]


def test_params_defaults():
    p = APFParams()
    assert (p.d0, p.v_max, p.d_slow, p.k_att, p.k_rep) == (3.0, 1.2, 1.5, 10.0, 2500.0)
    assert (p.goal_threshold, p.k_z, p.vz_max) == (0.2, 0.6, 0.5)


def test_at_goal_stops():
    r = APFCore().compute(v3(0, 0, 3), v3(0.1, 0, 3), [])
    assert r.at_goal
    assert r.velocity == pytest.approx([0, 0, 0])


def test_far_goal_full_speed_straight():
    r = APFCore().compute(v3(0, 0, 0), v3(10, 0, 0), [])
    assert not r.at_goal
    assert r.velocity == pytest.approx([1.2, 0.0, 0.0])
    assert r.yaw_cmd == pytest.approx(0.0)


def test_deceleration_ramp_near_goal():
    p = APFParams()
    r = APFCore(p).compute(v3(0, 0, 0), v3(0.8, 0, 0), [])
    scale = (0.8 - p.goal_threshold) / (p.d_slow - p.goal_threshold)
    assert r.velocity[0] == pytest.approx(p.v_max * scale, rel=1e-6)


def test_speed_never_exceeds_vmax():
    core = APFCore()
    for goal in ([50, 0, 0], [3, 4, 0], [-20, 15, 0]):
        r = core.compute(v3(0, 0, 0), v3(*goal), [])
        assert math.hypot(r.velocity[0], r.velocity[1]) <= core.params.v_max + 1e-9


def test_obstacle_ahead_generates_repulsion_and_tangent():
    r = APFCore().compute(v3(0, 0, 0), v3(10, 0, 0), [(1.0, 0.0, 0.0)])
    assert r.f_rep[0] < 0
    assert r.f_rep[1] < 0
    assert r.velocity[1] < 0


def test_tangent_direction_follows_goal_side():
    r = APFCore().compute(v3(0, 0, 0), v3(10, 0, 0), [(1.0, -0.05, 0.0)])
    assert r.f_rep[1] > 0
    assert r.velocity[1] > 0


def test_obstacle_outside_influence_ignored():
    r = APFCore().compute(v3(0, 0, 0), v3(10, 0, 0), [(0.0, 5.0, 0.0)])
    assert r.f_rep == pytest.approx([0, 0, 0])


def test_influence_radius_shrinks_near_goal():
    r = APFCore().compute(v3(0, 0, 0), v3(0.5, 0, 0), [(-1.0, 0.0, 0.0)])
    assert r.f_rep == pytest.approx([0, 0, 0])


def test_k_rep_override_scales_repulsion():
    core = APFCore()
    obs = [(1.0, 0.0, 0.0)]
    base = core.compute(v3(0, 0, 0), v3(10, 0, 0), obs)
    double = core.compute(v3(0, 0, 0), v3(10, 0, 0), obs, k_rep_override=2 * core.params.k_rep)
    zero = core.compute(v3(0, 0, 0), v3(10, 0, 0), obs, k_rep_override=0.0)
    assert np.linalg.norm(double.f_rep) == pytest.approx(2 * np.linalg.norm(base.f_rep))
    assert zero.f_rep == pytest.approx([0, 0, 0])


def test_obstacle_on_drone_position_is_finite():
    r = APFCore().compute(v3(0, 0, 0), v3(10, 0, 0), [(0.0, 0.0, 0.0)])
    assert np.isfinite(r.velocity).all()


def test_obstacle_z_is_ignored_in_horizontal_force():
    core = APFCore()
    a = core.compute(v3(0, 0, 0), v3(10, 0, 0), [(1.0, 0.0, 0.0)])
    b = core.compute(v3(0, 0, 0), v3(10, 0, 0), [(1.0, 0.0, 5.0)])
    assert a.f_rep == pytest.approx(b.f_rep)


@pytest.mark.parametrize('goal_z,expected', [
    (1.0, 0.5),
    (0.5, 0.6 * (0.5 - 0.08)),
    (0.05, 0.0),
    (-1.0, -0.5),
    (-0.5, -0.6 * (0.5 - 0.08)),
])
def test_vertical_regulation(goal_z, expected):
    r = APFCore().compute(v3(0, 0, 0), v3(10, 0, goal_z), [])
    assert r.velocity[2] == pytest.approx(expected)


def test_vertical_zero_near_goal_within_relaxed_band():
    r = APFCore().compute(v3(0, 0, 0), v3(0.05, 0, 0.12), [])
    assert r.at_goal
    assert r.velocity[2] == 0.0


def test_yaw_defaults_to_goal_bearing():
    r = APFCore().compute(v3(0, 0, 0), v3(0, 10, 0), [])
    assert r.yaw_cmd == pytest.approx(math.pi / 2)


def test_yaw_locks_to_target():
    r = APFCore().compute(v3(0, 0, 0), v3(10, 0, 0), [], target_pos=v3(0, -5, 0))
    assert r.yaw_cmd == pytest.approx(-math.pi / 2)


def test_yaw_zero_when_target_too_close():
    r = APFCore().compute(v3(0, 0, 0), v3(10, 0, 0), [], target_pos=v3(0.05, 0.05, 0))
    assert r.yaw_cmd == 0.0


def test_forces_are_horizontal_only():
    r = APFCore().compute(v3(0, 0, 0), v3(10, 3, 2), [(1.0, 0.5, 0.0)])
    assert r.f_att[2] == 0.0 and r.f_rep[2] == 0.0 and r.f_total[2] == 0.0


def test_deterministic():
    core = APFCore()
    a = core.compute(v3(0, 0, 0), v3(10, 2, 1), [(2.0, 0.5, 0.0), (3.0, -1.0, 0.0)])
    b = core.compute(v3(0, 0, 0), v3(10, 2, 1), [(2.0, 0.5, 0.0), (3.0, -1.0, 0.0)])
    assert a.velocity == pytest.approx(b.velocity)


def test_zero_net_force_gives_zero_velocity():
    core = APFCore()
    r = core.compute(v3(0, 0, 0), v3(10, 0, 0), [(1.0, 0.0, 0.0)], k_rep_override=0.0)
    assert r.velocity[0] > 0
    core2 = APFCore(APFParams(k_att=0.0))
    r2 = core2.compute(v3(0, 0, 0), v3(10, 0, 0), [])
    assert r2.velocity[0] == 0.0 and r2.velocity[1] == 0.0