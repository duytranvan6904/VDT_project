import math
import statistics

from sitl_common import (APPROACH, COMPLETE, FOLLOW, LAND, SEARCH,
                         ensure_follow, wait_until)


def sel(log, state, key):
    return [s[key] for s in log if s['state'] == state and math.isfinite(s[key])]


def test_fly_full_sequence(probe):
    ensure_follow(probe)
    probe.land_switch = True
    assert wait_until(lambda: probe.state == COMPLETE, 300), f'state={probe.state}'


def test_gimbal_profile(probe):
    log = probe.log
    search = [s['gimbal'] for s in log if s['state'] == SEARCH and s['z'] >= 2.0
              and math.isfinite(s['gimbal'])]
    if search:
        assert abs(statistics.median(search)) < 5.0
    follow = sel(log, FOLLOW, 'gimbal')
    assert follow and -90.0 <= statistics.median(follow) < -5.0
    land = sel(log, LAND, 'gimbal')
    assert land and min(land) >= -92.0 and statistics.median(land) <= -55.0


def test_velocity_limits(probe):
    flying = [s for s in probe.log if s['state'] >= FOLLOW]
    assert flying
    assert max(math.hypot(s['vx'], s['vy']) for s in flying) <= 2.0 * 1.25
    assert max(abs(s['vz']) for s in flying) <= 1.0 * 1.5


def test_descent_rates(probe):
    appr = sel(probe.log, APPROACH, 'vz')
    assert appr and statistics.median(appr) >= -0.5
    land = [s['vz'] for s in probe.log if s['state'] == LAND and s['z'] > 0.2]
    assert len(land) >= 5
    assert -0.7 <= statistics.median(land) <= -0.1


def test_enu_frame_consistency(probe):
    moving = [s for s in probe.log if s['state'] in (FOLLOW, APPROACH)
              and math.isfinite(s['sx']) and math.hypot(s['sx'], s['sy']) > 0.4]
    assert len(moving) >= 10, 'planner khong tao lenh ngang'
    agree = sum(1 for s in moving if s['sx'] * s['vx'] + s['sy'] * s['vy'] > 0)
    assert agree / len(moving) >= 0.7, f'ENU/NED lech: {agree}/{len(moving)}'