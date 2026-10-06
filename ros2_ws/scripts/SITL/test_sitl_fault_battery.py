from sitl_common import COMPLETE, LAND, ensure_follow, wait_until


def test_battery_critical_force_land(probe):
    ensure_follow(probe)
    assert not probe.force_land
    probe.low_batt = True
    assert wait_until(lambda: probe.state == LAND, 15), f'state={probe.state}'
    probe.low_batt = False
    assert wait_until(lambda: probe.state == COMPLETE, 120)