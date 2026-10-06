from sitl_common import SEARCH, ensure_follow, pkill, wait_until


def test_snapshot_loss_hover(probe):
    ensure_follow(probe)
    z0 = probe.z()
    c0 = probe.cnt['fsm/state']
    pkill('input_cache_node')
    assert wait_until(lambda: probe.state == SEARCH, 5), 'FSM khong ve SEARCH khi mat snapshot'
    assert wait_until(lambda: probe.hspeed() < 0.3 and probe.vspeed() < 0.3, 10), 'UAV khong hover'
    assert probe.z() >= z0 - 0.7
    assert probe.cnt['fsm/state'] > c0