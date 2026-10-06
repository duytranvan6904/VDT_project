import time

from sitl_common import pgrep, pkill, wait_until


def test_xrce_agent_restart(probe):
    assert wait_until(lambda: probe.snap is not None and probe.snap.valid, 60)
    probe.reset_stats()
    pkill('MicroXRCEAgent')
    assert wait_until(lambda: not probe.snap.valid, 5), 'valid van true khi mat odom'
    assert wait_until(lambda: pgrep('MicroXRCEAgent'), 30), 'xrce_bridge khong khoi dong lai Agent'
    assert wait_until(lambda: probe.snap.valid, 60), 'khong khoi phuc sau reconnect'
    time.sleep(1.0)
    assert probe.max_gap < 0.2, f'snapshot bi dung {probe.max_gap:.3f}s trong luc mat Agent'