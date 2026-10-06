import subprocess
import time

import pytest
from sitl_common import DISARMED, ensure_follow, wait_until

KILL_PKG = 'kill_switch'


@pytest.fixture(scope='module')
def kill_node():
    proc = subprocess.Popen(['ros2', 'run', KILL_PKG, 'kill_switch_node'])
    yield proc
    proc.terminate()


def test_kill_switch(probe, kill_node):
    ensure_follow(probe)
    probe.drop_killed_pub()
    probe.rc_raw_on = True
    probe.kill_pwm = 1000
    time.sleep(2.0)
    assert probe.killed in (None, False)
    probe.kill_pwm = 2000
    assert wait_until(lambda: probe.killed is True, 5), 'system/killed khong len true'
    time.sleep(1.0)
    c0 = probe.cnt['fsm/state']
    time.sleep(1.5)
    assert probe.cnt['fsm/state'] == c0, 'FSM van publish sau khi killed'
    assert wait_until(lambda: probe.arming == DISARMED, 10), 'khong force-disarm'