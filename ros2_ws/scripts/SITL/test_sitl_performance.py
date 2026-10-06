import time

import psutil
from sitl_common import FOLLOW, ensure_follow

WINDOW_S = 60


def cpu_temp():
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as f:
            return int(f.read()) / 1000.0
    except OSError:
        return None


def test_load_over_follow(probe):
    ensure_follow(probe)
    time.sleep(3.0)
    probe.reset_stats()
    n_hist = len(probe.history)
    n0, t0 = probe.snap_count, time.monotonic()
    cpu, mem, temp = [], [], []
    psutil.cpu_percent(None)
    for _ in range(WINDOW_S):
        time.sleep(1.0)
        cpu.append(psutil.cpu_percent(None))
        mem.append(psutil.virtual_memory().percent)
        t = cpu_temp()
        if t is not None:
            temp.append(t)
    hz = (probe.snap_count - n0) / (time.monotonic() - t0)
    assert 18.0 <= hz <= 22.0, f'snapshot {hz:.1f} Hz'
    assert probe.max_gap < 0.2, f'gap snapshot {probe.max_gap:.3f}s'
    assert probe.max_gap_sp < 0.3, f'gap planner setpoint {probe.max_gap_sp:.3f}s'
    assert len(probe.history) == n_hist and probe.state == FOLLOW, 'FSM dao dong state'
    assert sum(cpu) / len(cpu) < 80.0, f'CPU TB {sum(cpu) / len(cpu):.0f}%'
    assert max(mem) < 85.0, f'RAM {max(mem):.0f}%'
    if temp:
        assert max(temp) < 80.0, f'nhiet do {max(temp):.0f}C'