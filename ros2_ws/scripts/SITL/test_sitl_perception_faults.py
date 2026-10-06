import math
import time

from sitl_common import (APPROACH, FOLLOW, SEARCH, ensure_follow, wait_until)


def test_00_follow(probe):
    ensure_follow(probe)


def test_01_ekf_accuracy(probe):
    time.sleep(5.0)
    errs = []
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        errs.append(probe.pad_error())
        time.sleep(0.05)
    assert sum(errs) / len(errs) < 0.3, f'sai so EKF TB {sum(errs) / len(errs):.2f} m'
    assert max(errs) < 0.6


def test_02_outlier_rejected(probe):
    probe.outlier = True
    errs = []
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        errs.append(probe.pad_error())
        time.sleep(0.05)
    probe.outlier = False
    assert max(errs) < 1.0, f'EKF nhay {max(errs):.2f} m khi gap outlier'
    assert probe.state == FOLLOW


def test_03_tracking_expired(probe):
    probe.blackout = True
    assert wait_until(lambda: probe.mode == 'EXPIRED', 8), f'mode={probe.mode}'
    assert wait_until(lambda: math.isnan(probe.snap.delta_h), 3)
    assert math.isnan(probe.snap.d_horiz) and math.isnan(probe.snap.align_error)
    assert probe.flags.ekf_timeout
    assert abs(probe.sp.vx) < 1e-3 and abs(probe.sp.vy) < 1e-3 and abs(probe.sp.vz) < 1e-3
    probe.blackout = False
    assert wait_until(lambda: probe.state == FOLLOW, 90)
    assert wait_until(lambda: probe.mode in ('TRACKING', 'PREDICTING'), 10)


def test_04_land_switch_cancel(probe):
    probe.land_switch = True
    assert wait_until(lambda: probe.state == APPROACH, 15)
    time.sleep(1.0)
    probe.land_switch = False
    assert wait_until(lambda: probe.state == FOLLOW, 5)


def test_05_land_inhibit_after_marker_loss(probe):
    probe.land_switch = True
    assert wait_until(lambda: probe.state == APPROACH, 15)
    time.sleep(1.0)
    probe.blackout = True
    assert wait_until(lambda: probe.state == FOLLOW, 6)
    probe.blackout = False
    assert wait_until(lambda: probe.snap.marker_detected, 10)
    assert not wait_until(lambda: probe.state == APPROACH, 5), 'land_inhibit khong chan APPROACH'
    probe.land_switch = False
    time.sleep(1.0)
    probe.land_switch = True
    assert wait_until(lambda: probe.state == APPROACH, 10), 'khong re-arm duoc APPROACH'
    probe.land_switch = False
    assert wait_until(lambda: probe.state in (FOLLOW, SEARCH), 5)