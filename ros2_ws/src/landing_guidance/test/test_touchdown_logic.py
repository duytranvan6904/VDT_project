import math

from landing_guidance.touchdown_logic import TouchdownDetector, TouchdownParams

DT = 0.05


def mk(det, t, **kw):
    base = dict(actual_vz=0.0, commanded_vz=-0.3, current_alt_z=0.1,
                optical_z=0.1, px4_landed=False, actual_vxy=0.0)
    base.update(kw)
    return det.step(current_time=t, **base)


def descend(det, alt0=1.0, secs=3.0):
    t, res = 0.0, None
    for i in range(int(secs / DT)):
        alt = alt0 - 0.3 * i * DT
        res = mk(det, t, actual_vz=-0.3, current_alt_z=alt, optical_z=alt)
        t += DT
    return res, t


def hold(det, t, steps, **kw):
    res = None
    for _ in range(steps):
        res = mk(det, t, **kw)
        t += DT
    return res, t


def test_touchdown_xac_nhan_dung_luc():
    det = TouchdownDetector()
    _, t = descend(det)
    res, t = hold(det, t, 5)
    assert not res.touchdown_confirmed
    res, t = hold(det, t, 8)
    assert res.touchdown_confirmed


def test_dung_0p2s_roi_chuyen_dong_thi_reset():
    det = TouchdownDetector()
    _, t = descend(det)
    # Hold 3 steps (0.15s < debounce 0.18s), not confirmed yet
    _, t = hold(det, t, 3)
    for i in range(4):
        res = mk(det, t, actual_vz=-0.2, current_alt_z=0.1 - 0.01 * (i + 1),
                 optical_z=0.1)
        t += DT
    # Hold 2 steps (0.10s < debounce 0.18s), still not confirmed
    res, t = hold(det, t, 2, current_alt_z=0.06, optical_z=0.1)
    assert not res.touchdown_confirmed


def test_px4_landed_o_do_cao_lon_khong_duoc_chot():
    det = TouchdownDetector()
    _, t = descend(det, alt0=2.0, secs=1.5)
    res, t = hold(det, t, 40, current_alt_z=1.55, optical_z=1.55, px4_landed=True)
    assert not res.touchdown_confirmed


def test_px4_landed_mat_marker_o_thap_van_chot():
    det = TouchdownDetector()
    _, t = descend(det)
    res, t = hold(det, t, 20, optical_z=None, px4_landed=True)
    assert res.touchdown_confirmed


def test_con_troi_ngang_khong_chot():
    det = TouchdownDetector()
    _, t = descend(det)
    res, t = hold(det, t, 40, actual_vxy=0.3)
    assert not res.touchdown_confirmed


def test_nan_khong_chot():
    det = TouchdownDetector()
    _, t = descend(det)
    res, t = hold(det, t, 40, actual_vz=math.nan, px4_landed=True)
    assert not res.touchdown_confirmed
    assert res.diagnostic_reason == 'INVALID_INPUT'


def test_ngoai_do_cao_va_dung_van_khong_chot():
    det = TouchdownDetector()
    _, t = descend(det, alt0=2.0, secs=1.5)
    res, t = hold(det, t, 40, current_alt_z=1.55, optical_z=1.55)
    assert not res.touchdown_confirmed


def _descend_cam(det, alt0, secs, off):
    t = 0.0
    for i in range(int(secs / DT)):
        alt = alt0 - 0.3 * i * DT
        mk(det, t, actual_vz=-0.3, current_alt_z=alt, optical_z=alt + off)
        t += DT
    return t


def test_camera_offset_chot_dung_do_cao_that():
    det = TouchdownDetector(TouchdownParams(camera_offset_m=0.80))
    t = _descend_cam(det, 1.0, 3.0, 0.80)
    res, t = hold(det, t, 20, current_alt_z=0.1, optical_z=0.90)
    assert res.touchdown_confirmed


def test_camera_offset_o_cao_khong_chot():
    det = TouchdownDetector(TouchdownParams(camera_offset_m=0.80))
    t = _descend_cam(det, 2.0, 1.5, 0.80)
    res, t = hold(det, t, 40, current_alt_z=1.55, optical_z=2.35)
    assert not res.touchdown_confirmed
