from land_arbiter import AutoLand, Debouncer


def run_deb(d, seq, dt=0.02):
    t = 0.0
    out = []
    for raw, n in seq:
        for _ in range(n):
            out.append(d.update(raw, t))
            t += dt
    return out


def run_auto(al, seq, dt=0.1, t0=0.0):
    t = t0
    out = []
    for phase, lock, secs in seq:
        for _ in range(int(round(secs / dt))):
            out.append(al.update(t, phase, lock))
            t += dt
    return out, t


def test_debouncer_short_pulse_does_not_trigger():
    assert not any(run_deb(Debouncer(0.10), [(True, 4), (False, 10)]))


def test_debouncer_triggers_when_held_long_enough():
    out = run_deb(Debouncer(0.10), [(True, 12)])
    assert not out[0] and out[-1]


def test_debouncer_alternating_input_restarts_timer():
    out = run_deb(Debouncer(0.10), [(True, 4), (False, 1), (True, 4), (False, 1), (True, 4)])
    assert not any(out)


def test_debouncer_releases_immediately():
    out = run_deb(Debouncer(0.10), [(True, 12), (False, 1)])
    assert out[11] and not out[12]


def test_auto_land_disabled_by_default_never_triggers():
    out, _ = run_auto(AutoLand(0.0, 3.0), [('FOLLOW', True, 600.0)])
    assert not any(out)


def test_auto_land_triggers_after_enough_time_and_lock():
    out, _ = run_auto(AutoLand(30.0, 3.0), [('FOLLOW', True, 29.0)])
    assert not any(out)
    out, _ = run_auto(AutoLand(30.0, 3.0), [('FOLLOW', True, 33.0)])
    assert out[-1]


def test_auto_land_lock_loss_delays_trigger():
    al = AutoLand(10.0, 3.0)
    out, t = run_auto(al, [('FOLLOW', True, 9.0), ('FOLLOW', False, 1.0), ('FOLLOW', True, 2.0)])
    assert not any(out)
    out, t = run_auto(al, [('FOLLOW', True, 2.0)], t0=t)
    assert out[-1]


def test_auto_land_leaving_follow_restarts_timer():
    al = AutoLand(10.0, 3.0)
    out, t = run_auto(al, [('FOLLOW', True, 9.0), ('SEARCH', True, 1.0), ('FOLLOW', True, 5.0)])
    assert not any(out)


def test_auto_land_latched_until_idle():
    al = AutoLand(5.0, 3.0)
    out, t = run_auto(al, [('FOLLOW', True, 6.0), ('APPROACH', False, 3.0), ('LAND', False, 3.0)])
    assert out[-1]
    out, t = run_auto(al, [('IDLE', False, 1.0)], t0=t)
    assert not out[-1]


def test_auto_land_cancel_restarts_timer():
    al = AutoLand(5.0, 3.0)
    out, t = run_auto(al, [('FOLLOW', True, 6.0)])
    assert out[-1]
    al.cancel(t)
    out, t = run_auto(al, [('FOLLOW', True, 3.0)], t0=t)
    assert not any(out)
    out, t = run_auto(al, [('FOLLOW', True, 4.0)], t0=t)
    assert out[-1]
