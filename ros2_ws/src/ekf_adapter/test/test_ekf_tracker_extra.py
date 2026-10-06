import numpy as np
import pytest

from ekf_adapter.ekf_logic import MeasurementOutcome, TargetTracker, TrackerConfig
from ekf_adapter.target_state_ekf import TargetStateEKF

DT = 1.0 / 30.0
T0 = 1000.0
TIGHT = np.diag([0.10, 0.10, 0.08]) ** 2
LOOSE = np.diag([0.5, 0.5, 0.5]) ** 2
GAPS = [0.05, 0.3, 0.8, 1.5]
OFFSETS = np.round(np.arange(0.1, 2.61, 0.1), 1)


def make_tracker(**overrides):
    return TargetTracker(TargetStateEKF(), TrackerConfig(**overrides))


def settle(tracker, steps=30, covariance=TIGHT):
    for index in range(steps):
        tracker.process_measurement([0.0, 0.0, 1.5], covariance, T0 + index * DT)
    return T0 + (steps - 1) * DT


def accepted_limit(gap, **config):
    limit = 0.0
    for offset in OFFSETS:
        tracker = make_tracker(**config)
        last = settle(tracker, covariance=LOOSE)
        outcome = tracker.process_measurement([float(offset), 0.0, 1.5], LOOSE, last + gap)
        if outcome != MeasurementOutcome.ACCEPTED:
            break
        limit = float(offset)
    return limit


def test_gate_limit_stays_inside_configured_bounds():
    cfg = TrackerConfig()
    for gap in GAPS:
        limit = accepted_limit(gap)
        assert limit >= cfg.gate_min_m - 0.1 - 1e-9
        assert limit <= cfg.gate_max_m + 1e-6


def test_gate_limit_does_not_shrink_with_gap():
    limits = [accepted_limit(gap) for gap in GAPS]
    assert limits == sorted(limits)


def test_fixed_gate_is_respected():
    limit = accepted_limit(0.05, gate_min_m=1.0, gate_max_m=1.0)
    assert 0.9 - 1e-9 <= limit <= 1.0 + 1e-6


def test_order_tolerance_accepts_tiny_lag():
    tracker = make_tracker()
    last = settle(tracker)
    outcome = tracker.process_measurement([0.0, 0.0, 1.5], TIGHT, last - 0.0005)
    assert outcome != MeasurementOutcome.REJECTED_ORDER


def test_order_tolerance_rejects_large_lag():
    tracker = make_tracker()
    last = settle(tracker)
    outcome = tracker.process_measurement([0.0, 0.0, 1.5], TIGHT, last - 0.01)
    assert outcome == MeasurementOutcome.REJECTED_ORDER


def test_reacquire_needs_configured_frame_count():
    tracker = make_tracker(reacquire_frames=3)
    last = settle(tracker)
    outcomes = [
        tracker.process_measurement([5.0 + 0.01 * i, 5.0, 1.5], TIGHT, last + (i + 1) * DT)
        for i in range(3)
    ]
    assert outcomes[:2] == [MeasurementOutcome.REJECTED_GATE] * 2
    assert outcomes[2] == MeasurementOutcome.REACQUIRED


def test_vertical_speed_stays_clamped_while_tracking():
    tracker = TargetTracker(TargetStateEKF(vz_max=0.5), TrackerConfig())
    for index in range(60):
        tracker.process_measurement([0.0, 0.0, 1.5 + 2.0 * index * DT], TIGHT, T0 + index * DT)
    state = tracker.snapshot_at(T0 + 59 * DT).state
    assert abs(state[5]) <= 0.5 + 1e-9


@pytest.mark.parametrize("seed", range(5))
def test_fuzz_snapshot_stays_finite_and_symmetric(seed):
    rng = np.random.default_rng(seed)
    tracker = make_tracker()
    stamp = T0
    for _ in range(300):
        stamp += float(rng.uniform(0.01, 0.2))
        position = rng.normal(0.0, 3.0, size=3)
        position[2] = abs(position[2]) + 0.5
        tracker.process_measurement(position, TIGHT, stamp)
        snapshot = tracker.snapshot_at(stamp + float(rng.uniform(0.0, 3.0)))
        if snapshot is None:
            continue
        assert np.all(np.isfinite(snapshot.state))
        assert np.all(np.isfinite(snapshot.covariance))
        assert np.allclose(snapshot.covariance, snapshot.covariance.T, rtol=1e-6, atol=1e-9)