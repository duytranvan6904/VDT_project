import numpy as np
import pytest

from ekf_adapter.ekf_logic import MeasurementOutcome, TargetTracker
from ekf_adapter.target_state_ekf import TargetStateEKF
from ekf_adapter.target_state_simulation import SimulationConfig, simulate_measurements
from ekf_adapter.tracking_policy import classify_tracking_mode

BASE = 1000.0
COVARIANCE = np.diag([0.10, 0.10, 0.08]) ** 2
TRAJECTORIES = ["straight", "circle", "figure8", "zigzag", "stop_go"]


def replay(trajectory, config):
    data = simulate_measurements(trajectory, config)
    tracker = TargetTracker(TargetStateEKF())
    outcomes = [None] * len(data.timestamps)
    estimates = np.full((len(data.timestamps), 3), np.nan)
    for index, timestamp in enumerate(data.timestamps):
        stamp = BASE + float(timestamp)
        if data.detected[index]:
            outcomes[index] = tracker.process_measurement(
                data.measurements[index], COVARIANCE, stamp
            )
        snapshot = tracker.snapshot_at(stamp)
        if snapshot is not None:
            estimates[index] = snapshot.state[:3]
    return data, outcomes, estimates


@pytest.mark.parametrize("seed", [1, 2, 3])
@pytest.mark.parametrize("trajectory", TRAJECTORIES)
def test_filter_beats_raw_and_stays_finite(trajectory, seed):
    config = SimulationConfig(dropout_intervals_s=((6.0, 7.0),), outlier_probability=0.02, seed=seed)
    data, _, estimates = replay(trajectory, config)
    truth = data.truth_state[:, :3]
    valid = ~np.isnan(estimates[:, 0]) & data.detected
    raw = np.sqrt(np.mean(np.sum((data.measurements[valid] - truth[valid]) ** 2, axis=1)))
    filtered = np.sqrt(np.mean(np.sum((estimates[valid] - truth[valid]) ** 2, axis=1)))
    first = int(np.flatnonzero(~np.isnan(estimates[:, 0]))[0])
    assert filtered < raw
    assert np.all(np.isfinite(estimates[first:]))


@pytest.mark.parametrize("length", [0.5, 1.0, 2.0, 3.0])
@pytest.mark.parametrize("trajectory", ["straight", "circle"])
def test_recovery_after_dropout(trajectory, length):
    config = SimulationConfig(dropout_intervals_s=((5.0, 5.0 + length),), duration_s=15.0, seed=9)
    data, _, estimates = replay(trajectory, config)
    index = int(np.searchsorted(data.timestamps, 5.0 + length + 0.5))
    error = np.linalg.norm(estimates[index] - data.truth_state[index, :3])
    assert error < 0.5


@pytest.mark.parametrize("probability", [0.0, 0.02, 0.05, 0.1])
def test_outliers_mostly_rejected(probability):
    config = SimulationConfig(outlier_probability=probability, seed=5, duration_s=30.0)
    data, outcomes, _ = replay("straight", config)
    total = int(np.count_nonzero(data.outlier))
    accepted = sum(
        1
        for index in np.flatnonzero(data.outlier)
        if outcomes[index] in (MeasurementOutcome.ACCEPTED, MeasurementOutcome.INITIALIZED)
    )
    assert accepted <= max(1, total // 2)


def window(names, low, high):
    return [name for time, name in names.items() if low <= time < high]


def test_mode_timeline_follows_dropout():
    config = SimulationConfig(dropout_intervals_s=((5.0, 8.0),), duration_s=15.0, seed=4)
    data = simulate_measurements("straight", config)
    tracker = TargetTracker(TargetStateEKF())
    names = {}
    for index, timestamp in enumerate(data.timestamps):
        stamp = BASE + float(timestamp)
        if data.detected[index]:
            tracker.process_measurement(data.measurements[index], COVARIANCE, stamp)
        mode = classify_tracking_mode(tracker.detected(stamp), tracker.age(stamp), "FOLLOW")
        names[float(timestamp)] = mode.name

    before = window(names, 2.0, 4.9)
    assert before.count("TRACKING") >= 0.9 * len(before)
    assert "TRACKING" not in window(names, 5.5, 6.0)
    assert set(window(names, 7.2, 7.9)) <= {"EXPIRED", "LOST"}
    after = window(names, 9.0, 14.0)
    assert after.count("TRACKING") >= 0.9 * len(after)