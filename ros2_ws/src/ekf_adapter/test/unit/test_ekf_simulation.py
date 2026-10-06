import numpy as np
import pytest

from ekf_adapter.ekf_logic import MeasurementOutcome, TargetTracker
from ekf_adapter.target_state_ekf import TargetStateEKF
from ekf_adapter.target_state_simulation import iter_default_scenarios, simulate_measurements

COVARIANCE = np.diag((0.10, 0.10, 0.08)) ** 2
SCENARIOS = list(iter_default_scenarios())


@pytest.fixture(params=SCENARIOS, ids=[name for name, _ in SCENARIOS])
def replay(request):
    name, config = request.param
    data = simulate_measurements(name, config)
    tracker = TargetTracker(TargetStateEKF())
    count = len(data.timestamps)
    filtered = np.full((count, 3), np.nan)
    accepted = np.zeros(count, dtype=bool)
    for index, stamp in enumerate(data.timestamps):
        if data.detected[index]:
            outcome = tracker.process_measurement(data.measurements[index], COVARIANCE, float(stamp))
            accepted[index] = outcome in (MeasurementOutcome.INITIALIZED, MeasurementOutcome.ACCEPTED)
        snapshot = tracker.snapshot_at(float(stamp))
        if snapshot is not None:
            filtered[index] = snapshot.state[:3]
    return config, data, filtered, accepted


def test_filter_error_is_below_raw_measurement_error(replay):
    _, data, filtered, _ = replay
    mask = data.detected & (data.timestamps > 1.0)
    truth = data.truth_state[:, :3]
    raw = np.sqrt(np.mean(np.sum((data.measurements[mask] - truth[mask]) ** 2, axis=1)))
    est = np.sqrt(np.mean(np.sum((filtered[mask] - truth[mask]) ** 2, axis=1)))
    assert est < raw


def test_most_outliers_are_rejected(replay):
    _, data, _, accepted = replay
    assert np.any(data.outlier)
    assert np.mean(~accepted[data.outlier]) >= 0.5


def test_track_recovers_after_dropout(replay):
    config, data, filtered, _ = replay
    end = max(stop for _, stop in config.dropout_intervals_s)
    mask = data.timestamps >= end + 2.0
    error = np.linalg.norm(filtered[mask] - data.truth_state[mask, :3], axis=1)
    assert np.max(error) < 0.35


def test_error_during_dropout_stays_bounded(replay):
    config, data, filtered, _ = replay
    start, stop = config.dropout_intervals_s[0]
    mask = (data.timestamps >= start) & (data.timestamps < stop)
    error = np.linalg.norm(filtered[mask] - data.truth_state[mask, :3], axis=1)
    assert np.max(error) < 1.0