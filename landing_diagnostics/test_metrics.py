import csv
import math

import numpy as np
import pytest

from landing_diagnostics.classify import DriftCause, classify_drift
from landing_diagnostics.metrics import compute_stats, dominant_frequency, load_flight_log
from landing_diagnostics.tuning import propose_param_change

COLUMNS = [
    'timestamp', 'fsm_state', 'pos_x', 'pos_y', 'pos_z',
    'vel_x', 'vel_y', 'vel_z', 'sp_vx', 'sp_vy', 'sp_vz'
]


def write_csv(path, rows, columns=COLUMNS):
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(rows)


def land_rows(count=6, timestamps=None):
    timestamps = timestamps or [index * 50000 for index in range(count)]
    return [
        [timestamp, 3, index * 0.1, 0.0, -1.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
        for index, timestamp in enumerate(timestamps)
    ]


def test_load_flight_log_rejects_missing_columns(tmp_path):
    path = tmp_path / 'missing.csv'
    write_csv(path, [[0, 3]], columns=['timestamp', 'fsm_state'])
    with pytest.raises(ValueError, match='Thieu cot bat buoc'):
        load_flight_log(path)


def test_compute_stats_rejects_short_land_data(tmp_path):
    path = tmp_path / 'short.csv'
    write_csv(path, land_rows(4))
    with pytest.raises(ValueError, match='Khong du du lieu'):
        compute_stats(path)


def test_compute_stats_deduplicates_timestamps(tmp_path):
    path = tmp_path / 'bad_time.csv'
    write_csv(path, land_rows(6, [0, 50000, 50000, 100000, 150000, 200000]))
    stats = compute_stats(path)
    assert math.isfinite(stats.mean_err_xy)


def test_compute_stats_filters_non_finite_rows(tmp_path):
    path = tmp_path / 'nan.csv'
    rows = land_rows(6)
    rows[2][5] = math.nan
    write_csv(path, rows)
    stats = compute_stats(path)
    assert math.isfinite(stats.mean_err_xy)
    assert stats.horizontal_drift_m > 0.0


def test_dominant_frequency_rejects_invalid_signal():
    assert dominant_frequency(np.array([math.nan, 1.0, 2.0, 3.0]), 0.05) == 0.0
    assert dominant_frequency(np.ones(4), 0.0) == 0.0


def test_classification_and_tuning():
    stats = type('Stats', (), {
        'oscillation_freq_hz': 1.0,
        'max_err_xy': 2.0,
        'mean_err_xy': 0.5,
        'mean_vx': 0.0,
        'mean_vy': 0.0,
        'std_err_xy': 0.1,
        'altitude_error_corr': 0.0,
    })()
    assert classify_drift(stats) == DriftCause.UNDERDAMPED
    suggestions = propose_param_change(
        DriftCause.UNDERDAMPED,
        {'MPC_XY_VEL_P_ACC': 1.0, 'MPC_XY_VEL_D_ACC': 1.0},
    )
    assert suggestions == {'MPC_XY_VEL_P_ACC': 0.85, 'MPC_XY_VEL_D_ACC': 1.2}
