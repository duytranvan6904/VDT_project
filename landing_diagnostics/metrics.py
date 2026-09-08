from dataclasses import dataclass

import numpy as np
import pandas as pd

FSM_LAND = 3
REQUIRED_COLUMNS = {
    'timestamp', 'fsm_state', 'pos_x', 'pos_y', 'pos_z',
    'vel_x', 'vel_y', 'vel_z', 'sp_vx', 'sp_vy', 'sp_vz'
}


@dataclass
class ErrorStats:
    mean_err_xy: float
    max_err_xy: float
    std_err_xy: float
    mean_vx: float
    mean_vy: float
    oscillation_freq_hz: float
    altitude_error_corr: float
    horizontal_drift_m: float


def load_flight_log(csv_path):
    df = pd.read_csv(csv_path)
    missing = REQUIRED_COLUMNS - set(df.columns)
    if missing:
        raise ValueError(f'Thieu cot bat buoc: {sorted(missing)}')
    numeric_columns = sorted(REQUIRED_COLUMNS - {'fsm_state'})
    for column in numeric_columns:
        df[column] = pd.to_numeric(df[column], errors='coerce')
    df['fsm_state'] = pd.to_numeric(df['fsm_state'], errors='coerce')
    df = df[np.isfinite(df['timestamp'])]
    return df.sort_values('timestamp').drop_duplicates('timestamp').reset_index(drop=True)


def filter_land_phase(df):
    return df[df['fsm_state'] == FSM_LAND].reset_index(drop=True)


def compute_tracking_error(land_df):
    err_vx = land_df['vel_x'] - land_df['sp_vx']
    err_vy = land_df['vel_y'] - land_df['sp_vy']
    err_xy = np.hypot(err_vx, err_vy)
    return err_vx.to_numpy(), err_vy.to_numpy(), err_xy.to_numpy()


def dominant_frequency(signal, dt):
    if len(signal) < 4 or not np.isfinite(dt) or dt <= 0 or not np.all(np.isfinite(signal)):
        return 0.0
    spectrum = np.abs(np.fft.rfft(signal - np.mean(signal)))
    freqs = np.fft.rfftfreq(len(signal), d=dt)
    if len(spectrum) <= 1:
        return 0.0
    dominant_idx = np.argmax(spectrum[1:]) + 1
    return float(freqs[dominant_idx])


def compute_horizontal_drift(land_df):
    x0, y0 = land_df['pos_x'].iloc[0], land_df['pos_y'].iloc[0]
    x1, y1 = land_df['pos_x'].iloc[-1], land_df['pos_y'].iloc[-1]
    return float(np.hypot(x1 - x0, y1 - y0))


def compute_altitude_error_corr(land_df, err_xy):
    altitude = -land_df['pos_z'].to_numpy()
    if len(altitude) < 3 or not np.all(np.isfinite(altitude)) or not np.all(np.isfinite(err_xy)):
        return 0.0
    if np.std(altitude) == 0 or np.std(err_xy) == 0:
        return 0.0
    correlation = np.corrcoef(altitude, err_xy)[0, 1]
    return float(correlation) if np.isfinite(correlation) else 0.0


def compute_stats(csv_path):
    df = load_flight_log(csv_path)
    land_df = filter_land_phase(df)
    land_df = land_df[np.isfinite(land_df[list(REQUIRED_COLUMNS - {'fsm_state'})]).all(axis=1)]
    if len(land_df) < 5:
        raise ValueError('Khong du du lieu trong phase LAND de phan tich')

    err_vx, err_vy, err_xy = compute_tracking_error(land_df)
    timestamp_diff = np.diff(land_df['timestamp'].to_numpy())
    if len(timestamp_diff) == 0 or not np.all(np.isfinite(timestamp_diff)) or np.any(timestamp_diff <= 0):
        raise ValueError('Timestamp LAND khong tang dan hoac khong hop le')
    dt = float(np.median(timestamp_diff) / 1e6)
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError('Khoang thoi gian LAND khong hop le')

    return ErrorStats(
        mean_err_xy=float(np.mean(err_xy)),
        max_err_xy=float(np.max(err_xy)),
        std_err_xy=float(np.std(err_xy)),
        mean_vx=float(np.mean(err_vx)),
        mean_vy=float(np.mean(err_vy)),
        oscillation_freq_hz=dominant_frequency(err_xy, dt),
        altitude_error_corr=compute_altitude_error_corr(land_df, err_xy),
        horizontal_drift_m=compute_horizontal_drift(land_df),
    )