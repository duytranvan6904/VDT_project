"""Synthetic trajectories and replay helpers for target-state estimator tests."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import numpy as np

from .target_state_ekf import TargetStateEKF


MAX_SIMULATION_SPEED_MPS = 2.0
MAX_OUTLIER_DISTANCE_M = 0.95


def _smooth_step(value: np.ndarray) -> np.ndarray:
    """Cubic smoothstep, clipped to [0, 1], for finite acceleration changes."""
    value = np.clip(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def _smooth_stop_go_velocity(
    timestamps: np.ndarray,
    speed_a: float,
    speed_b: float,
    final_speed: float,
    switch_a: float,
    switch_b: float,
) -> np.ndarray:
    """Generate a continuous stop-go speed profile with one-second ramps."""
    t = timestamps
    velocity = np.zeros_like(t)
    # Accelerate from rest, cruise, then smoothly stop.
    velocity += speed_a * _smooth_step((t - 2.0) / 1.0)
    velocity -= speed_a * _smooth_step((t - switch_a) / 1.0)
    # Reverse after the stop, then smoothly approach the final cruise speed.
    velocity += speed_b * _smooth_step((t - switch_b) / 1.0)
    velocity += (final_speed - speed_b) * _smooth_step((t - (switch_b + 2.0)) / 1.0)
    return velocity


@dataclass(frozen=True)
class SimulationConfig:
    duration_s: float = 20.0
    dt_s: float = 1.0 / 30.0
    measurement_std_m: Sequence[float] = (0.10, 0.10, 0.08)
    dropout_intervals_s: Sequence[tuple[float, float]] = ()
    outlier_probability: float = 0.0
    outlier_std_m: float = 3.0
    outlier_max_distance_m: float = MAX_OUTLIER_DISTANCE_M
    seed: Optional[int] = 7
    randomize_trajectory: bool = False


@dataclass(frozen=True)
class SimulationData:
    timestamps: np.ndarray
    truth_state: np.ndarray
    measurements: np.ndarray
    detected: np.ndarray
    outlier: np.ndarray


@dataclass(frozen=True)
class EstimatorReplay:
    states: np.ndarray
    covariance_diagonal: np.ndarray
    accepted: np.ndarray
    nis: np.ndarray
    initialized: np.ndarray


def generate_truth(trajectory: str, duration_s: float = 20.0, dt_s: float = 1.0 / 30.0) -> tuple[np.ndarray, np.ndarray]:
    """Create a deterministic 6-D ground-truth trajectory.

    Supported trajectories deliberately cover both the CV model's strengths and
    its limitations: ``straight``, ``circle``, ``figure8``, ``zigzag`` and
    ``stop_go``.
    """
    if duration_s <= 0.0 or dt_s <= 0.0:
        raise ValueError("duration_s and dt_s must be positive")
    timestamps = np.arange(0.0, duration_s + 0.5 * dt_s, dt_s)
    t = timestamps

    if trajectory == "straight":
        position = np.column_stack((0.7 * t, 0.2 * t, np.full_like(t, 1.5)))
    elif trajectory == "circle":
        radius, omega = 2.0, 0.60
        position = np.column_stack((radius * np.cos(omega * t), radius * np.sin(omega * t), 1.5 + 0.10 * np.sin(0.5 * omega * t)))
    elif trajectory == "figure8":
        omega = 0.55
        position = np.column_stack((2.0 * np.sin(omega * t), 1.2 * np.sin(2.0 * omega * t), 1.5 + 0.15 * np.cos(omega * t)))
    elif trajectory == "zigzag":
        omega = 0.45
        triangle = 2.0 / np.pi * np.arcsin(np.sin(omega * t))
        position = np.column_stack((0.65 * t, 2.0 * triangle, 1.5 + 0.10 * np.sin(omega * t)))
    elif trajectory == "stop_go":
        velocity_x = _smooth_stop_go_velocity(t, 0.8, -0.6, 0.5, 6.0, 8.0)
        x = np.cumsum(velocity_x) * dt_s
        position = np.column_stack((x, 0.6 * np.sin(0.35 * t), 1.5 + 0.05 * np.sin(0.6 * t)))
    else:
        raise ValueError(f"unknown trajectory '{trajectory}'")

    velocity = np.gradient(position, dt_s, axis=0, edge_order=2)
    horizontal_speed = np.linalg.norm(velocity[:, :2], axis=1)
    if float(np.max(horizontal_speed)) > MAX_SIMULATION_SPEED_MPS:
        raise RuntimeError(
            f"trajectory '{trajectory}' exceeds the simulation speed limit "
            f"of {MAX_SIMULATION_SPEED_MPS:.1f} m/s"
        )
    return timestamps, np.column_stack((position, velocity))


def generate_random_truth(
    trajectory: str,
    rng: np.random.Generator,
    duration_s: float = 20.0,
    dt_s: float = 1.0 / 30.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate a new parameterized trajectory while enforcing the 2 m/s cap."""
    if duration_s <= 0.0 or dt_s <= 0.0:
        raise ValueError("duration_s and dt_s must be positive")
    timestamps = np.arange(0.0, duration_s + 0.5 * dt_s, dt_s)
    t = timestamps
    phase = float(rng.uniform(-np.pi, np.pi))
    z0 = float(rng.uniform(1.2, 1.8))

    if trajectory == "straight":
        speed = float(rng.uniform(0.4, 1.8))
        heading = float(rng.uniform(-np.pi, np.pi))
        position = np.column_stack((speed * np.cos(heading) * t, speed * np.sin(heading) * t, np.full_like(t, z0)))
    elif trajectory == "circle":
        radius = float(rng.uniform(2.5, 5.0))
        speed = float(rng.uniform(0.5, 1.8))
        omega = speed / radius
        position = np.column_stack((radius * np.cos(omega * t + phase), radius * np.sin(omega * t + phase), z0 + 0.08 * np.sin(0.5 * omega * t)))
    elif trajectory == "figure8":
        amp_x = float(rng.uniform(1.5, 3.0))
        amp_y = float(rng.uniform(0.8, 1.8))
        omega = float(rng.uniform(0.20, 0.45))
        position = np.column_stack((amp_x * np.sin(omega * t + phase), amp_y * np.sin(2.0 * omega * t + phase), z0 + 0.10 * np.cos(omega * t)))
    elif trajectory == "zigzag":
        speed_x = float(rng.uniform(0.35, 1.1))
        amp_y = float(rng.uniform(1.0, 2.0))
        omega = float(rng.uniform(0.20, 0.45))
        triangle = 2.0 / np.pi * np.arcsin(np.sin(omega * t + phase))
        position = np.column_stack((speed_x * t, amp_y * triangle, z0 + 0.08 * np.sin(omega * t)))
    elif trajectory == "stop_go":
        speed_a = float(rng.uniform(0.4, 1.3))
        speed_b = -float(rng.uniform(0.3, 1.1))
        switch_a = float(rng.uniform(3.0, 5.0))
        switch_b = float(rng.uniform(7.0, 10.0))
        final_speed = float(rng.uniform(0.2, 0.9))
        velocity_x = _smooth_stop_go_velocity(t, speed_a, speed_b, final_speed, switch_a, switch_b)
        x = np.cumsum(velocity_x) * dt_s
        position = np.column_stack((x, 0.6 * np.sin(0.25 * t + phase), z0 + 0.05 * np.sin(0.5 * t)))
    else:
        raise ValueError(f"unknown trajectory '{trajectory}'")

    velocity = np.gradient(position, dt_s, axis=0, edge_order=2)
    horizontal_speed = np.linalg.norm(velocity[:, :2], axis=1)
    if float(np.max(horizontal_speed)) > MAX_SIMULATION_SPEED_MPS:
        # Scale the time base for the rare high-curvature draw and recompute
        # derivatives, preserving shape while enforcing the physical limit.
        position[:, :2] *= MAX_SIMULATION_SPEED_MPS / float(np.max(horizontal_speed))
        velocity = np.gradient(position, dt_s, axis=0, edge_order=2)
    return timestamps, np.column_stack((position, velocity))


def simulate_measurements(trajectory: str, config: SimulationConfig) -> SimulationData:
    """Inject repeatable Gaussian noise, dropout and outliers into truth."""
    if config.randomize_trajectory:
        trajectory_rng = np.random.default_rng(config.seed)
        timestamps, truth_state = generate_random_truth(trajectory, trajectory_rng, config.duration_s, config.dt_s)
    else:
        timestamps, truth_state = generate_truth(trajectory, config.duration_s, config.dt_s)
    noise_std = np.asarray(config.measurement_std_m, dtype=float)
    if noise_std.shape != (3,) or np.any(noise_std <= 0.0):
        raise ValueError("measurement_std_m must contain three positive values")
    if not 0.0 <= config.outlier_probability <= 1.0:
        raise ValueError("outlier_probability must be within [0, 1]")
    if not 0.0 < config.outlier_max_distance_m < 1.0:
        raise ValueError("outlier_max_distance_m must be within (0, 1) metres")

    rng = np.random.default_rng(config.seed)
    measurements = truth_state[:, :3] + rng.normal(0.0, noise_std, size=(len(timestamps), 3))
    detected = np.ones(len(timestamps), dtype=bool)
    for start, end in config.dropout_intervals_s:
        if end <= start:
            raise ValueError("every dropout interval must have end > start")
        detected &= ~((timestamps >= start) & (timestamps < end))

    outlier = (rng.random(len(timestamps)) < config.outlier_probability) & detected
    if np.any(outlier):
        offsets = rng.normal(0.0, config.outlier_std_m, size=(np.count_nonzero(outlier), 3))
        # Preserve a tunable direction/scale while hard-bounding every
        # synthetic outlier to keep plots and scenario geometry readable.
        offset_norm = np.linalg.norm(offsets, axis=1, keepdims=True)
        safe_norm = np.maximum(offset_norm, np.finfo(float).eps)
        offsets *= np.minimum(1.0, config.outlier_max_distance_m / safe_norm)
        measurements[outlier] += offsets
        # The displayed point includes the base sensor noise as well. Clamp
        # the final measurement displacement too, so the visible outlier is
        # guaranteed to remain within the configured radius.
        final_offsets = measurements[outlier] - truth_state[outlier, :3]
        final_norm = np.linalg.norm(final_offsets, axis=1, keepdims=True)
        measurements[outlier] = truth_state[outlier, :3] + final_offsets * np.minimum(
            1.0, config.outlier_max_distance_m / np.maximum(final_norm, np.finfo(float).eps)
        )
    measurements[~detected] = np.nan
    return SimulationData(timestamps, truth_state, measurements, detected, outlier)


def replay_estimator(
    data: SimulationData,
    estimator: Optional[TargetStateEKF] = None,
    measurement_covariance: Optional[np.ndarray] = None,
) -> EstimatorReplay:
    """Replay a measurement sequence through the estimator, including lost frames."""
    if estimator is None:
        estimator = TargetStateEKF()
    if measurement_covariance is None:
        measurement_covariance = np.diag((0.10, 0.10, 0.08)) ** 2
    measurement_covariance = np.asarray(measurement_covariance, dtype=float)
    sample_count = len(data.timestamps)
    state_dimension = estimator.STATE_DIM
    states = np.full((sample_count, state_dimension), np.nan)
    covariance_diagonal = np.full((sample_count, state_dimension), np.nan)
    accepted = np.zeros(sample_count, dtype=bool)
    nis = np.full(sample_count, np.nan)
    initialized = np.zeros(sample_count, dtype=bool)

    for index, timestamp in enumerate(data.timestamps):
        if not estimator.initialized:
            if data.detected[index]:
                estimator.initialize(data.measurements[index], float(timestamp), position_variance=np.diag(measurement_covariance))
                accepted[index] = True
        else:
            estimator.predict(float(timestamp))
            if data.detected[index]:
                accepted[index], nis[index] = estimator.update(data.measurements[index], measurement_covariance)

        if estimator.initialized:
            states[index] = estimator.state
            covariance_diagonal[index] = np.diag(estimator.covariance)
            initialized[index] = True

    return EstimatorReplay(states, covariance_diagonal, accepted, nis, initialized)


def calculate_metrics(data: SimulationData, replay: EstimatorReplay) -> dict[str, float]:
    """Return comparable raw and filtered position metrics in metres."""
    truth_position = data.truth_state[:, :3]
    raw_mask = data.detected
    filtered_mask = replay.initialized
    raw_error = np.linalg.norm(data.measurements[raw_mask] - truth_position[raw_mask], axis=1)
    filtered_error = np.linalg.norm(replay.states[filtered_mask, :3] - truth_position[filtered_mask], axis=1)
    lost_mask = (~data.detected) & replay.initialized
    lost_error = np.linalg.norm(replay.states[lost_mask, :3] - truth_position[lost_mask], axis=1)
    return {
        "raw_position_rmse_m": float(np.sqrt(np.mean(raw_error**2))),
        "filtered_position_rmse_m": float(np.sqrt(np.mean(filtered_error**2))),
        "filtered_position_p95_m": float(np.percentile(filtered_error, 95)),
        "max_error_during_dropout_m": float(np.max(lost_error)) if len(lost_error) else 0.0,
        "measurement_accept_rate": float(np.mean(replay.accepted[data.detected])) if np.any(data.detected) else 0.0,
        "outlier_rejection_rate": float(1.0 - np.mean(replay.accepted[data.outlier])) if np.any(data.outlier) else 0.0,
    }


def iter_default_scenarios(randomize: bool = False, seed: Optional[int] = 7) -> Iterable[tuple[str, SimulationConfig]]:
    """Scenarios used by the command-line verification run."""
    master_rng = np.random.default_rng(seed)
    def scenario_seed(fixed_seed: int) -> Optional[int]:
        return int(master_rng.integers(0, 2**31 - 1)) if randomize else fixed_seed
    yield "straight", SimulationConfig(dropout_intervals_s=((4.0, 5.0),), outlier_probability=0.02, seed=scenario_seed(7), randomize_trajectory=randomize)
    yield "circle", SimulationConfig(dropout_intervals_s=((7.0, 8.0),), outlier_probability=0.02, seed=scenario_seed(11), randomize_trajectory=randomize)
    yield "figure8", SimulationConfig(dropout_intervals_s=((8.0, 9.0),), outlier_probability=0.02, seed=scenario_seed(13), randomize_trajectory=randomize)
    yield "zigzag", SimulationConfig(dropout_intervals_s=((6.0, 7.0),), outlier_probability=0.02, seed=scenario_seed(17), randomize_trajectory=randomize)
    yield "stop_go", SimulationConfig(dropout_intervals_s=((10.0, 11.0),), outlier_probability=0.02, seed=scenario_seed(19), randomize_trajectory=randomize)
