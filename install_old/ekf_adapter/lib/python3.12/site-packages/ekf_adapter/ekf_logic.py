from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence

import numpy as np

from .target_state_ekf import EstimatorSnapshot, TargetStateEKF


class MeasurementOutcome(Enum):
    INITIALIZED = "initialized"
    ACCEPTED = "accepted"
    REACQUIRED = "reacquired"
    REJECTED_ORDER = "rejected_order"
    REJECTED_GATE = "rejected_gate"


@dataclass(frozen=True)
class TrackerConfig:
    gate_accel_mps2: float = 2.0
    gate_speed_fraction: float = 0.3
    gate_margin_m: float = 0.4
    gate_min_m: float = 0.6
    gate_max_m: float = 2.0
    gate_min_gap_s: float = 0.02
    gate_max_gap_s: float = 1.5
    reacquire_frames: int = 2
    candidate_window_s: float = 1.0
    candidate_step_min_m: float = 0.5
    candidate_step_margin_m: float = 0.3
    velocity_baseline_s: float = 0.3
    decay_after_s: float = 1.0
    decay_tau_s: float = 1.3
    out_of_order_tol_s: float = 1e-3
    detected_timeout_s: float = 0.25

    def __post_init__(self) -> None:
        if self.reacquire_frames < 2:
            raise ValueError("reacquire_frames must be at least 2")
        if self.decay_tau_s <= 0.0:
            raise ValueError("decay_tau_s must be positive")
        if self.gate_min_m > self.gate_max_m:
            raise ValueError("gate_min_m must not exceed gate_max_m")


@dataclass(frozen=True)
class NoiseConfig:
    lateral_floor_m: float = 0.02
    lateral_per_m: float = 0.003
    depth_floor_m: float = 0.02
    depth_per_m2: float = 0.01
    attitude_std_rad: float = 0.02
    vehicle_position_std_m: float = 0.05


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError("quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def transform_point(point: Sequence[float], rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    return rotation @ np.asarray(point, dtype=float) + np.asarray(translation, dtype=float)


def build_world_covariance(
    camera_position: Sequence[float], rotation: np.ndarray, noise: NoiseConfig
) -> np.ndarray:
    position = np.asarray(camera_position, dtype=float)
    depth = max(float(position[2]), 0.0)
    distance = float(np.linalg.norm(position))
    sigma_lateral = max(noise.lateral_floor_m, noise.lateral_per_m * depth)
    sigma_depth = max(noise.depth_floor_m, noise.depth_per_m2 * depth * depth)
    camera_covariance = np.diag([sigma_lateral**2, sigma_lateral**2, sigma_depth**2])
    world_covariance = rotation @ camera_covariance @ rotation.T
    isotropic = (noise.attitude_std_rad * distance) ** 2 + noise.vehicle_position_std_m**2
    world_covariance = world_covariance + np.eye(3) * isotropic
    return 0.5 * (world_covariance + world_covariance.T)


class TargetTracker:
    def __init__(self, estimator: TargetStateEKF, config: Optional[TrackerConfig] = None) -> None:
        self._estimator = estimator
        self.config = config if config is not None else TrackerConfig()
        self._last_valid_time: Optional[float] = None
        self._candidate_count = 0
        self._candidate_position: Optional[np.ndarray] = None
        self._candidate_time = 0.0
        self._candidate_first_position: Optional[np.ndarray] = None
        self._candidate_first_time = 0.0

    @property
    def initialized(self) -> bool:
        return self._estimator.initialized

    @property
    def last_valid_time(self) -> Optional[float]:
        return self._last_valid_time

    def age(self, now: float) -> float:
        if self._last_valid_time is None:
            return math.inf
        return max(0.0, now - self._last_valid_time)

    def detected(self, now: float) -> bool:
        return self.age(now) <= self.config.detected_timeout_s

    def snapshot_at(self, now: float) -> Optional[EstimatorSnapshot]:
        if not self._estimator.initialized:
            return None
        probe = copy.deepcopy(self._estimator)
        self._advance(probe, now)
        return probe.snapshot()

    def process_measurement(
        self, position: Sequence[float], covariance: np.ndarray, stamp: float
    ) -> MeasurementOutcome:
        measured = np.asarray(position, dtype=float)
        if not self._estimator.initialized:
            self._start_track(measured, covariance, stamp, np.zeros(3))
            return MeasurementOutcome.INITIALIZED
        if stamp < self._estimator.timestamp - self.config.out_of_order_tol_s:
            return MeasurementOutcome.REJECTED_ORDER
        self._advance(self._estimator, stamp)
        gap = stamp - self._last_valid_time
        innovation = float(np.linalg.norm(measured[:2] - self._estimator.state[:2]))
        if innovation <= self._allowed_distance(gap):
            accepted, _ = self._estimator.update(measured, covariance)
            if accepted:
                self._last_valid_time = stamp
                self._reset_candidate()
                return MeasurementOutcome.ACCEPTED
        return self._handle_out_of_gate(measured, covariance, stamp)

    def _advance(self, estimator: TargetStateEKF, timestamp: float) -> None:
        if timestamp <= estimator.timestamp:
            return
        decay_start = max(estimator.timestamp, self._last_valid_time + self.config.decay_after_s)
        decay_dt = timestamp - decay_start
        if decay_dt > 0.0:
            if decay_start > estimator.timestamp:
                estimator.predict(decay_start)
            estimator.decay_velocity(math.exp(-decay_dt / self.config.decay_tau_s))
        estimator.predict(timestamp)

    def _allowed_distance(self, gap: float) -> float:
        cfg = self.config
        dt = min(max(gap, cfg.gate_min_gap_s), cfg.gate_max_gap_s)
        budget = (
            0.5 * cfg.gate_accel_mps2 * dt * dt
            + cfg.gate_speed_fraction * self._estimator.v_max * dt
            + cfg.gate_margin_m
        )
        return min(cfg.gate_max_m, max(cfg.gate_min_m, budget))

    def _handle_out_of_gate(
        self, measured: np.ndarray, covariance: np.ndarray, stamp: float
    ) -> MeasurementOutcome:
        cfg = self.config
        consistent = False
        if self._candidate_position is not None:
            elapsed = stamp - self._candidate_time
            if 0.0 <= elapsed < cfg.candidate_window_s:
                step = max(cfg.candidate_step_min_m, self._estimator.v_max * elapsed + cfg.candidate_step_margin_m)
                distance = float(np.linalg.norm(measured[:2] - self._candidate_position[:2]))
                consistent = distance <= step
        if consistent:
            self._candidate_count += 1
        else:
            self._candidate_count = 1
            self._candidate_first_position = measured.copy()
            self._candidate_first_time = stamp
        self._candidate_position = measured.copy()
        self._candidate_time = stamp
        if self._candidate_count >= cfg.reacquire_frames:
            baseline = stamp - self._candidate_first_time
            velocity = np.zeros(3)
            if baseline >= cfg.velocity_baseline_s:
                velocity = (measured - self._candidate_first_position) / baseline
            self._start_track(measured, covariance, stamp, velocity)
            return MeasurementOutcome.REACQUIRED
        return MeasurementOutcome.REJECTED_GATE

    def _start_track(
        self, measured: np.ndarray, covariance: np.ndarray, stamp: float, velocity: np.ndarray
    ) -> None:
        self._estimator.initialize(
            measured,
            stamp,
            velocity=velocity,
            position_variance=np.diag(covariance),
        )
        self._last_valid_time = stamp
        self._reset_candidate()

    def _reset_candidate(self) -> None:
        self._candidate_count = 0
        self._candidate_position = None
        self._candidate_first_position = None
