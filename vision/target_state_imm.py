"""Lightweight interacting-model target estimator.

This module runs CV, coordinated-turn and constant-acceleration models in
parallel.  It exposes the same position/velocity interface as the existing
6-state estimator, which makes it suitable for offline comparison before ROS
integration.  Model mixing is performed on the common position/velocity
output; each model keeps its native internal state.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

from .target_state_ct_ekf import CoordinatedTurnEKF
from .target_state_ekf import TargetStateEKF


class ConstantAccelerationKF:
    """Nine-state linear constant-acceleration model."""

    STATE_DIM = 9

    def __init__(self, jerk_variance: float = 1.0, gate_threshold: float = 16.27) -> None:
        if jerk_variance <= 0.0 or gate_threshold <= 0.0:
            raise ValueError("jerk_variance and gate_threshold must be positive")
        self._jerk_variance = float(jerk_variance)
        self.gate_threshold = float(gate_threshold)
        self._state = np.zeros(9)
        self._covariance = np.eye(9)
        self._timestamp = None
        self._initialized = False

    @property
    def initialized(self):
        return self._initialized

    @property
    def state(self):
        return self._state.copy()

    @property
    def covariance(self):
        return self._covariance.copy()

    def initialize(self, position, timestamp, velocity=(0.0, 0.0, 0.0), **kwargs):
        position = np.asarray(position, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        if position.shape != (3,) or velocity.shape != (3,) or not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            raise ValueError("position and velocity must be finite 3-vectors")
        self._state = np.concatenate((position, velocity, np.zeros(3)))
        self._covariance = np.diag((0.25, 0.25, 0.25, 9.0, 9.0, 4.0, 16.0, 16.0, 9.0))
        self._timestamp = float(timestamp)
        self._initialized = True

    def predict(self, timestamp):
        if not self._initialized:
            raise RuntimeError("initialize the estimator before predict")
        dt = float(timestamp) - self._timestamp
        if dt < -1e-9:
            raise ValueError("predict received an out-of-order timestamp")
        f = np.eye(9)
        f[:3, 3:6] = np.eye(3) * dt
        f[:3, 6:9] = np.eye(3) * (0.5 * dt * dt)
        f[3:6, 6:9] = np.eye(3) * dt
        q = np.zeros((9, 9))
        q[6:9, 6:9] = np.eye(3) * self._jerk_variance * max(dt, 0.0)
        self._state = f @ self._state
        self._covariance = self._symmetrize(f @ self._covariance @ f.T + q)
        self._timestamp = float(timestamp)

    def update(self, position, measurement_covariance):
        z = np.asarray(position, dtype=float)
        r = np.asarray(measurement_covariance, dtype=float)
        if z.shape != (3,) or not np.all(np.isfinite(z)) or r.shape != (3, 3):
            raise ValueError("invalid measurement")
        h = np.zeros((3, 9)); h[:, :3] = np.eye(3)
        innovation = z - h @ self._state
        s = h @ self._covariance @ h.T + r
        nis = float(innovation @ np.linalg.solve(s, innovation))
        if nis > self.gate_threshold:
            return False, nis
        gain = np.linalg.solve(s, h @ self._covariance).T
        self._state += gain @ innovation
        identity = np.eye(9)
        residual = identity - gain @ h
        self._covariance = self._symmetrize(residual @ self._covariance @ residual.T + gain @ r @ gain.T)
        return True, nis

    @staticmethod
    def _symmetrize(matrix):
        return 0.5 * (matrix + matrix.T)


class TargetStateIMM:
    """CV/CT/CA model bank with a common six-state output."""

    STATE_DIM = 6

    def __init__(self, gate_threshold: float = 16.27) -> None:
        self._models = [
            TargetStateEKF(process_accel_variance=(1.0, 1.0, 0.5), gate_threshold=gate_threshold),
            CoordinatedTurnEKF(acceleration_variance=(0.8, 0.8, 0.4), turn_rate_variance=0.08, gate_threshold=gate_threshold),
            ConstantAccelerationKF(jerk_variance=1.0, gate_threshold=gate_threshold),
        ]
        self._probabilities = np.ones(3) / 3.0
        self._state = np.zeros(6)
        self._covariance = np.eye(6)
        self._timestamp = None
        self._initialized = False

    @property
    def initialized(self):
        return self._initialized

    @property
    def state(self):
        return self._state.copy()

    @property
    def covariance(self):
        return self._covariance.copy()

    @property
    def model_probabilities(self):
        return self._probabilities.copy()

    def initialize(self, position, timestamp, velocity=(0.0, 0.0, 0.0), **kwargs):
        for model in self._models:
            model.initialize(position, timestamp, velocity=velocity)
        self._probabilities = np.ones(3) / 3.0
        self._timestamp = float(timestamp)
        self._initialized = True
        self._combine()

    def predict(self, timestamp):
        if not self._initialized:
            raise RuntimeError("initialize the estimator before predict")
        for model in self._models:
            model.predict(timestamp)
        self._timestamp = float(timestamp)
        self._combine()

    def update(self, position, measurement_covariance):
        if not self._initialized:
            raise RuntimeError("initialize the estimator before update")
        nis_values = []
        accepted_values = []
        for model in self._models:
            accepted, nis = model.update(position, measurement_covariance)
            accepted_values.append(accepted)
            nis_values.append(nis)
        nis_values = np.asarray(nis_values)
        likelihood = np.exp(-0.5 * np.minimum(nis_values, 100.0))
        likelihood[~np.asarray(accepted_values)] *= 1e-6
        weighted = self._probabilities * likelihood
        if np.sum(weighted) <= 1e-15:
            self._probabilities = np.ones(3) / 3.0
        else:
            self._probabilities = weighted / np.sum(weighted)
        self._combine()
        return bool(np.any(accepted_values)), float(np.min(nis_values))

    def _combine(self):
        outputs = np.array([model.state[:6] for model in self._models])
        self._state = np.sum(self._probabilities[:, None] * outputs, axis=0)
        covariance = np.zeros((6, 6))
        for probability, model, state in zip(self._probabilities, self._models, outputs):
            delta = (state - self._state).reshape(6, 1)
            covariance += probability * (model.covariance[:6, :6] + delta @ delta.T)
        self._covariance = 0.5 * (covariance + covariance.T)
