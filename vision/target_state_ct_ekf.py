"""Coordinated-turn EKF for slow, curved H-Pad motion.

State order is ``[x, y, z, vx, vy, vz, omega]``.  Horizontal motion follows
a constant-turn-rate model while altitude remains constant-velocity.  The
implementation is hardware independent and consumes 3-D world-frame position
measurements, just like :mod:`vision.target_state_ekf`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


@dataclass(frozen=True)
class CTStateSnapshot:
    timestamp: float
    state: np.ndarray
    covariance: np.ndarray
    initialized: bool


class CoordinatedTurnEKF:
    """Estimate a slowly moving target with a constant-turn-rate model."""

    STATE_DIM = 7

    def __init__(
        self,
        acceleration_variance: Sequence[float] = (0.8, 0.8, 0.4),
        turn_rate_variance: float = 0.08,
        gate_threshold: float = 16.27,
    ) -> None:
        accel = np.asarray(acceleration_variance, dtype=float)
        if accel.shape != (3,) or np.any(accel <= 0.0):
            raise ValueError("acceleration_variance must contain three positive values")
        if turn_rate_variance <= 0.0 or gate_threshold <= 0.0:
            raise ValueError("turn_rate_variance and gate_threshold must be positive")
        self._accel_variance = accel
        self._turn_rate_variance = float(turn_rate_variance)
        self.gate_threshold = float(gate_threshold)
        self._state = np.zeros(self.STATE_DIM, dtype=float)
        self._covariance = np.eye(self.STATE_DIM, dtype=float)
        self._timestamp: Optional[float] = None
        self._initialized = False

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def timestamp(self) -> Optional[float]:
        return self._timestamp

    @property
    def state(self) -> np.ndarray:
        return self._state.copy()

    @property
    def covariance(self) -> np.ndarray:
        return self._covariance.copy()

    def initialize(
        self,
        position: Sequence[float],
        timestamp: float,
        velocity: Sequence[float] = (0.0, 0.0, 0.0),
        turn_rate: float = 0.0,
        position_variance: Sequence[float] = (0.25, 0.25, 0.25),
        velocity_variance: Sequence[float] = (9.0, 9.0, 4.0),
        turn_rate_variance: float = 0.5,
    ) -> None:
        position = self._vector3(position, "position")
        velocity = self._vector3(velocity, "velocity")
        position_variance = self._positive_vector3(position_variance, "position_variance")
        velocity_variance = self._positive_vector3(velocity_variance, "velocity_variance")
        if not np.isfinite(timestamp) or not np.isfinite(turn_rate) or turn_rate_variance <= 0.0:
            raise ValueError("timestamp, turn_rate and turn_rate_variance are invalid")
        self._state = np.concatenate((position, velocity, (float(turn_rate),)))
        self._covariance = np.diag(np.concatenate((position_variance, velocity_variance, (turn_rate_variance,))))
        self._timestamp = float(timestamp)
        self._initialized = True

    @staticmethod
    def transition(state: np.ndarray, dt: float) -> np.ndarray:
        """Propagate a state with exact constant-turn-rate integration."""
        if state.shape != (7,) or not np.all(np.isfinite(state)):
            raise ValueError("state must be a finite 7-vector")
        if not np.isfinite(dt) or dt < 0.0:
            raise ValueError("dt must be finite and non-negative")
        x, y, z, vx, vy, vz, omega = state
        theta = omega * dt
        if abs(omega) < 1e-6:
            sin_over_omega = dt
            one_minus_cos_over_omega = 0.0
        else:
            sin_over_omega = np.sin(theta) / omega
            one_minus_cos_over_omega = (1.0 - np.cos(theta)) / omega
        return np.array(
            [
                x + sin_over_omega * vx - one_minus_cos_over_omega * vy,
                y + one_minus_cos_over_omega * vx + sin_over_omega * vy,
                z + vz * dt,
                np.cos(theta) * vx - np.sin(theta) * vy,
                np.sin(theta) * vx + np.cos(theta) * vy,
                vz,
                omega,
            ],
            dtype=float,
        )

    def process_covariance(self, dt: float) -> np.ndarray:
        """Approximate process covariance for acceleration and turn-rate noise."""
        if not np.isfinite(dt) or dt < 0.0:
            raise ValueError("dt must be finite and non-negative")
        q = np.zeros((7, 7), dtype=float)
        q[:3, :3] = np.diag(self._accel_variance * dt**3 / 3.0)
        q[3:6, 3:6] = np.diag(self._accel_variance * dt)
        q[6, 6] = self._turn_rate_variance * dt
        return q

    def predict(self, timestamp: float) -> CTStateSnapshot:
        if not self._initialized or self._timestamp is None:
            raise RuntimeError("initialize the estimator before predict")
        if not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        dt = float(timestamp) - self._timestamp
        if dt < -1e-9:
            raise ValueError("predict received an out-of-order timestamp")
        dt = max(0.0, dt)
        previous_state = self._state.copy()
        self._state = self.transition(previous_state, dt)
        jacobian = self._numerical_jacobian(previous_state, dt)
        self._covariance = jacobian @ self._covariance @ jacobian.T + self.process_covariance(dt)
        self._covariance = self._symmetrize(self._covariance)
        self._timestamp = float(timestamp)
        return self.snapshot()

    def update(self, position: Sequence[float], measurement_covariance: np.ndarray) -> tuple[bool, float]:
        if not self._initialized:
            raise RuntimeError("initialize the estimator before update")
        measurement = self._vector3(position, "position")
        r = np.asarray(measurement_covariance, dtype=float)
        if r.shape != (3, 3) or not np.all(np.isfinite(r)):
            raise ValueError("measurement_covariance must be a finite 3x3 matrix")
        r = self._symmetrize(r)
        if np.min(np.linalg.eigvalsh(r)) <= 0.0:
            raise ValueError("measurement_covariance must be positive definite")
        h = np.zeros((3, self.STATE_DIM), dtype=float)
        h[:, :3] = np.eye(3)
        innovation = measurement - h @ self._state
        innovation_covariance = h @ self._covariance @ h.T + r
        nis = float(innovation.T @ np.linalg.solve(innovation_covariance, innovation))
        if nis > self.gate_threshold:
            return False, nis
        gain = np.linalg.solve(innovation_covariance, h @ self._covariance).T
        self._state = self._state + gain @ innovation
        identity = np.eye(self.STATE_DIM)
        residual_map = identity - gain @ h
        self._covariance = residual_map @ self._covariance @ residual_map.T + gain @ r @ gain.T
        self._covariance = self._symmetrize(self._covariance)
        return True, nis

    def snapshot(self) -> CTStateSnapshot:
        return CTStateSnapshot(
            timestamp=float(self._timestamp) if self._timestamp is not None else float("nan"),
            state=self.state,
            covariance=self.covariance,
            initialized=self._initialized,
        )

    @staticmethod
    def _numerical_jacobian(state: np.ndarray, dt: float) -> np.ndarray:
        jacobian = np.zeros((7, 7), dtype=float)
        for index in range(7):
            step = 1e-5 * max(1.0, abs(state[index]))
            plus = state.copy()
            minus = state.copy()
            plus[index] += step
            minus[index] -= step
            jacobian[:, index] = (CoordinatedTurnEKF.transition(plus, dt) - CoordinatedTurnEKF.transition(minus, dt)) / (2.0 * step)
        return jacobian

    @staticmethod
    def _vector3(value: Sequence[float], name: str) -> np.ndarray:
        vector = np.asarray(value, dtype=float)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"{name} must contain three finite values")
        return vector

    @classmethod
    def _positive_vector3(cls, value: Sequence[float], name: str) -> np.ndarray:
        vector = cls._vector3(value, name)
        if np.any(vector <= 0.0):
            raise ValueError(f"{name} must contain positive values")
        return vector

    @staticmethod
    def _symmetrize(matrix: np.ndarray) -> np.ndarray:
        return 0.5 * (matrix + matrix.T)
