"""Hardware-independent 6-state target estimator.

The core model is a linear constant-velocity Kalman filter in one world frame.
It deliberately has no ROS, camera or TF dependency so it can be validated with
synthetic trajectories before being connected to ArUco/PnP measurements.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


Array = np.ndarray


@dataclass(frozen=True)
class EstimatorSnapshot:
    """Immutable state published by :class:`TargetStateEKF`."""

    timestamp: float
    state: Array
    covariance: Array
    initialized: bool


class TargetStateEKF:
    """Estimate ``[x, y, z, vx, vy, vz]`` with a CV motion model.

    ``process_accel_variance`` is the continuous white-acceleration spectral
    density for each world axis.  A measurement is a 3-D position in that same
    world frame.  Later, the ROS adapter will be responsible for converting
    PnP camera coordinates into this frame before calling :meth:`update`.
    """

    STATE_DIM = 6
    MEASUREMENT_DIM = 3

    def __init__(
        self,
        process_accel_variance: Sequence[float] = (1.0, 1.0, 0.5),
        gate_threshold: float = 16.27,
    ) -> None:
        q_accel = np.asarray(process_accel_variance, dtype=float)
        if q_accel.shape != (3,) or np.any(q_accel <= 0.0):
            raise ValueError("process_accel_variance must contain three positive values")
        if gate_threshold <= 0.0:
            raise ValueError("gate_threshold must be positive")

        self._q_accel = q_accel
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
    def state(self) -> Array:
        return self._state.copy()

    @property
    def covariance(self) -> Array:
        return self._covariance.copy()

    def snapshot(self) -> EstimatorSnapshot:
        """Return a copy of the current estimate for logging or evaluation."""
        return EstimatorSnapshot(
            timestamp=float(self._timestamp) if self._timestamp is not None else float("nan"),
            state=self.state,
            covariance=self.covariance,
            initialized=self._initialized,
        )

    def initialize(
        self,
        position: Sequence[float],
        timestamp: float,
        velocity: Sequence[float] = (0.0, 0.0, 0.0),
        position_variance: Sequence[float] = (0.25, 0.25, 0.25),
        velocity_variance: Sequence[float] = (25.0, 25.0, 9.0),
    ) -> None:
        """Initialize from a trusted position measurement with unknown velocity."""
        pos = self._vector3(position, "position")
        vel = self._vector3(velocity, "velocity")
        pos_var = self._positive_vector3(position_variance, "position_variance")
        vel_var = self._positive_vector3(velocity_variance, "velocity_variance")
        if not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")

        self._state = np.concatenate((pos, vel))
        self._covariance = np.diag(np.concatenate((pos_var, vel_var)))
        self._timestamp = float(timestamp)
        self._initialized = True

    @staticmethod
    def transition_matrix(dt: float) -> Array:
        """Return the CV state-transition matrix for a non-negative interval."""
        if not np.isfinite(dt) or dt < 0.0:
            raise ValueError("dt must be finite and non-negative")
        f = np.eye(6, dtype=float)
        f[:3, 3:] = np.eye(3) * dt
        return f

    def process_covariance(self, dt: float) -> Array:
        """Discrete covariance for continuous white acceleration noise."""
        if not np.isfinite(dt) or dt < 0.0:
            raise ValueError("dt must be finite and non-negative")
        q = np.zeros((6, 6), dtype=float)
        q[:3, :3] = np.diag(self._q_accel * dt**3 / 3.0)
        q[:3, 3:] = np.diag(self._q_accel * dt**2 / 2.0)
        q[3:, :3] = q[:3, 3:]
        q[3:, 3:] = np.diag(self._q_accel * dt)
        return q

    def predict(self, timestamp: float) -> EstimatorSnapshot:
        """Propagate state and covariance to ``timestamp``.

        Prediction must be chronological.  The caller should buffer delayed
        sensor messages instead of silently applying them to a newer state.
        """
        if not self._initialized or self._timestamp is None:
            raise RuntimeError("initialize the estimator before predict")
        if not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        dt = float(timestamp) - self._timestamp
        if dt < -1e-9:
            raise ValueError("predict received an out-of-order timestamp")
        dt = max(0.0, dt)

        f = self.transition_matrix(dt)
        self._state = f @ self._state
        self._covariance = f @ self._covariance @ f.T + self.process_covariance(dt)
        self._covariance = self._symmetrize(self._covariance)
        self._timestamp = float(timestamp)
        return self.snapshot()

    def update(
        self,
        position: Sequence[float],
        measurement_covariance: Array,
    ) -> tuple[bool, float]:
        """Gate and fuse a position measurement at the current timestamp.

        Returns ``(accepted, nis)``.  A rejected measurement leaves the state
        unchanged.  The default gate is chi-square 3-D, 99.7% (16.27).
        """
        if not self._initialized:
            raise RuntimeError("initialize the estimator before update")
        z = self._vector3(position, "position")
        r = np.asarray(measurement_covariance, dtype=float)
        if r.shape != (3, 3) or not np.all(np.isfinite(r)):
            raise ValueError("measurement_covariance must be a finite 3x3 matrix")
        r = self._symmetrize(r)
        if np.min(np.linalg.eigvalsh(r)) <= 0.0:
            raise ValueError("measurement_covariance must be positive definite")

        h = np.zeros((3, 6), dtype=float)
        h[:, :3] = np.eye(3)
        innovation = z - h @ self._state
        innovation_covariance = h @ self._covariance @ h.T + r
        nis = float(innovation.T @ np.linalg.solve(innovation_covariance, innovation))
        if nis > self.gate_threshold:
            return False, nis

        kalman_gain = np.linalg.solve(innovation_covariance, h @ self._covariance).T
        self._state = self._state + kalman_gain @ innovation
        identity = np.eye(self.STATE_DIM)
        residual_map = identity - kalman_gain @ h
        self._covariance = residual_map @ self._covariance @ residual_map.T + kalman_gain @ r @ kalman_gain.T
        self._covariance = self._symmetrize(self._covariance)
        return True, nis

    @staticmethod
    def _vector3(value: Sequence[float], name: str) -> Array:
        vector = np.asarray(value, dtype=float)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"{name} must contain three finite values")
        return vector

    @classmethod
    def _positive_vector3(cls, value: Sequence[float], name: str) -> Array:
        vector = cls._vector3(value, name)
        if np.any(vector <= 0.0):
            raise ValueError(f"{name} must contain positive values")
        return vector

    @staticmethod
    def _symmetrize(matrix: Array) -> Array:
        return 0.5 * (matrix + matrix.T)
