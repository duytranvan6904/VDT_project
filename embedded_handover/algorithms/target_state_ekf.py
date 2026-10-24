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


def reacquisition_distance_budget(
    *,
    max_target_speed: float,
    time_since_last_measurement: float,
    margin_m: float = 0.50,
    hard_limit_m: float = 8.0,
) -> float:
    """Bound how far an untracked target may plausibly move before reacquisition.

    The speed budget grows with the measurement gap but has a finite ceiling so
    two mutually consistent PnP outliers cannot reset the filter arbitrarily far
    from its predicted trajectory.
    """
    speed = float(max_target_speed)
    elapsed = float(time_since_last_measurement)
    margin = float(margin_m)
    hard_limit = float(hard_limit_m)
    if speed < 0.0 or elapsed < 0.0 or margin < 0.0 or hard_limit <= 0.0:
        raise ValueError("reacquisition budget inputs must be non-negative")
    return min(hard_limit, speed * elapsed + margin)


def bounded_prediction_timestamp(
    anchor_time: float,
    requested_time: float,
    max_horizon_s: float,
) -> float:
    """Cap constant-velocity dead reckoning from the last accepted measurement."""
    anchor = float(anchor_time)
    requested = float(requested_time)
    horizon = float(max_horizon_s)
    if not np.isfinite(anchor) or not np.isfinite(requested):
        raise ValueError("prediction timestamps must be finite")
    if not np.isfinite(horizon) or horizon < 0.0:
        raise ValueError("max_horizon_s must be finite and non-negative")
    if requested < anchor:
        raise ValueError("requested_time must not precede anchor_time")
    return min(requested, anchor + horizon)


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
        v_max: float = 2.5,
    ) -> None:
        q_accel = np.asarray(process_accel_variance, dtype=float)
        if q_accel.shape != (3,) or np.any(q_accel <= 0.0):
            raise ValueError("process_accel_variance must contain three positive values")
        if gate_threshold <= 0.0:
            raise ValueError("gate_threshold must be positive")
        if v_max <= 0.0:
            raise ValueError("v_max must be positive")

        self._q_accel = q_accel
        self.gate_threshold = float(gate_threshold)
        self.v_max = float(v_max)
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
        velocity_variance: Sequence[float] = (1.0, 1.0, 0.5),
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
        self._clamp_velocity()
        self._timestamp = float(timestamp)
        self._initialized = True

    def _clamp_velocity(self) -> None:
        """Clamp horizontal velocity to physical vehicle limits."""
        v_xy = float(np.hypot(self._state[3], self._state[4]))
        if v_xy > self.v_max:
            scale = self.v_max / v_xy
            self._state[3] *= scale
            self._state[4] *= scale
        if abs(self._state[5]) > 1.5:
            self._state[5] = float(np.clip(self._state[5], -1.5, 1.5))

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
        predicted = self.predict_snapshot(timestamp)
        self._state = predicted.state.copy()
        self._covariance = predicted.covariance.copy()
        self._timestamp = predicted.timestamp
        return self.snapshot()

    def predict_snapshot(self, timestamp: float) -> EstimatorSnapshot:
        """Return a prediction without advancing the filter's measurement time.

        This is useful for publishing dead-reckoned state between sensor
        updates. Keeping the filter at its latest measurement timestamp lets
        the next delayed camera frame still be fused in chronological order.
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
        state = f @ self._state
        v_xy = float(np.hypot(state[3], state[4]))
        if v_xy > self.v_max:
            state[3:5] *= self.v_max / v_xy
        if abs(state[5]) > 1.5:
            state[5] = float(np.clip(state[5], -1.5, 1.5))
        covariance = f @ self._covariance @ f.T + self.process_covariance(dt)
        covariance = self._symmetrize(covariance)
        return EstimatorSnapshot(
            timestamp=float(timestamp),
            state=state,
            covariance=covariance,
            initialized=True,
        )

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
        prev_vel = self._state[3:5].copy()
        self._state = self._state + kalman_gain @ innovation
        # Bound per-step velocity innovation to prevent single-frame glitches from shocking velocity
        delta_v = self._state[3:5] - prev_vel
        norm_dv = float(np.hypot(delta_v[0], delta_v[1]))
        max_dv_per_step = 0.40
        if norm_dv > max_dv_per_step:
            self._state[3:5] = prev_vel + delta_v * (max_dv_per_step / norm_dv)
        self._clamp_velocity()
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
