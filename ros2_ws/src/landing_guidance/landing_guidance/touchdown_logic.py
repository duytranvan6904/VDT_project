from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class TouchdownParams:
    optical_height_threshold_m: float = 0.40
    descent_cmd_threshold_mps: float = -0.10
    stoppage_vz_max_mps: float = 0.08
    stoppage_dz_dt_max_mps: float = 0.05
    jerk_impact_threshold: float = 3.50
    confirmation_duration_s: float = 0.35
    altitude_ceiling_m: float = 0.80
    min_descent_time_s: float = 1.0
    seen_descending_vz_mps: float = -0.10
    seen_descending_hold_s: float = 0.30


@dataclass
class TouchdownResult:
    touchdown_confirmed: bool
    kinematic_stopped: bool
    optical_near: bool
    impact_detected: bool
    confirm_duration_s: float
    descent_time_s: float
    seen_descending: bool
    actual_vz: float
    commanded_vz: float
    optical_z: Optional[float]
    diagnostic_reason: str


class TouchdownDetector:
    def __init__(self, params: Optional[TouchdownParams] = None):
        self.p = params or TouchdownParams()
        self.reset()

    def reset(self):
        self.is_latched = False
        self.confirm_start: Optional[float] = None
        self.last_alt: Optional[float] = None
        self.last_alt_time: Optional[float] = None
        self.last_accel_z = 9.81
        self.last_accel_time: Optional[float] = None
        self.descent_start: Optional[float] = None
        self.fall_start: Optional[float] = None
        self.seen_descending = False

    def step(
        self,
        actual_vz: float,
        commanded_vz: float,
        current_alt_z: float,
        optical_z: Optional[float] = None,
        accel_z_body: Optional[float] = None,
        px4_landed: bool = False,
        current_time: Optional[float] = None,
    ) -> TouchdownResult:
        now = time.monotonic() if current_time is None else current_time
        p = self.p

        if self.is_latched:
            return TouchdownResult(
                True, True, True, False, p.confirmation_duration_s, 0.0, True,
                actual_vz, commanded_vz, optical_z, 'LATCHED_TOUCHDOWN',
            )

        dz_dt = 0.0
        if self.last_alt is not None and self.last_alt_time is not None:
            dz_dt = (current_alt_z - self.last_alt) / max(1e-4, now - self.last_alt_time)
        self.last_alt = current_alt_z
        self.last_alt_time = now

        impact = False
        if accel_z_body is not None:
            if self.last_accel_time is not None:
                jerk = abs(accel_z_body - self.last_accel_z) / max(1e-4, now - self.last_accel_time)
                impact = jerk >= p.jerk_impact_threshold
            self.last_accel_z = accel_z_body
            self.last_accel_time = now

        commanding = commanded_vz <= p.descent_cmd_threshold_mps
        if commanding:
            if self.descent_start is None:
                self.descent_start = now
            descent_time = now - self.descent_start
        else:
            self.descent_start = None
            descent_time = 0.0

        if actual_vz <= p.seen_descending_vz_mps:
            if self.fall_start is None:
                self.fall_start = now
            if now - self.fall_start >= p.seen_descending_hold_s:
                self.seen_descending = True
        else:
            self.fall_start = None

        stopped = abs(actual_vz) <= p.stoppage_vz_max_mps and abs(dz_dt) <= p.stoppage_dz_dt_max_mps
        descent_proven = self.seen_descending and descent_time >= p.min_descent_time_s
        kinematic_stopped = descent_proven and stopped

        optical_near = optical_z is not None and optical_z <= p.optical_height_threshold_m

        contact = False
        reason = 'DESCENT_IN_PROGRESS'
        if px4_landed and self.seen_descending:
            contact = True
            reason = 'PX4_FIRMWARE_LANDED'
        elif kinematic_stopped and optical_near:
            contact = True
            reason = f'KINEMATIC_STOPPED (vz={actual_vz:.2f}, opt_z={optical_z})'
        elif impact and kinematic_stopped and optical_near:
            contact = True
            reason = 'IMPACT_JERK_STOPPED'

        confirm = 0.0
        if contact:
            if self.confirm_start is None:
                self.confirm_start = now
            confirm = now - self.confirm_start
            if confirm >= p.confirmation_duration_s:
                self.is_latched = True
                reason = f'TOUCHDOWN_CONFIRMED ({confirm:.2f}s)'
        else:
            self.confirm_start = None

        return TouchdownResult(
            self.is_latched, kinematic_stopped, optical_near, impact, confirm,
            descent_time, self.seen_descending, actual_vz, commanded_vz,
            optical_z, reason,
        )