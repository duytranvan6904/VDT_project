from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class SMCParams:
    """Parameters for the SMC Precision Landing Guidance Law (v2.4)."""
    ka: float = 0.20
    kb: float = 0.60
    kc: float = 0.40
    k1: float = 0.1395
    k2: float = 0.1784
    k3: float = 0.0442
    n: float = 3.0
    m: float = 5.0
    theta_des_rad: float = math.pi / 4.0  # 45 deg glide slope
    zeta_des_rad: float = 0.0
    v_max: float = 1.80                   # Max velocity in approach (m/s)
    v_final_xy_max: float = 1.30          # Max horizontal centering velocity (m/s, tracks 1.0 m/s pad)
    v_descend_fast: float = 0.55          # Descent velocity when altitude > 1.2m or glide (m/s)
    v_descend_touch: float = 0.22         # Touchdown descent velocity when near ground (m/s)
    rswitch_default: float = 5.00         # Default switching radius (m)
    final_descent_rxy_m: float = 0.85     # Final descent cylinder radius (m)
    final_descent_alt_m: float = 0.60     # Final descent start altitude (m)
    final_centering_kp: float = 0.70      # Centering gain
    glide_centering_kp: float = 0.70      # Glide pursuit gain
    target_velocity_ff_gain: float = 0.90 # Target velocity feedforward gain
    rmin: float = 0.05                    # Singularity avoidance radius (m)
    final_xy_gate: float = 0.45           # Centering gate in final descent (m)
    corr_fade_low_alt_m: float = 0.10     # Altitude below which correction is 0 (m)
    corr_fade_high_alt_m: float = 0.20    # Altitude below which correction starts fading (m)
    max_altitude_rate_mps: float = 2.00   # Max allowable altitude rate change for spike filtering (m/s)


@dataclass
class SMCResult:
    velocity_cmd: np.ndarray
    yaw_rate_cmd: float
    sub_phase: str
    sliding_surface: np.ndarray
    r_xy: float
    r_z: float
    valid: bool


class SMCGuidance:
    def __init__(self, params: Optional[SMCParams] = None):
        self.p = params or SMCParams()
        self.Vp = 0.5
        self.alpha_p = 0.0
        self.gamma = -0.1
        self.r_z_filtered: Optional[float] = None
        self.initialized = False

    def reset(self, initial_vel: np.ndarray):
        vx, vy, vz = initial_vel
        v_xy = math.hypot(vx, vy)
        self.Vp = max(0.1, float(np.linalg.norm(initial_vel)))
        self.alpha_p = math.atan2(vy, vx)
        self.gamma = math.atan2(vz, max(v_xy, 1e-4))
        self.r_z_filtered = None
        self.initialized = True

    def step(
        self,
        drone_pos: np.ndarray,
        drone_vel: np.ndarray,
        target_pos: np.ndarray,
        target_vel: Optional[np.ndarray] = None,
        dt: float = 0.05,
        rswitch_override: Optional[float] = None,
        sliding_weight: float = 1.0,
    ) -> SMCResult:
        if not self.initialized:
            self.reset(drone_vel)
        if target_vel is None:
            target_vel = np.zeros(3)

        dx = target_pos[0] - drone_pos[0]
        dy = target_pos[1] - drone_pos[1]
        dz = drone_pos[2] - target_pos[2]
        r_xy = math.hypot(dx, dy)
        r_z = max(0.0, dz)

        # Altitude Spike & Ground-Effect Jump Filter (Rate-of-Climb Slew Limiter)
        if self.r_z_filtered is None:
            self.r_z_filtered = r_z
        else:
            max_dz = self.p.max_altitude_rate_mps * max(1e-3, dt)
            self.r_z_filtered = float(np.clip(r_z, self.r_z_filtered - max_dz, self.r_z_filtered + max_dz))
        r_z = self.r_z_filtered

        rswitch = rswitch_override if rswitch_override is not None else self.p.rswitch_default
        rswitch = max(self.p.rmin, rswitch)

        final_radius = min(rswitch, self.p.final_descent_rxy_m)
        if r_xy <= final_radius and r_z <= self.p.final_descent_alt_m:
            return self._final_descent_step(dx, dy, r_z, target_vel)

        return self._glide_slope_step(
            target_vel, dx, dy, r_xy, r_z, rswitch, dt, sliding_weight
        )

    def _final_descent_step(self, dx: float, dy: float, r_z: float, target_vel: Optional[np.ndarray]) -> SMCResult:
        target_vel_xy = (
            np.asarray(target_vel, dtype=np.float64)[:2]
            if target_vel is not None else np.zeros(2)
        )

        # Attitude Leveling: Fade out position correction as drone touches down!
        # When r_z <= corr_fade_low_alt_m (0.10m), corr_fade -> 0, eliminating horizontal pitch tilt
        # so drone lands perfectly flat on all 4 legs without nose-diving on moving pad.
        corr_fade = float(np.clip(
            (r_z - self.p.corr_fade_low_alt_m) / max(0.01, self.p.corr_fade_high_alt_m - self.p.corr_fade_low_alt_m),
            0.0, 1.0
        ))

        xy_cmd = self._position_velocity(
            dx, dy, target_vel_xy, self.p.final_centering_kp * corr_fade, self.p.v_final_xy_max
        )

        # Profile descending speed by altitude zones (v2.4)
        cushion_high = min(0.50, self.p.final_descent_alt_m * 0.8)
        touchdown_low = 0.25
        if r_z >= cushion_high:
            vz_cmd = -self.p.v_descend_fast
        elif r_z <= touchdown_low:
            vz_cmd = -self.p.v_descend_touch
        else:
            alpha = (r_z - touchdown_low) / max(0.05, cushion_high - touchdown_low)
            vz_cmd = -(self.p.v_descend_touch + alpha * (self.p.v_descend_fast - self.p.v_descend_touch))

        # Do not descend if horizontal tracking is outside centering gate, unless very close to ground (< 0.25m)
        if math.hypot(dx, dy) > self.p.final_xy_gate and r_z > touchdown_low:
            vz_cmd = 0.0

        vel_cmd = np.array([xy_cmd[0], xy_cmd[1], vz_cmd], dtype=np.float64)
        self.reset(vel_cmd)
        return SMCResult(
            velocity_cmd=vel_cmd,
            yaw_rate_cmd=0.0,
            sub_phase='FINAL_DESCENT',
            sliding_surface=np.zeros(3),
            r_xy=math.hypot(dx, dy),
            r_z=r_z,
            valid=True,
        )

    @staticmethod
    def _position_velocity(dx: float, dy: float, target_vel_xy: np.ndarray, gain: float, speed_limit: float) -> np.ndarray:
        corr = gain * np.array([dx, dy], dtype=np.float64)
        target_velocity = np.asarray(target_vel_xy, dtype=np.float64)[:2]
        limit = max(0.0, float(speed_limit))
        corr_speed = float(np.linalg.norm(corr))
        if corr_speed > limit and corr_speed > 1e-9:
            corr *= limit / corr_speed

        command = corr.copy()
        if target_velocity.shape == (2,) and np.all(np.isfinite(target_velocity)):
            command += target_velocity
        return command

    def _glide_slope_step(
        self, target_vel: np.ndarray, dx: float, dy: float, r_xy: float, r_z: float, rswitch: float, dt: float, sliding_weight: float
    ) -> SMCResult:
        v_tx, v_ty = target_vel[0], target_vel[1]
        Vt = math.hypot(v_tx, v_ty)
        alpha_t = math.atan2(v_ty, v_tx) if Vt > 1e-4 else 0.0
        dVt = 0.0
        dalpha_t = 0.0
        ddalpha_t = 0.0

        Vp = max(0.1, self.Vp)
        alpha_p = self.alpha_p
        gamma = float(np.clip(self.gamma, -math.pi / 2.5, math.pi / 2.5))
        cg = math.cos(gamma)
        sg = math.sin(gamma)

        psi = math.atan2(dy, dx)
        dRxy = Vt * math.cos(alpha_t - psi) - Vp * cg * math.cos(alpha_p - psi)
        dpsi = (
            Vt * math.sin(alpha_t - psi) - Vp * cg * math.sin(alpha_p - psi)
        ) / max(r_xy, 1e-3)
        dRz = Vp * sg - target_vel[2]

        td = math.tan(self.p.theta_des_rad)
        S1 = dRxy + self.p.ka * r_xy
        S2 = -dRz + td * dRxy + self.p.kb * (-r_z + td * r_xy)
        e_raw = psi - alpha_t - self.p.zeta_des_rad
        e_psi = math.atan2(math.sin(e_raw), math.cos(e_raw))
        S3 = (dpsi - dalpha_t) + self.p.kc * e_psi

        S = np.array([sliding_weight * S1, S2, S3], dtype=np.float64)
        power = self.p.n / self.p.m
        Sp = np.sign(S) * (np.abs(S) ** power)

        dp = alpha_p - psi
        dt_ang = alpha_t - psi

        Ap = np.array([
            [-math.cos(dp) * cg, Vp * math.sin(dp) * cg, Vp * math.cos(dp) * sg],
            [sg, 0.0, Vp * cg],
            [-math.sin(dp) * cg, -Vp * math.cos(dp) * cg, Vp * math.sin(dp) * sg],
        ], dtype=np.float64)
        M = np.array([[1.0, 0.0, 0.0], [td, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        A = M @ Ap

        dRxy_model = Vt * math.cos(dt_ang) - Vp * cg * math.cos(dp)
        Fxy = (
            Vp * math.sin(dp) * cg * dpsi
            - dVt * math.cos(dt_ang)
            + Vt * math.sin(dt_ang) * (dalpha_t - dpsi)
        )
        B1 = -self.p.k1 * Sp[0] + Fxy - self.p.ka * dRxy_model
        B2 = td * (Fxy - self.p.kb * dRxy_model) - self.p.k2 * Sp[1] - self.p.kb * Vp * sg
        B3 = (
            -r_xy * self.p.k3 * Sp[2]
            - self.p.kc * r_xy * (dpsi - dalpha_t)
            + ddalpha_t * r_xy
            + dRxy * dpsi
            - Vp * math.cos(dp) * cg * dpsi
            - Vt * math.cos(dt_ang) * (dalpha_t - dpsi)
            - dVt * math.sin(dt_ang)
        )
        B = np.array([B1, B2, B3], dtype=np.float64)

        try:
            cond = np.linalg.cond(A)
            if not np.isfinite(cond) or cond > 1e12:
                raise np.linalg.LinAlgError('ill-conditioned')
            Uraw = np.linalg.solve(A, B)
        except np.linalg.LinAlgError:
            Uraw = np.array([0.0, 0.5 * (psi - alpha_p), -0.2])

        dVp = float(np.clip(Uraw[0], -10.0, 10.0))
        dalpha_p = float(np.clip(Uraw[1], -math.pi / 2.0, math.pi / 2.0))
        dgamma = float(np.clip(Uraw[2], -math.pi / 2.0, math.pi / 2.0))

        dt_safe = min(max(dt, 0.001), 0.1)
        self.Vp = float(np.clip(self.Vp + dVp * dt_safe, 0.20, self.p.v_max))
        self.alpha_p = float(self.alpha_p + dalpha_p * dt_safe)
        self.gamma = float(np.clip(self.gamma + dgamma * dt_safe, -math.pi / 3.0, 0.0))

        xy_cmd = self._position_velocity(
            dx, dy,
            np.asarray(target_vel, dtype=np.float64)[:2] * self.p.target_velocity_ff_gain,
            self.p.glide_centering_kp,
            self.p.v_max,
        )
        vx_cmd, vy_cmd = float(xy_cmd[0]), float(xy_cmd[1])
        horizontal_speed = math.hypot(vx_cmd, vy_cmd)

        if r_xy <= rswitch and r_z > self.p.final_descent_alt_m:
            # 45 deg glide descent profile
            slope_speed = max(0.35, horizontal_speed * math.tan(abs(self.p.theta_des_rad)))
            alt_speed = max(self.p.v_descend_touch, 0.40 * r_z)
            descent_speed = min(self.p.v_descend_fast, max(slope_speed, alt_speed))
            vz_cmd = -descent_speed
            horizontal_limit = math.sqrt(max(0.0, self.p.v_max ** 2 - vz_cmd ** 2))
            if horizontal_speed > horizontal_limit and horizontal_speed > 1e-9:
                scale = horizontal_limit / horizontal_speed
                vx_cmd *= scale
                vy_cmd *= scale
                horizontal_speed = horizontal_limit
            self.gamma = math.atan2(vz_cmd, max(horizontal_speed, 1e-4))
        elif r_xy <= rswitch and r_z <= self.p.final_descent_alt_m:
            vz_cmd = -self.p.v_descend_touch
            self.gamma = math.atan2(vz_cmd, max(horizontal_speed, 1e-4))
        else:
            vz_cmd = 0.0
            self.gamma = 0.0

        return SMCResult(
            velocity_cmd=np.array([vx_cmd, vy_cmd, vz_cmd], dtype=np.float64),
            yaw_rate_cmd=float(np.clip(dalpha_p, -0.6, 0.6)),
            sub_phase='GLIDE_SLOPE',
            sliding_surface=S,
            r_xy=r_xy,
            r_z=r_z,
            valid=True,
        )
