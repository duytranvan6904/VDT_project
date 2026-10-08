from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class CovarianceStatusResult:
    safe_to_land: bool
    r_uncertainty_2sigma: float
    lambda_max_2d: float
    lambda_max_3d: float
    sigma_target_xy: float
    sigma_target_z: float
    sigma_drone_xy: float
    sigma_drone_z: float
    sigma_rel_xy: float
    sigma_rel_z: float
    rswitch_adaptive: float
    sliding_weight: float
    relative_dist_xy: float
    relative_dist_z: float
    measurement_age_s: float
    reject_reason: str = ""


class CovarianceGate:
    def __init__(
        self,
        pad_radius_m: float = 0.25,
        confidence_sigma: float = 2.0,
        max_measurement_age_s: float = 0.5,
        max_horizontal_distance_m: float = 5.0,
        rswitch_base_m: float = 1.5,
        rswitch_alpha: float = 1.0,
        sliding_beta: float = 2.0,
        default_drone_eph_m: float = 0.10,
        default_drone_epv_m: float = 0.15,
        rswitch_min_m: Optional[float] = None,
        rswitch_max_m: Optional[float] = None,
        sigma_ideal_m: float = 0.05,
        sigma_bad_m: float = 0.30,
        max_uncertainty_2sigma_m: Optional[float] = None,
        use_marker_relative: bool = False,
        attitude_sigma_rad: float = 0.0087,
    ):
        self.pad_radius = float(pad_radius_m)
        self.confidence_sigma = float(confidence_sigma)
        self.max_measurement_age = float(max_measurement_age_s)
        self.max_horizontal_dist = float(max_horizontal_distance_m)
        self.rswitch_base = float(rswitch_base_m)
        self.rswitch_alpha = float(rswitch_alpha)
        self.rswitch_min = None if rswitch_min_m is None else float(rswitch_min_m)
        self.rswitch_max = None if rswitch_max_m is None else float(rswitch_max_m)
        self.sigma_ideal = max(0.0, float(sigma_ideal_m))
        self.sigma_bad = max(self.sigma_ideal + 1e-6, float(sigma_bad_m))
        self.max_uncertainty_2sigma = (
            None if max_uncertainty_2sigma_m is None
            else max(0.0, float(max_uncertainty_2sigma_m))
        )
        self.sliding_beta = float(sliding_beta)
        self.default_drone_eph = float(default_drone_eph_m)
        self.default_drone_epv = float(default_drone_epv_m)
        self.use_marker_relative = bool(use_marker_relative)
        self.attitude_sigma_rad = float(attitude_sigma_rad)

    def extract_drone_covariance(self, odom_cov: Optional[np.ndarray]) -> np.ndarray:
        if odom_cov is not None and len(odom_cov) == 36:
            cov_3x3 = np.asarray(odom_cov, dtype=np.float64).reshape((6, 6))[:3, :3]
            if cov_3x3[0, 0] > 1e-6 and cov_3x3[1, 1] > 1e-6:
                return cov_3x3.copy()
        eph2 = self.default_drone_eph ** 2
        epv2 = self.default_drone_epv ** 2
        return np.diag([eph2, eph2, epv2])

    def extract_target_covariance(self, odom_cov: Optional[np.ndarray]) -> np.ndarray:
        if odom_cov is not None and len(odom_cov) == 36:
            cov_3x3 = np.asarray(odom_cov, dtype=np.float64).reshape((6, 6))[:3, :3]
            if cov_3x3[0, 0] > 1e-6 and cov_3x3[1, 1] > 1e-6:
                return cov_3x3.copy()
        return np.diag([1.0, 1.0, 1.0])

    def evaluate(
        self,
        drone_pos: np.ndarray,
        target_pos: np.ndarray,
        drone_cov_36: Optional[np.ndarray],
        target_cov_36: Optional[np.ndarray],
        is_detected: bool,
        measurement_age_s: float,
    ) -> CovarianceStatusResult:
        P_drone = self.extract_drone_covariance(drone_cov_36)
        P_target = self.extract_target_covariance(target_cov_36)

        dist_z = float(abs(drone_pos[2] - target_pos[2]))
        h = max(0.1, dist_z)

        if self.use_marker_relative:
            var_att = (h * self.attitude_sigma_rad) ** 2
            epv2 = float(P_drone[2, 2]) if P_drone[2, 2] > 1e-6 else self.default_drone_epv ** 2
            P_rel = P_target + np.diag([var_att, var_att, epv2])
        else:
            P_rel = P_drone + P_target

        lambda_max_2d = max(0.0, float(np.max(np.linalg.eigvalsh(P_rel[:2, :2]))))
        lambda_max_3d = max(0.0, float(np.max(np.linalg.eigvalsh(P_rel))))

        sigma_drone_xy = float(np.sqrt(max(0.0, 0.5 * (P_drone[0, 0] + P_drone[1, 1]))))
        sigma_drone_z = float(np.sqrt(max(0.0, P_drone[2, 2])))
        sigma_target_xy = float(np.sqrt(max(0.0, 0.5 * (P_target[0, 0] + P_target[1, 1]))))
        sigma_target_z = float(np.sqrt(max(0.0, P_target[2, 2])))
        sigma_rel_xy = float(np.sqrt(max(0.0, 0.5 * (P_rel[0, 0] + P_rel[1, 1]))))
        sigma_rel_z = float(np.sqrt(max(0.0, P_rel[2, 2])))

        r_unc = float(self.confidence_sigma * np.sqrt(lambda_max_2d))

        diff = drone_pos - target_pos
        dist_xy = float(np.linalg.norm(diff[:2]))
        dist_z = float(abs(diff[2]))

        reasons = []
        allowed_unc = max(self.pad_radius, 0.12 * dist_z)
        if r_unc > allowed_unc:
            reasons.append(
                f"Uncertainty radius {r_unc:.3f}m > allowed funnel {allowed_unc:.3f}m"
            )
        if (
            self.max_uncertainty_2sigma is not None
            and r_unc > self.max_uncertainty_2sigma
        ):
            reasons.append(
                f"Uncertainty radius {r_unc:.3f}m > glide limit "
                f"{self.max_uncertainty_2sigma:.3f}m (2-sigma)"
            )
        if measurement_age_s > self.max_measurement_age:
            reasons.append(
                f"Measurement age {measurement_age_s:.2f}s > timeout {self.max_measurement_age:.2f}s"
            )
        if not is_detected:
            reasons.append("Target is not actively detected by vision")
        if dist_xy > self.max_horizontal_dist:
            reasons.append(
                f"Horizontal distance {dist_xy:.2f}m > max approach {self.max_horizontal_dist:.2f}m"
            )

        safe_to_land = len(reasons) == 0
        reject_reason = "; ".join(reasons) if reasons else "OK"

        if self.rswitch_min is not None and self.rswitch_max is not None:
            sigma_xy = float(np.sqrt(lambda_max_2d))
            confidence = float(np.clip(
                (sigma_xy - self.sigma_ideal) / (self.sigma_bad - self.sigma_ideal),
                0.0,
                1.0,
            ))
            rswitch_adaptive = self.rswitch_max - confidence * (
                self.rswitch_max - self.rswitch_min
            )
        else:
            rswitch_adaptive = self.rswitch_base * (
                1.0 + self.rswitch_alpha * np.sqrt(lambda_max_2d)
            )
        rswitch_adaptive = float(max(0.0, rswitch_adaptive))
        sliding_weight = float(1.0 / (1.0 + self.sliding_beta * lambda_max_2d))

        return CovarianceStatusResult(
            safe_to_land=safe_to_land,
            r_uncertainty_2sigma=r_unc,
            lambda_max_2d=lambda_max_2d,
            lambda_max_3d=lambda_max_3d,
            sigma_target_xy=sigma_target_xy,
            sigma_target_z=sigma_target_z,
            sigma_drone_xy=sigma_drone_xy,
            sigma_drone_z=sigma_drone_z,
            sigma_rel_xy=sigma_rel_xy,
            sigma_rel_z=sigma_rel_z,
            rswitch_adaptive=rswitch_adaptive,
            sliding_weight=sliding_weight,
            relative_dist_xy=dist_xy,
            relative_dist_z=dist_z,
            measurement_age_s=float(measurement_age_s),
            reject_reason=reject_reason,
        )