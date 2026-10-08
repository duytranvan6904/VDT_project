#!/usr/bin/env python3
"""Covariance Gating & Safety Evaluator for Precision Landing.

Extracts target state covariance from the Target EKF and drone position covariance
from vehicle odometry, evaluates 3D/2D relative uncertainty ellipses against the
physical landing pad geometry, and decides whether it is safe to initiate or
maintain the descent phase.

Key outputs:
  /landing/safe_to_land        std_msgs/Bool
  /landing/uncertainty_radius  std_msgs/Float64 (meters, 2-sigma)
  /landing/rswitch_adaptive    std_msgs/Float64 (meters)
  /landing/sliding_weight      std_msgs/Float64 (dimensionless weight)
  /landing/covariance_status   diagnostic_msgs/DiagnosticStatus
"""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Point
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Float64, String
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False
    Node = object

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


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
    """Core mathematical engine for relative covariance gating."""

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
        """Extract 3x3 position covariance matrix from 36-element odom covariance."""
        if odom_cov is not None and len(odom_cov) == 36:
            cov_3x3 = odom_cov.reshape((6, 6))[:3, :3]
            # Verify positive variance
            if cov_3x3[0, 0] > 1e-6 and cov_3x3[1, 1] > 1e-6:
                return cov_3x3.copy()

        # Fallback to isotropic model using eph/epv
        eph2 = self.default_drone_eph ** 2
        epv2 = self.default_drone_epv ** 2
        return np.diag([eph2, eph2, epv2])

    def extract_target_covariance(self, odom_cov: Optional[np.ndarray]) -> np.ndarray:
        """Extract 3x3 position covariance matrix from Target EKF Odometry."""
        if odom_cov is not None and len(odom_cov) == 36:
            cov_3x3 = odom_cov.reshape((6, 6))[:3, :3]
            if cov_3x3[0, 0] > 1e-6 and cov_3x3[1, 1] > 1e-6:
                return cov_3x3.copy()

        # If uninitialized / degenerate
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
        """
        Evaluate full relative uncertainty and determine safe_to_land.
        """
        P_drone = self.extract_drone_covariance(drone_cov_36)
        P_target = self.extract_target_covariance(target_cov_36)

        # 1. Total relative covariance
        # In Marker-Relative mode (Section 3.5), global GPS position error (eph) is eliminated.
        # However, relative uncertainty must include:
        # - Target tracking covariance from vision / EKF (P_target)
        # - Drone attitude uncertainty from PX4 EKF projected over distance: var_att = (h * sigma_theta)^2
        # - Drone altitude uncertainty from PX4 EKF: epv^2
        dist_z = float(abs(drone_pos[2] - target_pos[2]))
        h = max(0.1, dist_z)

        if self.use_marker_relative:
            var_att = (h * self.attitude_sigma_rad) ** 2
            epv2 = float(P_drone[2, 2]) if P_drone is not None and P_drone[2, 2] > 1e-6 else self.default_drone_epv ** 2
            P_ekf_att_alt = np.diag([var_att, var_att, epv2])
            P_rel = P_target + P_ekf_att_alt
        else:
            P_rel = P_drone + P_target

        # 2. Horizontal 2D relative covariance
        P_rel_2d = P_rel[:2, :2]
        eigvals_2d = np.linalg.eigvalsh(P_rel_2d)
        lambda_max_2d = max(0.0, float(np.max(eigvals_2d)))

        # 3. 3D relative covariance
        eigvals_3d = np.linalg.eigvalsh(P_rel)
        lambda_max_3d = max(0.0, float(np.max(eigvals_3d)))

        # 4. Standard deviations
        sigma_drone_xy = float(np.sqrt(max(0.0, 0.5 * (P_drone[0, 0] + P_drone[1, 1]))))
        sigma_drone_z = float(np.sqrt(max(0.0, P_drone[2, 2])))

        sigma_target_xy = float(np.sqrt(max(0.0, 0.5 * (P_target[0, 0] + P_target[1, 1]))))
        sigma_target_z = float(np.sqrt(max(0.0, P_target[2, 2])))

        sigma_rel_xy = float(np.sqrt(max(0.0, 0.5 * (P_rel[0, 0] + P_rel[1, 1]))))
        sigma_rel_z = float(np.sqrt(max(0.0, P_rel[2, 2])))

        # 5. Uncertainty radius (2-sigma error ellipse semi-major axis)
        r_unc = self.confidence_sigma * np.sqrt(lambda_max_2d)

        # 6. Relative geometric distance
        diff = drone_pos - target_pos
        dist_xy = float(np.linalg.norm(diff[:2]))
        dist_z = float(abs(diff[2]))

        # 7. Safety conditions
        # Conical approach funnel: at approach altitudes, uncertainty is permitted to be
        # up to funnel boundary (max(pad_radius, 0.12 * dist_z)), narrowing to 0.25m at the pad.
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
        reject_reason = "; ".join(reasons) if not safe_to_land else "OK"

        # 8. Adaptive phase transition distance and sliding weight
        if self.rswitch_min is not None and self.rswitch_max is not None:
            # The glide starts farther away only when relative uncertainty is
            # low. Poor covariance moves the handover inward, never outward.
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
            # Keep the old constructor behavior for callers that still pass
            # rswitch_base_m/rswitch_alpha without an explicit range.
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


class CovarianceGateNode(Node):
    """ROS 2 Node interfacing CovarianceGate with system topics."""

    def __init__(self):
        super().__init__('covariance_gate_node')

        self.declare_parameter('pad_radius_m', 0.25)
        self.declare_parameter('confidence_sigma', 2.0)
        self.declare_parameter('max_measurement_age_s', 0.5)
        self.declare_parameter('max_horizontal_distance_m', 5.0)
        self.declare_parameter('rswitch_base_m', 1.5)
        self.declare_parameter('rswitch_alpha', 1.0)
        self.declare_parameter('r_switch_min_m', 3.0)
        self.declare_parameter('r_switch_max_m', 15.0)
        self.declare_parameter('sigma_ideal_m', 0.05)
        self.declare_parameter('sigma_bad_m', 0.30)
        self.declare_parameter('sigma_max_continue_glide_m', 0.15)
        self.declare_parameter('sliding_beta', 2.0)
        self.declare_parameter('default_drone_eph_m', 0.10)
        self.declare_parameter('default_drone_epv_m', 0.15)
        self.declare_parameter('use_marker_relative', True)
        self.declare_parameter('attitude_sigma_rad', 0.0087)
        self.declare_parameter('eval_rate_hz', 20.0)

        pad_radius = float(self.get_parameter('pad_radius_m').value)
        confidence_sigma = float(self.get_parameter('confidence_sigma').value)
        max_age = float(self.get_parameter('max_measurement_age_s').value)
        max_dist = float(self.get_parameter('max_horizontal_distance_m').value)
        rswitch_base = float(self.get_parameter('rswitch_base_m').value)
        rswitch_alpha = float(self.get_parameter('rswitch_alpha').value)
        rswitch_min = float(self.get_parameter('r_switch_min_m').value)
        rswitch_max = float(self.get_parameter('r_switch_max_m').value)
        sigma_ideal = float(self.get_parameter('sigma_ideal_m').value)
        sigma_bad = float(self.get_parameter('sigma_bad_m').value)
        sigma_max_continue = float(
            self.get_parameter('sigma_max_continue_glide_m').value
        )
        sliding_beta = float(self.get_parameter('sliding_beta').value)
        drone_eph = float(self.get_parameter('default_drone_eph_m').value)
        drone_epv = float(self.get_parameter('default_drone_epv_m').value)
        use_marker_relative = bool(self.get_parameter('use_marker_relative').value)
        attitude_sigma = float(self.get_parameter('attitude_sigma_rad').value)
        rate_hz = float(self.get_parameter('eval_rate_hz').value)

        self.gate = CovarianceGate(
            pad_radius_m=pad_radius,
            confidence_sigma=confidence_sigma,
            max_measurement_age_s=max_age,
            max_horizontal_distance_m=max_dist,
            rswitch_base_m=rswitch_base,
            rswitch_alpha=rswitch_alpha,
            rswitch_min_m=rswitch_min,
            rswitch_max_m=rswitch_max,
            sigma_ideal_m=sigma_ideal,
            sigma_bad_m=sigma_bad,
            max_uncertainty_2sigma_m=confidence_sigma * sigma_max_continue,
            sliding_beta=sliding_beta,
            default_drone_eph_m=drone_eph,
            default_drone_epv_m=drone_epv,
            use_marker_relative=use_marker_relative,
            attitude_sigma_rad=attitude_sigma,
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # State storage
        self.latest_drone_odom: Optional[Odometry] = None
        self.latest_target_odom: Optional[Odometry] = None
        self.tracking_mode: str = "LOST"
        self.ekf_tracking_mode: str = "EXPIRED"
        self.last_log_time: float = 0.0

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.drone_odom_cb, sensor_qos)
        self.create_subscription(Odometry, '/ekf/target_state', self.target_odom_cb, 10)
        self.create_subscription(String, '/hpad/tracking_mode', self.tracking_mode_cb, 10)
        self.create_subscription(String, '/ekf/tracking_mode', self.ekf_tracking_mode_cb, 10)

        # Publishers
        self.safe_pub = self.create_publisher(Bool, '/landing/safe_to_land', 10)
        self.uncertainty_pub = self.create_publisher(Float64, '/landing/uncertainty_radius', 10)
        self.rswitch_pub = self.create_publisher(Float64, '/landing/rswitch_adaptive', 10)
        self.sliding_weight_pub = self.create_publisher(Float64, '/landing/sliding_weight', 10)
        self.diag_pub = self.create_publisher(DiagnosticStatus, '/landing/covariance_status', 10)

        dt = 1.0 / max(rate_hz, 1.0)
        self.eval_timer = self.create_timer(dt, self.eval_loop)

        self.get_logger().info(
            f"CovarianceGateNode initialized: pad_radius={pad_radius:.2f}m, "
            f"confidence_sigma={confidence_sigma:.1f}, rate={rate_hz:.1f}Hz"
        )

    def drone_odom_cb(self, msg: Odometry):
        self.latest_drone_odom = msg

    def target_odom_cb(self, msg: Odometry):
        self.latest_target_odom = msg

    def tracking_mode_cb(self, msg: String):
        self.tracking_mode = msg.data

    def ekf_tracking_mode_cb(self, msg: String):
        self.ekf_tracking_mode = msg.data

    def eval_loop(self):
        now = time.monotonic()
        # Use the EKF's accepted-measurement mode, not the unsynchronized raw
        # Bool topic. A negative frame can otherwise race a valid pose and
        # close the landing gate at the camera's normal frame rate.
        measurement_fresh = self.ekf_tracking_mode == "TRACKING"
        meas_age = 0.0 if measurement_fresh else self.gate.max_measurement_age + 1.0

        if self.latest_drone_odom is None or self.latest_target_odom is None:
            # Insufficient inputs: output safe=False
            self.safe_pub.publish(Bool(data=False))
            self.uncertainty_pub.publish(Float64(data=999.0))
            return

        drone_p = self.latest_drone_odom.pose.pose.position
        target_p = self.latest_target_odom.pose.pose.position
        drone_pos = np.array([drone_p.x, drone_p.y, drone_p.z], dtype=np.float64)
        target_pos = np.array([target_p.x, target_p.y, target_p.z], dtype=np.float64)

        drone_cov = np.array(self.latest_drone_odom.pose.covariance, dtype=np.float64)
        target_cov = np.array(self.latest_target_odom.pose.covariance, dtype=np.float64)

        # Evaluate gating
        res = self.gate.evaluate(
            drone_pos=drone_pos,
            target_pos=target_pos,
            drone_cov_36=drone_cov,
            target_cov_36=target_cov,
            is_detected=measurement_fresh,
            measurement_age_s=meas_age,
        )

        # Publish topics
        self.safe_pub.publish(Bool(data=res.safe_to_land))
        self.uncertainty_pub.publish(Float64(data=res.r_uncertainty_2sigma))
        self.rswitch_pub.publish(Float64(data=res.rswitch_adaptive))
        self.sliding_weight_pub.publish(Float64(data=res.sliding_weight))

        # Diagnostic message
        diag = DiagnosticStatus()
        diag.level = DiagnosticStatus.OK if res.safe_to_land else DiagnosticStatus.WARN
        diag.name = "PrecisionLanding:CovarianceGate"
        diag.message = res.reject_reason
        diag.hardware_id = "PX4_EKF2_AND_TARGET_EKF"

        diag.values = [
            KeyValue(key="safe_to_land", value=str(res.safe_to_land)),
            KeyValue(key="r_uncertainty_2sigma_m", value=f"{res.r_uncertainty_2sigma:.4f}"),
            KeyValue(key="lambda_max_2d", value=f"{res.lambda_max_2d:.6f}"),
            KeyValue(key="sigma_target_xy_m", value=f"{res.sigma_target_xy:.4f}"),
            KeyValue(key="sigma_target_z_m", value=f"{res.sigma_target_z:.4f}"),
            KeyValue(key="sigma_drone_xy_m", value=f"{res.sigma_drone_xy:.4f}"),
            KeyValue(key="sigma_drone_z_m", value=f"{res.sigma_drone_z:.4f}"),
            KeyValue(key="sigma_rel_xy_m", value=f"{res.sigma_rel_xy:.4f}"),
            KeyValue(key="sigma_rel_z_m", value=f"{res.sigma_rel_z:.4f}"),
            KeyValue(key="rswitch_adaptive_m", value=f"{res.rswitch_adaptive:.3f}"),
            KeyValue(key="sliding_weight", value=f"{res.sliding_weight:.4f}"),
            KeyValue(key="rel_dist_xy_m", value=f"{res.relative_dist_xy:.3f}"),
            KeyValue(key="rel_dist_z_m", value=f"{res.relative_dist_z:.3f}"),
            KeyValue(key="measurement_age_s", value=f"{res.measurement_age_s:.2f}"),
            KeyValue(key="tracking_mode", value=self.tracking_mode),
            KeyValue(key="ekf_tracking_mode", value=self.ekf_tracking_mode),
        ]
        self.diag_pub.publish(diag)

        if now - self.last_log_time > 2.0:
            status_tag = "[SAFE_TO_LAND]" if res.safe_to_land else "[GATE_HOLD]"
            self.get_logger().info(
                f"{status_tag} r_unc={res.r_uncertainty_2sigma:.3f}m (limit {self.gate.pad_radius:.2f}m) | "
                f"DistXY={res.relative_dist_xy:.2f}m | Mode={self.tracking_mode} | Msg: {res.reject_reason}"
            )
            self.last_log_time = now


def main():
    rclpy.init()
    node = CovarianceGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
