#!/usr/bin/env python3
import time
from typing import Optional

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String

from landing_guidance.covariance_logic import CovarianceGate


class CovarianceGateNode(Node):
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

        def p(name):
            return self.get_parameter(name).value

        confidence_sigma = float(p('confidence_sigma'))
        self.gate = CovarianceGate(
            pad_radius_m=float(p('pad_radius_m')),
            confidence_sigma=confidence_sigma,
            max_measurement_age_s=float(p('max_measurement_age_s')),
            max_horizontal_distance_m=float(p('max_horizontal_distance_m')),
            rswitch_base_m=float(p('rswitch_base_m')),
            rswitch_alpha=float(p('rswitch_alpha')),
            rswitch_min_m=float(p('r_switch_min_m')),
            rswitch_max_m=float(p('r_switch_max_m')),
            sigma_ideal_m=float(p('sigma_ideal_m')),
            sigma_bad_m=float(p('sigma_bad_m')),
            max_uncertainty_2sigma_m=confidence_sigma * float(p('sigma_max_continue_glide_m')),
            sliding_beta=float(p('sliding_beta')),
            default_drone_eph_m=float(p('default_drone_eph_m')),
            default_drone_epv_m=float(p('default_drone_epv_m')),
            use_marker_relative=bool(p('use_marker_relative')),
            attitude_sigma_rad=float(p('attitude_sigma_rad')),
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.drone_odom: Optional[Odometry] = None
        self.target_odom: Optional[Odometry] = None
        self.ekf_tracking_mode = 'EXPIRED'
        self.last_log_time = 0.0

        self.create_subscription(Odometry, '/odom', self._drone_cb, sensor_qos)
        self.create_subscription(Odometry, '/hpad/state_filtered', self._target_cb, 10)
        self.create_subscription(String, '/ekf/tracking_mode', self._mode_cb, 10)

        self.safe_pub = self.create_publisher(Bool, '/landing/safe_to_land', 10)
        self.unc_pub = self.create_publisher(Float64, '/landing/uncertainty_radius', 10)
        self.rswitch_pub = self.create_publisher(Float64, '/landing/rswitch_adaptive', 10)
        self.weight_pub = self.create_publisher(Float64, '/landing/sliding_weight', 10)
        self.diag_pub = self.create_publisher(DiagnosticStatus, '/landing/covariance_status', 10)

        rate_hz = float(p('eval_rate_hz'))
        self.create_timer(1.0 / max(rate_hz, 1.0), self._eval)

    def _drone_cb(self, msg: Odometry):
        self.drone_odom = msg

    def _target_cb(self, msg: Odometry):
        self.target_odom = msg

    def _mode_cb(self, msg: String):
        self.ekf_tracking_mode = msg.data

    def _eval(self):
        now = time.monotonic()
        fresh = self.ekf_tracking_mode == 'TRACKING'
        age = 0.0 if fresh else self.gate.max_measurement_age + 1.0

        if self.drone_odom is None or self.target_odom is None:
            self.safe_pub.publish(Bool(data=False))
            self.unc_pub.publish(Float64(data=999.0))
            return

        dp = self.drone_odom.pose.pose.position
        tp = self.target_odom.pose.pose.position
        res = self.gate.evaluate(
            drone_pos=np.array([dp.x, dp.y, dp.z], dtype=np.float64),
            target_pos=np.array([tp.x, tp.y, tp.z], dtype=np.float64),
            drone_cov_36=np.array(self.drone_odom.pose.covariance, dtype=np.float64),
            target_cov_36=np.array(self.target_odom.pose.covariance, dtype=np.float64),
            is_detected=fresh,
            measurement_age_s=age,
        )

        self.safe_pub.publish(Bool(data=res.safe_to_land))
        self.unc_pub.publish(Float64(data=res.r_uncertainty_2sigma))
        self.rswitch_pub.publish(Float64(data=res.rswitch_adaptive))
        self.weight_pub.publish(Float64(data=res.sliding_weight))

        diag = DiagnosticStatus()
        diag.level = DiagnosticStatus.OK if res.safe_to_land else DiagnosticStatus.WARN
        diag.name = 'PrecisionLanding:CovarianceGate'
        diag.message = res.reject_reason
        diag.hardware_id = 'PX4_EKF2_AND_TARGET_EKF'
        diag.values = [
            KeyValue(key='safe_to_land', value=str(res.safe_to_land)),
            KeyValue(key='r_uncertainty_2sigma_m', value=f'{res.r_uncertainty_2sigma:.4f}'),
            KeyValue(key='sigma_rel_xy_m', value=f'{res.sigma_rel_xy:.4f}'),
            KeyValue(key='sigma_rel_z_m', value=f'{res.sigma_rel_z:.4f}'),
            KeyValue(key='rswitch_adaptive_m', value=f'{res.rswitch_adaptive:.3f}'),
            KeyValue(key='sliding_weight', value=f'{res.sliding_weight:.4f}'),
            KeyValue(key='rel_dist_xy_m', value=f'{res.relative_dist_xy:.3f}'),
            KeyValue(key='rel_dist_z_m', value=f'{res.relative_dist_z:.3f}'),
            KeyValue(key='ekf_tracking_mode', value=self.ekf_tracking_mode),
        ]
        self.diag_pub.publish(diag)

        if now - self.last_log_time > 2.0:
            tag = '[SAFE_TO_LAND]' if res.safe_to_land else '[GATE_HOLD]'
            self.get_logger().info(
                f'{tag} r_unc={res.r_uncertainty_2sigma:.3f}m '
                f'DistXY={res.relative_dist_xy:.2f}m {res.reject_reason}'
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