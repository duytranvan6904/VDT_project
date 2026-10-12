#!/usr/bin/env python3
import math
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import TrajectorySetpoint, VehicleLandDetected
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String

from landing_guidance.touchdown_logic import TouchdownDetector, TouchdownParams


class TouchdownDetectorNode(Node):
    def __init__(self):
        super().__init__('touchdown_detector_node')

        self.declare_parameter('optical_height_threshold_m', 0.40)
        self.declare_parameter('descent_cmd_threshold_mps', -0.10)
        self.declare_parameter('stoppage_vz_max_mps', 0.08)
        self.declare_parameter('stoppage_dz_dt_max_mps', 0.05)
        self.declare_parameter('jerk_impact_threshold', 3.50)
        self.declare_parameter('confirmation_duration_s', 0.35)
        self.declare_parameter('altitude_ceiling_m', 0.80)
        self.declare_parameter('min_descent_time_s', 1.0)
        self.declare_parameter('seen_descending_vz_mps', -0.10)
        self.declare_parameter('seen_descending_hold_s', 0.30)
        self.declare_parameter('eval_rate_hz', 20.0)

        def p(name):
            return self.get_parameter(name).value

        self.detector = TouchdownDetector(TouchdownParams(
            optical_height_threshold_m=float(p('optical_height_threshold_m')),
            descent_cmd_threshold_mps=float(p('descent_cmd_threshold_mps')),
            stoppage_vz_max_mps=float(p('stoppage_vz_max_mps')),
            stoppage_dz_dt_max_mps=float(p('stoppage_dz_dt_max_mps')),
            jerk_impact_threshold=float(p('jerk_impact_threshold')),
            confirmation_duration_s=float(p('confirmation_duration_s')),
            altitude_ceiling_m=float(p('altitude_ceiling_m')),
            min_descent_time_s=float(p('min_descent_time_s')),
            seen_descending_vz_mps=float(p('seen_descending_vz_mps')),
            seen_descending_hold_s=float(p('seen_descending_hold_s')),
        ))

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.odom: Optional[Odometry] = None
        self.t_odom = 0.0
        self.optical_z: Optional[float] = None
        self.t_optical = 0.0
        self.commanded_vz = 0.0
        self.t_sp = 0.0
        self.px4_landed = False
        self.phase = 'IDLE'
        self.t_log = 0.0

        self.create_subscription(Odometry, '/odom', self._odom_cb, sensor_qos)
        self.create_subscription(PointStamped, '/hpad/position_camera', self._optical_cb, 10)
        self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
            self._sp_cb, qos_profile_sensor_data,
        )
        self.create_subscription(String, '/mission/phase', self._phase_cb, 10)
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._land_cb, qos_profile_sensor_data,
        )

        self.td_pub = self.create_publisher(Bool, '/landing/touchdown', 10)
        self.diag_pub = self.create_publisher(DiagnosticStatus, '/landing/touchdown_status', 10)

        rate_hz = float(p('eval_rate_hz'))
        self.create_timer(1.0 / max(rate_hz, 1.0), self._eval)

    def _odom_cb(self, msg):
        self.odom = msg
        self.t_odom = time.monotonic()

    def _optical_cb(self, msg):
        self.optical_z = float(msg.point.z)
        self.t_optical = time.monotonic()

    def _sp_cb(self, msg):
        vz = float(msg.velocity[2])
        self.commanded_vz = -vz if math.isfinite(vz) else 0.0
        self.t_sp = time.monotonic()

    def _land_cb(self, msg):
        self.px4_landed = bool(msg.landed)

    def _phase_cb(self, msg):
        self.phase = msg.data
        if self.phase not in ('LAND', 'APPROACH'):
            self.detector.reset()

    def _publish_false(self):
        self.detector.reset()
        self.td_pub.publish(Bool(data=False))

    def _eval(self):
        now = time.monotonic()
        if self.odom is None or now - self.t_odom > 0.5:
            self._publish_false()
            return
        if self.phase not in ('LAND', 'APPROACH'):
            self._publish_false()
            return

        alt = float(self.odom.pose.pose.position.z)
        if self.phase == 'APPROACH' and alt > self.detector.p.altitude_ceiling_m:
            self._publish_false()
            return

        opt = self.optical_z if now - self.t_optical < 0.5 else None
        res = self.detector.step(
            actual_vz=float(self.odom.twist.twist.linear.z),
            commanded_vz=self.commanded_vz if now - self.t_sp < 0.5 else 0.0,
            current_alt_z=alt,
            optical_z=opt,
            px4_landed=self.px4_landed,
            current_time=now,
        )
        self.td_pub.publish(Bool(data=res.touchdown_confirmed))

        diag = DiagnosticStatus()
        diag.level = DiagnosticStatus.OK if res.touchdown_confirmed else DiagnosticStatus.WARN
        diag.name = 'PrecisionLanding:TouchdownDetector'
        diag.message = res.diagnostic_reason
        diag.hardware_id = 'TOUCHDOWN_DETECTOR'
        diag.values = [
            KeyValue(key='kinematic_stopped', value=str(res.kinematic_stopped)),
            KeyValue(key='optical_near', value=str(res.optical_near)),
            KeyValue(key='seen_descending', value=str(res.seen_descending)),
            KeyValue(key='descent_time_s', value=f'{res.descent_time_s:.2f}'),
            KeyValue(key='actual_vz_mps', value=f'{res.actual_vz:.3f}'),
            KeyValue(key='commanded_vz_mps', value=f'{res.commanded_vz:.3f}'),
            KeyValue(key='optical_z_m', value=f'{opt:.3f}' if opt is not None else 'None'),
            KeyValue(key='alt_z_m', value=f'{alt:.3f}'),
            KeyValue(key='confirm_s', value=f'{res.confirm_duration_s:.2f}'),
        ]
        self.diag_pub.publish(diag)

        if res.touchdown_confirmed and now - self.t_log > 1.0:
            self.get_logger().info(f'[TOUCHDOWN CONFIRMED] {res.diagnostic_reason}')
            self.t_log = now


def main():
    rclpy.init()
    node = TouchdownDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()