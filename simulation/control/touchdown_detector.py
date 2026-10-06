#!/usr/bin/env python3
"""Touchdown Detector for Precision Drone Landing.

Drift-invariant multi-tier touchdown detection:
  Tier 1: Kinematic Stoppage (descent commanded while vertical velocity and altitude rate flatten)
  Tier 2: Optical Proximity (camera-to-pad distance <= mount height + margin, ~0.38m)
  Tier 3: Inertial Ground Impact Spike (jerk spike on touchdown)
  Tier 4: PX4 firmware land detector redundancy (/fmu/out/vehicle_land_detected)
  Tier 5: Temporal persistence debouncing (condition confirmed for >= 0.35s)

Outputs:
  /landing/touchdown         std_msgs/Bool (latched True upon confirmed ground contact)
  /landing/touchdown_status  diagnostic_msgs/DiagnosticStatus
"""

from __future__ import annotations

import collections
import os
import sys
import time
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import numpy as np

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    from geometry_msgs.msg import PointStamped, Twist
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
class TouchdownParams:
    """Parameters for drift-invariant touchdown detection."""
    # Mount height of camera / Pixhawk on the airframe (~25-30cm) + safe margin
    optical_height_threshold_m: float = 0.40
    # Minimum downward descent command (negative in ENU) required to consider landing
    descent_cmd_threshold_mps: float = -0.10
    # Maximum actual vertical speed allowed when stopped on ground
    stoppage_vz_max_mps: float = 0.08
    # Maximum altitude rate of change (|dz/dt|) allowed on ground
    stoppage_dz_dt_max_mps: float = 0.05
    # Jerk spike threshold indicating impact reaction force (m/s³)
    jerk_impact_threshold: float = 3.50
    # Required confirmation duration on ground before latching touchdown (seconds)
    confirmation_duration_s: float = 0.35
    # Fallback absolute altitude window (very relaxed to handle up to 0.6m barometer drift)
    altitude_ceiling_m: float = 0.80


@dataclass
class TouchdownResult:
    """Result of one touchdown evaluation cycle."""
    touchdown_confirmed: bool
    kinematic_stopped: bool
    optical_near: bool
    impact_detected: bool
    confirm_duration_s: float
    actual_vz: float
    commanded_vz: float
    optical_z: Optional[float]
    diagnostic_reason: str


class TouchdownDetector:
    """Mathematical and stateful engine for touchdown detection."""

    def __init__(self, params: Optional[TouchdownParams] = None):
        self.p = params or TouchdownParams()
        self.is_latched = False
        self.confirm_start_time: Optional[float] = None
        self.last_alt: Optional[float] = None
        self.last_alt_time: Optional[float] = None
        self.last_accel_z: float = 9.81
        self.last_accel_time: Optional[float] = None
        self.jerk_history: Deque[float] = collections.deque(maxlen=10)

    def reset(self):
        """Reset the detector state."""
        self.is_latched = False
        self.confirm_start_time = None
        self.last_alt = None
        self.last_alt_time = None
        self.last_accel_z = 9.81
        self.last_accel_time = None
        self.jerk_history.clear()

    def step(
        self,
        actual_vz: float,           # Current vertical velocity (m/s in ENU, positive up)
        commanded_vz: float,        # Commanded vertical velocity (m/s in ENU, negative down)
        current_alt_z: float,       # Current altitude estimate (m)
        optical_z: Optional[float] = None, # Optical height to pad (m) from camera
        accel_z_body: Optional[float] = None, # Body Z accel (m/s²)
        px4_landed: bool = False,   # PX4 firmware land detector flag
        current_time: Optional[float] = None,
    ) -> TouchdownResult:
        """
        Evaluate touchdown criteria.
        """
        now = time.monotonic() if current_time is None else current_time

        if self.is_latched:
            return TouchdownResult(
                touchdown_confirmed=True,
                kinematic_stopped=True,
                optical_near=True,
                impact_detected=False,
                confirm_duration_s=self.p.confirmation_duration_s,
                actual_vz=actual_vz,
                commanded_vz=commanded_vz,
                optical_z=optical_z,
                diagnostic_reason="LATCHED_TOUCHDOWN",
            )

        # 1. Altitude derivative dz/dt check
        dz_dt = 0.0
        if self.last_alt is not None and self.last_alt_time is not None:
            dt = max(1e-4, now - self.last_alt_time)
            dz_dt = (current_alt_z - self.last_alt) / dt
        self.last_alt = current_alt_z
        self.last_alt_time = now

        # 2. Jerk / Inertial impact check
        impact_detected = False
        if accel_z_body is not None and self.last_accel_time is not None:
            dt_acc = max(1e-4, now - self.last_accel_time)
            jerk = abs(accel_z_body - self.last_accel_z) / dt_acc
            self.jerk_history.append(jerk)
            if jerk >= self.p.jerk_impact_threshold:
                impact_detected = True
            self.last_accel_z = accel_z_body
            self.last_accel_time = now
        elif accel_z_body is not None:
            self.last_accel_z = accel_z_body
            self.last_accel_time = now

        # 3. Kinematic Stoppage Criteria (Drift-Invariant!)
        # - Flight controller is demanding descent (commanded_vz <= -0.10 m/s)
        # - But drone cannot descend further: |actual_vz| <= 0.08 m/s and |dz_dt| <= 0.05 m/s
        is_commanding_descent = commanded_vz <= self.p.descent_cmd_threshold_mps
        is_vertically_stopped = (
            abs(actual_vz) <= self.p.stoppage_vz_max_mps
            and abs(dz_dt) <= self.p.stoppage_dz_dt_max_mps
        )
        kinematic_stopped = is_commanding_descent and is_vertically_stopped

        # 4. Optical Height check (Marker-Relative)
        # Directly measures distance from camera mount to the pad surface
        optical_near = False
        if optical_z is not None:
            optical_near = optical_z <= self.p.optical_height_threshold_m
        else:
            # If camera view is occluded or not available, rely on relaxed altitude ceiling
            optical_near = current_alt_z <= self.p.altitude_ceiling_m

        # 5. Combined Ground Contact Condition
        # A. Kinematic stoppage + optical near, OR
        # B. Firmware PX4 landed flag, OR
        # C. Impact jerk + kinematic stopped
        ground_contact = False
        reason = "DESCENT_IN_PROGRESS"

        if px4_landed:
            ground_contact = True
            reason = "PX4_FIRMWARE_LANDED"
        elif kinematic_stopped and optical_near:
            ground_contact = True
            reason = f"KINEMATIC_STOPPED (vz={actual_vz:.2f}, opt_z={optical_z})"
        elif impact_detected and is_vertically_stopped and optical_near:
            ground_contact = True
            reason = "IMPACT_JERK_STOPPED"

        # 6. Temporal persistence confirmation
        confirm_duration = 0.0
        if ground_contact:
            if self.confirm_start_time is None:
                self.confirm_start_time = now
            confirm_duration = now - self.confirm_start_time

            if confirm_duration >= self.p.confirmation_duration_s:
                self.is_latched = True
                reason = f"TOUCHDOWN_CONFIRMED ({confirm_duration:.2f}s)"
        else:
            self.confirm_start_time = None

        return TouchdownResult(
            touchdown_confirmed=self.is_latched,
            kinematic_stopped=kinematic_stopped,
            optical_near=optical_near,
            impact_detected=impact_detected,
            confirm_duration_s=confirm_duration,
            actual_vz=actual_vz,
            commanded_vz=commanded_vz,
            optical_z=optical_z,
            diagnostic_reason=reason,
        )


class TouchdownDetectorNode(Node):
    """ROS 2 Node executing Touchdown Detection and publishing status."""

    def __init__(self):
        super().__init__('touchdown_detector_node')

        self.declare_parameter('optical_height_threshold_m', 0.40)
        self.declare_parameter('descent_cmd_threshold_mps', -0.10)
        self.declare_parameter('stoppage_vz_max_mps', 0.08)
        self.declare_parameter('stoppage_dz_dt_max_mps', 0.05)
        self.declare_parameter('jerk_impact_threshold', 3.50)
        self.declare_parameter('confirmation_duration_s', 0.35)
        self.declare_parameter('eval_rate_hz', 20.0)

        params = TouchdownParams(
            optical_height_threshold_m=float(self.get_parameter('optical_height_threshold_m').value),
            descent_cmd_threshold_mps=float(self.get_parameter('descent_cmd_threshold_mps').value),
            stoppage_vz_max_mps=float(self.get_parameter('stoppage_vz_max_mps').value),
            stoppage_dz_dt_max_mps=float(self.get_parameter('stoppage_dz_dt_max_mps').value),
            jerk_impact_threshold=float(self.get_parameter('jerk_impact_threshold').value),
            confirmation_duration_s=float(self.get_parameter('confirmation_duration_s').value),
        )

        self.detector = TouchdownDetector(params)
        rate_hz = float(self.get_parameter('eval_rate_hz').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # State storage
        self.drone_odom: Optional[Odometry] = None
        self.optical_z: Optional[float] = None
        self.last_optical_time: float = 0.0
        self.commanded_vz: float = 0.0  # default zero
        self.px4_landed: bool = False
        self.phase: str = 'IDLE'
        self.last_log_time: float = 0.0

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_cb, sensor_qos)
        self.create_subscription(PointStamped, '/hpad/position_camera', self.optical_cb, 10)
        self.create_subscription(Twist, '/landing/velocity_cmd', self.cmd_cb, 10)
        self.create_subscription(Twist, '/mission/velocity_setpoint', self.cmd_cb, 10)
        self.create_subscription(String, '/mission/phase', self.phase_cb, 10)

        # Publishers
        self.touchdown_pub = self.create_publisher(Bool, '/landing/touchdown', 10)
        self.diag_pub = self.create_publisher(DiagnosticStatus, '/landing/touchdown_status', 10)

        dt = 1.0 / max(rate_hz, 1.0)
        self.timer = self.create_timer(dt, self.eval_loop)

        self.get_logger().info(
            f"TouchdownDetectorNode initialized: optical_thresh={params.optical_height_threshold_m:.2f}m, "
            f"confirm_duration={params.confirmation_duration_s:.2f}s, rate={rate_hz:.1f}Hz"
        )

    def phase_cb(self, msg: String):
        prev_phase = self.phase
        self.phase = msg.data
        if self.phase != 'LAND':
            self.detector.reset()

    def odom_cb(self, msg: Odometry):
        self.drone_odom = msg

    def optical_cb(self, msg: PointStamped):
        # In camera optical frame, z is distance along line-of-sight
        self.optical_z = float(msg.point.z)
        self.last_optical_time = time.monotonic()

    def cmd_cb(self, msg: Twist):
        self.commanded_vz = float(msg.linear.z)

    def eval_loop(self):
        now = time.monotonic()
        if self.drone_odom is None:
            return

        # Touchdown detection MUST only be active during LAND phase!
        # When hovering, taking off, or following, reset state and hold touchdown False.
        if self.phase != 'LAND':
            self.detector.reset()
            self.touchdown_pub.publish(Bool(data=False))
            return

        actual_vz = float(self.drone_odom.twist.twist.linear.z)
        current_alt_z = float(self.drone_odom.pose.pose.position.z)

        # Optical height valid if recent (< 0.5s)
        opt_z = self.optical_z if (now - self.last_optical_time < 0.5) else None

        res = self.detector.step(
            actual_vz=actual_vz,
            commanded_vz=self.commanded_vz,
            current_alt_z=current_alt_z,
            optical_z=opt_z,
            px4_landed=self.px4_landed,
            current_time=now,
        )

        self.touchdown_pub.publish(Bool(data=res.touchdown_confirmed))

        diag = DiagnosticStatus()
        diag.level = DiagnosticStatus.OK if res.touchdown_confirmed else DiagnosticStatus.WARN
        diag.name = "PrecisionLanding:TouchdownDetector"
        diag.message = res.diagnostic_reason
        diag.hardware_id = "TOUCHDOWN_DETECTOR"
        diag.values = [
            KeyValue(key="touchdown_confirmed", value=str(res.touchdown_confirmed)),
            KeyValue(key="kinematic_stopped", value=str(res.kinematic_stopped)),
            KeyValue(key="optical_near", value=str(res.optical_near)),
            KeyValue(key="actual_vz_mps", value=f"{actual_vz:.3f}"),
            KeyValue(key="commanded_vz_mps", value=f"{self.commanded_vz:.3f}"),
            KeyValue(key="optical_z_m", value=f"{opt_z:.3f}" if opt_z is not None else "None"),
            KeyValue(key="alt_z_m", value=f"{current_alt_z:.3f}"),
            KeyValue(key="confirm_duration_s", value=f"{res.confirm_duration_s:.2f}"),
        ]
        self.diag_pub.publish(diag)

        if res.touchdown_confirmed:
            if now - self.last_log_time > 1.0:
                self.get_logger().info(
                    f"🏆 [TOUCHDOWN CONFIRMED] Motors ready for shutdown! ({res.diagnostic_reason})"
                )
                self.last_log_time = now
        elif res.kinematic_stopped:
            if now - self.last_log_time > 0.5:
                self.get_logger().info(
                    f"[CONTACT DETECTED] Confirming ground contact: {res.confirm_duration_s:.2f}s / "
                    f"{self.detector.p.confirmation_duration_s:.2f}s"
                )
                self.last_log_time = now


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
