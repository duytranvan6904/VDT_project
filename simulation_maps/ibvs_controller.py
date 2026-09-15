#!/usr/bin/env python3
"""IBVS Controller — Image-Based Visual Servoing cho Gimbal Pitch + Drone Yaw.

Điều khiển theo pixel error giữa vị trí ArUco marker trên ảnh và tâm ảnh.
Behavior thay đổi theo mission phase (SEARCH / FOLLOW / APPROACH / LAND).

Luật điều khiển (từ IBVS_Implementation_Guide.md):
  Gimbal Pitch: θ += K_pitch * (v - v0) / f_y * dt
  Drone Yaw:    ψ += K_yaw   * (u - u0) / f_x * dt

ROS 2 interface:
  Subscribe: /hpad/bbox, /hpad/detected, /ekf/tracking_mode,
             /mission/phase, /odom
  Publish:   /ibvs/gimbal_pitch (Float64 → bridge → Gazebo gimbal),
             /ibvs/yaw_cmd (Float64)
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String
from vision_msgs.msg import BoundingBox2D


def _quaternion_to_yaw(q: Quaternion) -> float:
    """Extract yaw (heading) angle from a ROS quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class IBVSController(Node):
    """Gimbal Pitch + Drone Yaw visual servoing controller.

    The controller runs event-driven: it computes outputs whenever a new
    BoundingBox2D is received from the ArUco detection node.  A fallback
    timer handles the SEARCH phase yaw rotation and publishes commands
    when no detections are available.
    """

    def __init__(self):
        super().__init__('ibvs_controller')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('K_pitch', 0.8)
        self.declare_parameter('K_yaw', 0.5)
        self.declare_parameter('focal_x', 380.0)
        self.declare_parameter('focal_y', 380.0)
        self.declare_parameter('u0', 320.0)
        self.declare_parameter('v0', 240.0)
        self.declare_parameter('search_yaw_rate', 0.2618)   # 15 deg/s
        self.declare_parameter('pitch_rate_limit', 3.1416)  # 180 deg/s
        self.declare_parameter('pitch_ema_alpha', 0.25)

        self.K_pitch = self.get_parameter('K_pitch').value
        self.K_yaw = self.get_parameter('K_yaw').value
        self.fx = self.get_parameter('focal_x').value
        self.fy = self.get_parameter('focal_y').value
        self.u0 = self.get_parameter('u0').value
        self.v0 = self.get_parameter('v0').value
        self.search_yaw_rate = self.get_parameter('search_yaw_rate').value
        self.pitch_rate_limit = self.get_parameter('pitch_rate_limit').value
        self.ema_alpha = self.get_parameter('pitch_ema_alpha').value

        # ── State ────────────────────────────────────────────────────────
        self.phase = 'IDLE'
        self.detected = False
        self.tracking_mode = 'EXPIRED'
        self.drone_yaw = 0.0           # current drone yaw from odometry (rad)
        self.gimbal_pitch = 0.0        # current gimbal pitch command (rad)
        self.gimbal_pitch_filtered = 0.0  # EMA-filtered gimbal pitch
        self.yaw_cmd = 0.0             # output yaw command (rad)
        self.last_update_time = time.monotonic()
        self.last_pixel_u = self.u0    # last known pixel x
        self.last_pixel_v = self.v0    # last known pixel y

        # ── Phase-dependent pitch limits ─────────────────────────────────
        # pitch ≤ 0 means pointing downward (negative = down)
        self.pitch_limits = {
            'SEARCH':   (0.0, 0.0),             # fixed horizontal
            'FOLLOW':   (math.radians(-30), 0.0),
            'APPROACH': (math.radians(-60), math.radians(-15)),
            'LAND':     (math.radians(-90), math.radians(-90)),  # locked down
        }

        # ── Subscribers ──────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        self.create_subscription(
            BoundingBox2D, '/hpad/bbox',
            self.bbox_cb, 10,
        )
        self.create_subscription(
            Bool, '/hpad/detected',
            self.detected_cb, 10,
        )
        self.create_subscription(
            String, '/ekf/tracking_mode',
            self.tracking_mode_cb, 10,
        )
        self.create_subscription(
            String, '/mission/phase',
            self.phase_cb, 10,
        )
        self.create_subscription(
            Odometry, '/odom',
            self.odom_cb, sensor_qos,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.pitch_pub = self.create_publisher(Float64, '/ibvs/gimbal_pitch', 10)
        self.yaw_pub = self.create_publisher(Float64, '/ibvs/yaw_cmd', 10)

        # ── Fallback timer for SEARCH and idle output ────────────────────
        self.create_timer(1.0 / 30.0, self.fallback_timer_cb)

        self.get_logger().info('IBVS Controller ready.')

    # ── Callbacks ────────────────────────────────────────────────────────

    def detected_cb(self, msg: Bool):
        self.detected = msg.data

    def tracking_mode_cb(self, msg: String):
        self.tracking_mode = msg.data

    def phase_cb(self, msg: String):
        self.phase = msg.data

    def odom_cb(self, msg: Odometry):
        self.drone_yaw = _quaternion_to_yaw(msg.pose.pose.orientation)

    def bbox_cb(self, msg: BoundingBox2D):
        """Event-driven: compute IBVS outputs from ArUco bounding box center."""
        if self.phase in ('IDLE',):
            return
        if not self.detected:
            return

        # Pixel coordinates of marker center
        u = msg.center.position.x
        v = msg.center.position.y
        self.last_pixel_u = u
        self.last_pixel_v = v

        now = time.monotonic()
        dt = now - self.last_update_time
        dt = max(0.001, min(dt, 0.5))
        self.last_update_time = now

        self._compute_ibvs(u, v, dt)

    # ── Core IBVS computation ────────────────────────────────────────────

    def _compute_ibvs(self, u: float, v: float, dt: float):
        """Compute gimbal pitch and yaw commands from pixel error."""
        # Pixel error from image center
        eu = u - self.u0   # horizontal error (positive = target is right)
        ev = v - self.v0   # vertical error (positive = target is below center)

        # Get pitch limits for current phase
        pitch_min, pitch_max = self.pitch_limits.get(
            self.phase, (math.radians(-30), 0.0)
        )

        if self.phase == 'LAND':
            # Lock gimbal straight down
            self.gimbal_pitch = math.radians(-90)
            # Fine yaw alignment from pixel error
            self.yaw_cmd = self.drone_yaw + self.K_yaw * (eu / self.fx) * dt

        elif self.phase in ('FOLLOW', 'APPROACH'):
            # Gimbal Pitch: proportional on vertical pixel error
            # When v > v0 (target below center) → pitch down (decrease θ)
            delta_pitch = self.K_pitch * (ev / self.fy) * dt
            self.gimbal_pitch += delta_pitch

            # Rate limiter
            max_delta = self.pitch_rate_limit * dt
            self.gimbal_pitch = max(
                self.gimbal_pitch_filtered - max_delta,
                min(self.gimbal_pitch_filtered + max_delta, self.gimbal_pitch),
            )

            # Clamp to phase limits
            self.gimbal_pitch = max(pitch_min, min(pitch_max, self.gimbal_pitch))

            # EMA filter to smooth servo jitter
            self.gimbal_pitch_filtered = (
                self.ema_alpha * self.gimbal_pitch
                + (1.0 - self.ema_alpha) * self.gimbal_pitch_filtered
            )

            # Drone Yaw: proportional on horizontal pixel error
            self.yaw_cmd = self.drone_yaw + self.K_yaw * (eu / self.fx) * dt

        # Publish
        self._publish_commands()

    def _publish_commands(self):
        """Publish gimbal pitch and yaw commands."""
        pitch_msg = Float64()
        pitch_msg.data = float(self.gimbal_pitch_filtered)
        self.pitch_pub.publish(pitch_msg)

        yaw_msg = Float64()
        yaw_msg.data = float(self.yaw_cmd)
        self.yaw_pub.publish(yaw_msg)

    # ── Fallback timer (SEARCH + idle) ───────────────────────────────────

    def fallback_timer_cb(self):
        """Handle SEARCH yaw rotation and publish when no bbox callback fires."""
        now = time.monotonic()
        dt = now - self.last_update_time
        dt = max(0.001, min(dt, 0.5))

        if self.phase == 'SEARCH':
            # Fixed pitch = 0° (horizontal)
            self.gimbal_pitch = 0.0
            self.gimbal_pitch_filtered = 0.0
            # Slow yaw rotation to scan 360°
            self.yaw_cmd = self.drone_yaw + self.search_yaw_rate * dt
            self.last_update_time = now
            self._publish_commands()

        elif self.phase == 'IDLE':
            self.gimbal_pitch = 0.0
            self.gimbal_pitch_filtered = 0.0
            self.yaw_cmd = self.drone_yaw
            self._publish_commands()

        elif not self.detected and self.tracking_mode in ('PREDICTING', 'PREDICTING_DEGRADED'):
            # Target lost but EKF still predicting — hold last known commands
            # (slightly decay pitch toward center to help re-acquisition)
            self.last_update_time = now
            self._publish_commands()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = IBVSController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[IBVS] Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
