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


def _wrap_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _slew_angle(current: float, target: float, max_step: float) -> float:
    """Move toward target through the shortest angular path."""
    error = _wrap_angle(target - current)
    return _wrap_angle(current + max(-max_step, min(max_step, error)))


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
        self.declare_parameter('yaw_rate_limit', 0.6)        # rad/s

        self.K_pitch = self.get_parameter('K_pitch').value
        self.K_yaw = self.get_parameter('K_yaw').value
        self.fx = self.get_parameter('focal_x').value
        self.fy = self.get_parameter('focal_y').value
        self.u0 = self.get_parameter('u0').value
        self.v0 = self.get_parameter('v0').value
        self.search_yaw_rate = self.get_parameter('search_yaw_rate').value
        self.pitch_rate_limit = self.get_parameter('pitch_rate_limit').value
        self.ema_alpha = self.get_parameter('pitch_ema_alpha').value
        self.yaw_rate_limit = float(self.get_parameter('yaw_rate_limit').value)

        self.declare_parameter('drone_model', 'x500_depth_0')
        self.drone_model = self.get_parameter('drone_model').value

        # ── State ────────────────────────────────────────────────────────
        self.phase = 'IDLE'
        self.detected = False
        self.tracking_mode = 'EXPIRED'
        self.drone_yaw = 0.0           # current drone yaw from odometry (rad)
        self.default_pitch = math.radians(-20.0)  # -20° chúc nhẹ vừa phải để quét mặt đất
        self.gimbal_pitch = self.default_pitch
        self.gimbal_pitch_filtered = self.default_pitch
        self.yaw_cmd = 0.0             # output yaw command (rad)
        self.have_odom = False
        self.last_update_time = time.monotonic()
        self.last_pixel_u = self.u0    # last known pixel x
        self.last_pixel_v = self.v0    # last known pixel y

        # ── Phase-dependent pitch limits ─────────────────────────────────
        # pitch ≤ 0 means pointing downward (negative = down)
        self.pitch_limits = {
            'IDLE':     (math.radians(-30), math.radians(-10)),
            'SEARCH':   (math.radians(-30), math.radians(-10)),  # chúc -20° quét mặt đất
            'FOLLOW':   (math.radians(-80), math.radians(-10)),  # bám theo target linh hoạt
            'APPROACH': (math.radians(-85), math.radians(-20)),
            'LAND':     (math.radians(-90), math.radians(-60)),  # chúi thẳng xuống H-pad
        }

        # ── Subscribers ──────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        self.drone_pos = [0.0, 0.0, 0.0]
        self.target_pos = None

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
        self.create_subscription(
            Odometry, '/ekf/target_state',
            self.target_cb, 10,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.pitch_pub = self.create_publisher(Float64, '/ibvs/gimbal_pitch', 10)
        self.gz_pitch_pub = self.create_publisher(
            Float64, f'/model/{self.drone_model}/command/gimbal_pitch', 10
        )
        self.yaw_pub = self.create_publisher(Float64, '/ibvs/yaw_cmd', 10)

        # ── Fallback timer for SEARCH and idle output ────────────────────
        self.create_timer(1.0 / 30.0, self.fallback_timer_cb)

        self.get_logger().info('IBVS Controller ready with 3D Feedforward + Visual Servoing.')

    # ── Callbacks ────────────────────────────────────────────────────────

    def detected_cb(self, msg: Bool):
        self.detected = msg.data

    def tracking_mode_cb(self, msg: String):
        self.tracking_mode = msg.data

    def phase_cb(self, msg: String):
        self.phase = msg.data

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.drone_pos = [p.x, p.y, p.z]
        self.drone_yaw = _quaternion_to_yaw(msg.pose.pose.orientation)
        if not self.have_odom:
            self.yaw_cmd = self.drone_yaw
            self.have_odom = True

    def target_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.target_pos = [p.x, p.y, p.z]

    def bbox_cb(self, msg: BoundingBox2D):
        """Event-driven: compute IBVS outputs from ArUco bounding box center."""
        if self.phase in ('IDLE',):
            return

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

    def _compute_ibvs(self, u: float | None, v: float | None, dt: float):
        """Compute gimbal pitch and yaw commands from 3D target geometry + pixel error trimming."""
        pitch_min, pitch_max = self.pitch_limits.get(
            self.phase, (math.radians(-85), 0.0)
        )

        if self.phase == 'LAND':
            self.gimbal_pitch = math.radians(-90)
            self.gimbal_pitch_filtered = math.radians(-90)
            if u is not None:
                eu = u - self.u0
                desired_yaw = self.drone_yaw - self.K_yaw * (eu / self.fx) * dt
                self.yaw_cmd = _slew_angle(
                    self.yaw_cmd, desired_yaw, self.yaw_rate_limit * dt,
                )
            else:
                self.yaw_cmd = self.drone_yaw
            self._publish_commands()
            return

        if self.phase in ('FOLLOW', 'APPROACH'):
            # 1. Tính góc pitch và yaw hình học trực tiếp từ vị trí 3D của EKF target
            target_is_usable = (
                self.target_pos is not None
                and self.tracking_mode in ('TRACKING', 'PREDICTING')
            )
            if target_is_usable:
                dx = self.target_pos[0] - self.drone_pos[0]
                dy = self.target_pos[1] - self.drone_pos[1]
                dz = max(0.1, self.drone_pos[2] - self.target_pos[2])
                dist_h = math.hypot(dx, dy)
                target_pitch = - math.atan2(dz, max(0.2, dist_h))
                target_yaw = math.atan2(dy, dx)
            else:
                # Do not keep commanding a stale target after EKF expiry.
                target_pitch = self.gimbal_pitch_filtered
                target_yaw = self.drone_yaw

            # 2. Tinh chỉnh bằng độ lệch pixel IBVS khi có detection
            # eu > 0 nghĩa là mục tiêu lệch sang phải ảnh -> drone cần quay sang phải (giảm yaw ENU)
            if u is not None and v is not None and self.detected:
                eu = u - self.u0
                ev = v - self.v0
                target_pitch -= self.K_pitch * (ev / self.fy) * 0.15
                target_yaw -= self.K_yaw * (eu / self.fx) * 0.15

            # Giới hạn góc và lọc mượt
            self.gimbal_pitch = max(pitch_min, min(pitch_max, target_pitch))
            max_pitch_delta = self.pitch_rate_limit * dt
            self.gimbal_pitch = max(
                self.gimbal_pitch_filtered - max_pitch_delta,
                min(self.gimbal_pitch_filtered + max_pitch_delta, self.gimbal_pitch),
            )
            self.gimbal_pitch_filtered = (
                self.ema_alpha * self.gimbal_pitch
                + (1.0 - self.ema_alpha) * self.gimbal_pitch_filtered
            )

            # The previous implementation published atan2() directly.  A
            # noisy EKF estimate or a +/-pi crossing then looked like a large
            # yaw step, making PX4 spin and causing the camera to lose the
            # marker.  Follow the shortest angular error at a bounded rate.
            if target_is_usable:
                self.yaw_cmd = _slew_angle(
                    self.yaw_cmd,
                    target_yaw,
                    self.yaw_rate_limit * dt,
                )
            else:
                self.yaw_cmd = self.drone_yaw
            self._publish_commands()

    def _publish_commands(self):
        """Publish gimbal pitch and yaw commands."""
        pitch_msg = Float64()
        pitch_msg.data = float(self.gimbal_pitch_filtered)
        self.pitch_pub.publish(pitch_msg)
        self.gz_pitch_pub.publish(pitch_msg)

        yaw_msg = Float64()
        yaw_msg.data = float(self.yaw_cmd)
        self.yaw_pub.publish(yaw_msg)

    # ── Fallback timer (SEARCH + idle) ───────────────────────────────────

    def fallback_timer_cb(self):
        """Handle SEARCH yaw rotation and publish continuous commands at 30 Hz."""
        now = time.monotonic()
        dt = now - self.last_update_time
        dt = max(0.001, min(dt, 0.5))

        if self.phase == 'SEARCH':
            self.gimbal_pitch = self.default_pitch
            self.gimbal_pitch_filtered = self.default_pitch
            self.yaw_cmd = _wrap_angle(
                self.yaw_cmd + self.search_yaw_rate * dt
            )
            self.last_update_time = now
            self._publish_commands()

        elif self.phase == 'IDLE':
            self.gimbal_pitch = self.default_pitch
            self.gimbal_pitch_filtered = self.default_pitch
            self.yaw_cmd = self.drone_yaw
            self._publish_commands()

        elif self.phase in ('FOLLOW', 'APPROACH'):
            # Luôn duy trì bám góc gimbal 3D liên tục ở 30 Hz
            self.last_update_time = now
            self._compute_ibvs(
                self.last_pixel_u if self.detected else None,
                self.last_pixel_v if self.detected else None,
                dt
            )


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
