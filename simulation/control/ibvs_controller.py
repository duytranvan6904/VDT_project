#!/usr/bin/env python3
"""IBVS controller for gimbal pitch and aircraft yaw.

The fixed-rate servo uses fresh ArUco pixel error and a bounded target
line-of-sight lead. SEARCH holds its yaw as soon as a detection arrives;
FOLLOW/APPROACH stop using a pixel once its image timestamp expires.

ROS 2 interface:
  Subscribe: /hpad/bbox, /hpad/detected, /ekf/tracking_mode,
             /mission/phase, /odom
  Publish:   /ibvs/gimbal_pitch (Float64 → bridge → Gazebo gimbal),
             /ibvs/yaw_cmd (Float64)
"""

from __future__ import annotations

import math
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Quaternion
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo
    from std_msgs.msg import Bool, Float64, String
    from vision_msgs.msg import BoundingBox2D
except ImportError:
    Node = object
    Quaternion = object
    Odometry = object
    CameraInfo = object
    Bool = object
    Float64 = object
    String = object
    BoundingBox2D = object
    QoSProfile = object
    ReliabilityPolicy = object
    HistoryPolicy = object

try:
    from simulation.control.tracking_control import (
        landing_pitch_from_geometry,
        update_tracking_yaw_command,
        update_search_pitch_command,
    )
except ImportError:
    from tracking_control import (
        landing_pitch_from_geometry,
        update_tracking_yaw_command,
        update_search_pitch_command,
    )


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

    Bounding-box callbacks store the latest image measurement. A 30 Hz timer
    runs the servo so yaw slew and freshness checks use a stable time base.
    """

    def __init__(self):
        super().__init__('ibvs_controller')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('K_pitch', 0.8)
        self.declare_parameter('K_yaw', 0.5)
        self.declare_parameter('focal_x', 466.0)
        self.declare_parameter('focal_y', 466.0)
        self.declare_parameter('u0', 320.0)
        self.declare_parameter('v0', 240.0)
        self.declare_parameter('search_yaw_rate', 0.20)     # 11.5 deg/s
        self.declare_parameter('pitch_rate_limit', 1.5)      # servo limit
        self.declare_parameter('approach_pitch_rate_limit', 0.50)  # rad/s
        self.declare_parameter('pitch_ema_alpha', 0.25)
        self.declare_parameter('pitch_pixel_trim_gain', 0.5)
        self.declare_parameter('yaw_rate_limit', 0.80)       # rad/s (46 deg/s max)
        self.declare_parameter('yaw_feedforward_min_target_speed', 0.30)  # m/s
        self.declare_parameter('yaw_feedforward_max_rate', 0.15)   # rad/s
        self.declare_parameter('target_state_timeout', 0.50)       # seconds
        self.declare_parameter('target_prediction_horizon', 0.25)  # seconds
        self.declare_parameter('landing_pitch_rate_limit', 0.50)  # rad/s
        self.declare_parameter('bbox_timeout', 0.30)         # seconds without a fresh image measurement

        self.K_pitch = self.get_parameter('K_pitch').value
        self.K_yaw = self.get_parameter('K_yaw').value
        self.fx = self.get_parameter('focal_x').value
        self.fy = self.get_parameter('focal_y').value
        self.u0 = self.get_parameter('u0').value
        self.v0 = self.get_parameter('v0').value
        self.search_yaw_rate = self.get_parameter('search_yaw_rate').value
        self.pitch_rate_limit = self.get_parameter('pitch_rate_limit').value
        self.approach_pitch_rate_limit = max(
            0.05, float(self.get_parameter('approach_pitch_rate_limit').value)
        )
        self.ema_alpha = self.get_parameter('pitch_ema_alpha').value
        self.pitch_pixel_trim_gain = float(
            self.get_parameter('pitch_pixel_trim_gain').value
        )
        self.yaw_rate_limit = float(self.get_parameter('yaw_rate_limit').value)
        self.yaw_feedforward_min_target_speed = max(
            0.0,
            float(self.get_parameter('yaw_feedforward_min_target_speed').value),
        )
        self.yaw_feedforward_max_rate = max(
            0.0, float(self.get_parameter('yaw_feedforward_max_rate').value)
        )
        self.target_state_timeout = max(
            0.05, float(self.get_parameter('target_state_timeout').value)
        )
        self.target_prediction_horizon = max(
            0.0, float(self.get_parameter('target_prediction_horizon').value)
        )
        self.landing_pitch_rate_limit = max(
            0.05, float(self.get_parameter('landing_pitch_rate_limit').value)
        )
        self.bbox_timeout = max(
            0.05, float(self.get_parameter('bbox_timeout').value)
        )

        self.declare_parameter('drone_model', 'x500_depth_0')
        self.drone_model = self.get_parameter('drone_model').value

        # ── State ────────────────────────────────────────────────────────
        self.phase = 'IDLE'
        self.detected = False
        self.touchdown_confirmed = False
        self.tracking_mode = 'EXPIRED'
        self.drone_yaw = 0.0           # current drone yaw from odometry (rad)
        self.default_pitch = math.radians(-30.0)  # keep the existing drone setup
        self.gimbal_pitch = self.default_pitch
        self.gimbal_pitch_filtered = self.default_pitch
        self.yaw_cmd = 0.0             # output yaw command (rad)
        self.have_odom = False
        self.last_update_time = time.monotonic()
        self.last_pixel_u = self.u0    # last known pixel x
        self.last_pixel_v = self.v0    # last known pixel y
        self.last_bbox_time = 0.0
        self.visual_measurement_active = False
        self.target_vel = [0.0, 0.0, 0.0]  # EKF target velocity (world frame)
        self.last_eu_sign = 0.0        # hướng pixel cuối cùng (-1 trái, +1 phải)
        self.search_hold_active = False
        self.search_hold_yaw = 0.0
        self.search_hold_last_seen = 0.0
        self.search_hold_timeout = 2.0
        self.search_entry_hold_active = False
        self.search_entry_hold_yaw = 0.0
        self.search_entry_started = 0.0
        self.search_entry_hold_time = 0.30
        self.last_target_time = 0.0
        self.landing_yaw_hold = 0.0
        self.landing_yaw_hold_initialized = False

        # ── Phase-dependent pitch limits ─────────────────────────────────
        # pitch ≤ 0 means pointing downward (negative = down)
        self.pitch_limits = {
            'IDLE':     (math.radians(-45), math.radians(-10)),
            'SEARCH':   (math.radians(-60), math.radians(-10)),  # nới rộng để không bị kẹt khi mất dấu gần
            'FOLLOW':   (math.radians(-88), math.radians(-10)),  # bám theo target linh hoạt đến gần thẳng đứng
            'APPROACH': (math.radians(-88), math.radians(-20)),
            'LAND':     (math.radians(-88), math.radians(-20)),
        }

        # ── Subscribers ──────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        self.drone_pos = [0.0, 0.0, 0.0]
        self.target_pos = None

        self.create_subscription(
            CameraInfo, '/camera_info',
            self.camera_info_cb, 10,
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
            Bool, '/landing/touchdown',
            self.touchdown_cb, 10,
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

    def camera_info_cb(self, msg: CameraInfo):
        if msg.k[0] > 0.0:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.u0 = float(msg.k[2])
            self.v0 = float(msg.k[5])

    def detected_cb(self, msg: Bool):
        self.detected = msg.data
        if msg.data and self.phase == 'SEARCH':
            now = time.monotonic()
            self.search_hold_last_seen = now
            if not self.search_hold_active:
                self.search_hold_active = True
                self.search_hold_yaw = self.drone_yaw
                self.yaw_cmd = self.search_hold_yaw

    def tracking_mode_cb(self, msg: String):
        self.tracking_mode = msg.data

    def touchdown_cb(self, msg: Bool):
        self.touchdown_confirmed = bool(msg.data)

    def phase_cb(self, msg: String):
        previous_phase = self.phase
        self.phase = msg.data
        if self.phase == 'LAND' and previous_phase != 'LAND' and self.have_odom:
            # Keep the heading already commanded during the gated approach.
            # A nadir camera cannot use horizontal pixel error to infer yaw;
            # yawing there makes the marker orbit in the image during descent.
            self.landing_yaw_hold = self.yaw_cmd
            self.landing_yaw_hold_initialized = True
            self.visual_measurement_active = False
        if self.phase == 'SEARCH' and previous_phase != 'SEARCH':
            self.search_hold_active = False
            self.search_hold_last_seen = 0.0
            # The timer will latch the newest odometry sample.  Resetting
            # here can use a stale callback value and create a yaw jump.
            self.search_entry_hold_active = True
            self.search_entry_started = 0.0
        if previous_phase == 'SEARCH' and self.phase == 'FOLLOW' and self.have_odom:
            # Do not carry the accumulated SEARCH sweep command into FOLLOW.
            # The FSM has already confirmed a stable reacquisition; start the
            # visual servo loop from the actual vehicle yaw and let the next
            # bbox update generate the correction.
            self.yaw_cmd = self.drone_yaw
            self.search_hold_active = False
            self.search_hold_last_seen = 0.0
            self.search_entry_hold_active = False
            self.search_entry_started = 0.0
            self.last_update_time = time.monotonic()
            self.get_logger().info(
                f'[IBVS] SEARCH->FOLLOW: reset yaw_cmd to '
                f'{math.degrees(self.drone_yaw):.1f}°'
            )

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
        v = msg.twist.twist.linear
        self.target_vel = [v.x, v.y, v.z]
        self.last_target_time = time.monotonic()

    def bbox_cb(self, msg: BoundingBox2D):
        """Store the newest visual measurement for the fixed-rate control loop."""
        self.last_pixel_u = float(msg.center.position.x)
        self.last_pixel_v = float(msg.center.position.y)
        self.last_bbox_time = time.monotonic()

    # ── Core IBVS computation ────────────────────────────────────────────

    def _compute_ibvs(self, u: float | None, v: float | None, dt: float):
        """Compute gimbal pitch and yaw commands from 3D target geometry + pixel error trimming."""
        pitch_min, pitch_max = self.pitch_limits.get(
            self.phase, (math.radians(-85), 0.0)
        )

        if self.phase == 'LAND':
            has_live_marker = (
                u is not None
                and v is not None
                and self.detected
                and self.tracking_mode == 'TRACKING'
                and self.target_pos is not None
                and self.last_target_time > 0.0
                and time.monotonic() - self.last_target_time <= self.target_state_timeout
            )
            if has_live_marker:
                self.visual_measurement_active = True
                dx = self.target_pos[0] - self.drone_pos[0]
                dy = self.target_pos[1] - self.drone_pos[1]
                horizontal_range = math.hypot(dx, dy)
                target_pitch = landing_pitch_from_geometry(
                    self.drone_pos[2], self.target_pos[2], horizontal_range,
                )
                pitch_min, pitch_max = self.pitch_limits['LAND']
                target_pitch = max(pitch_min, min(pitch_max, target_pitch))
                max_pitch_delta = self.landing_pitch_rate_limit * dt
                self.gimbal_pitch = max(
                    self.gimbal_pitch_filtered - max_pitch_delta,
                    min(self.gimbal_pitch_filtered + max_pitch_delta, target_pitch),
                )
                self.gimbal_pitch_filtered = (
                    self.ema_alpha * self.gimbal_pitch
                    + (1.0 - self.ema_alpha) * self.gimbal_pitch_filtered
                )
            else:
                # Keep the last camera aim and yaw command during a visual
                # dropout; the FSM pauses descent until the marker is visible.
                self.gimbal_pitch = self.gimbal_pitch_filtered
                self.visual_measurement_active = False
            if self.landing_yaw_hold_initialized:
                self.yaw_cmd = self.landing_yaw_hold
            else:
                self.yaw_cmd = self.drone_yaw
            self._publish_commands()
            return

        if self.phase in ('FOLLOW', 'APPROACH'):
            has_pixel = (u is not None and v is not None and self.detected)
            target_state_age = (
                time.monotonic() - self.last_target_time
                if self.last_target_time > 0.0
                else float('inf')
            )
            target_is_usable = (
                self.target_pos is not None
                and self.tracking_mode in ('TRACKING', 'PREDICTING', 'PREDICTING_DEGRADED')
                and 0.0 <= target_state_age <= self.target_state_timeout
            )

            # ── Pre-compute 3D geometry when EKF target is available ──
            dx = dy = dz = dist_h = 0.0
            if target_is_usable:
                dx = self.target_pos[0] - self.drone_pos[0]
                dy = self.target_pos[1] - self.drone_pos[1]
                dz = max(0.1, self.drone_pos[2] - self.target_pos[2])
                dist_h = math.hypot(dx, dy)

            # ── PITCH: 3D geometry feedforward + soft pixel trim ──
            if target_is_usable:
                target_pitch = -math.atan2(dz, max(0.2, dist_h))
                # Khi có pixel, thêm trim nhỏ để bù sai số EKF
                if has_pixel:
                    ev = v - self.v0
                    if abs(ev) < 6.0:
                        ev_eff = 0.0
                    else:
                        ev_eff = ev - math.copysign(6.0, ev)
                    # Positive ev means marker is below center (v > v0).
                    # In Gazebo, negative pitch tilts camera down.
                    # Therefore, subtract correction so ev > 0 tilts camera down to center marker!
                    target_pitch -= (
                        self.K_pitch
                        * (ev_eff / self.fy)
                        * self.pitch_pixel_trim_gain
                    )
            elif has_pixel:
                ev = v - self.v0
                if abs(ev) < 6.0:
                    ev_eff = 0.0
                else:
                    ev_eff = ev - math.copysign(6.0, ev)
                target_pitch = (
                    self.gimbal_pitch_filtered
                    - self.K_pitch
                    * (ev_eff / self.fy)
                    * self.pitch_pixel_trim_gain
                )
            else:
                target_pitch = self.gimbal_pitch_filtered

            # Pitch clamping + rate-limit + EMA
            self.gimbal_pitch = max(pitch_min, min(pitch_max, target_pitch))
            pitch_rate_limit = (
                self.approach_pitch_rate_limit
                if self.phase == 'APPROACH'
                else self.pitch_rate_limit
            )
            max_pitch_delta = pitch_rate_limit * dt
            self.gimbal_pitch = max(
                self.gimbal_pitch_filtered - max_pitch_delta,
                min(self.gimbal_pitch_filtered + max_pitch_delta, self.gimbal_pitch),
            )
            self.gimbal_pitch_filtered = (
                self.ema_alpha * self.gimbal_pitch
                + (1.0 - self.ema_alpha) * self.gimbal_pitch_filtered
            )

            if has_pixel:
                eu = u - self.u0
                self.last_eu_sign = 1.0 if eu >= 0 else -1.0
            self.yaw_cmd = update_tracking_yaw_command(
                yaw_command=self.yaw_cmd,
                drone_yaw=self.drone_yaw,
                pixel_u=u if has_pixel else None,
                image_center_u=self.u0,
                focal_x=self.fx,
                pixel_gain=self.K_yaw,
                target_delta_xy=(dx, dy) if target_is_usable else None,
                target_velocity_xy=(self.target_vel[0], self.target_vel[1]),
                tracking_mode=self.tracking_mode,
                target_state_age=target_state_age,
                target_state_timeout=self.target_state_timeout,
                dt=dt,
                yaw_rate_limit=self.yaw_rate_limit,
                feedforward_min_speed=self.yaw_feedforward_min_target_speed,
                feedforward_max_rate=self.yaw_feedforward_max_rate,
                prediction_horizon=self.target_prediction_horizon,
            )

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
        """Run all IBVS control at a fixed rate and publish continuous commands."""
        now = time.monotonic()
        dt = now - self.last_update_time
        dt = max(0.001, min(dt, 0.1))
        self.last_update_time = now

        bbox_fresh = (
            self.last_bbox_time > 0.0
            and now - self.last_bbox_time <= self.bbox_timeout
        )

        if self.phase == 'SEARCH':
            self.visual_measurement_active = False
            if not self.search_hold_active and not self.detected:
                self.gimbal_pitch = self.default_pitch
                # Chuyển dần về default_pitch mượt mà, không giật nảy đột ngột
                self.gimbal_pitch_filtered = (
                    0.05 * self.default_pitch + 0.95 * self.gimbal_pitch_filtered
                )

            if self.search_entry_hold_active:
                if self.search_entry_started <= 0.0:
                    self.search_entry_started = now
                    self.search_entry_hold_yaw = self.drone_yaw
                    self.yaw_cmd = self.search_entry_hold_yaw
                elif now - self.search_entry_started >= self.search_entry_hold_time:
                    self.search_entry_hold_active = False
                    # Start the sweep from the latched handoff yaw, not from
                    # a stale FOLLOW command or a second asynchronous reset.
                    self.yaw_cmd = self.search_entry_hold_yaw

            if self.detected:
                if not self.search_hold_active:
                    self.search_hold_active = True
                    self.search_hold_yaw = self.drone_yaw
                self.search_hold_last_seen = now
                if bbox_fresh:
                    search_limits = self.pitch_limits['SEARCH']
                    pitch_target = update_search_pitch_command(
                        current_pitch=self.gimbal_pitch_filtered,
                        pixel_v=self.last_pixel_v,
                        image_center_v=self.v0,
                        focal_y=self.fy,
                        pixel_gain=self.K_pitch,
                        trim_gain=self.pitch_pixel_trim_gain,
                        dt=dt,
                        pitch_rate_limit=self.pitch_rate_limit,
                        pitch_limits=search_limits,
                    )
                    self.gimbal_pitch = pitch_target
                    self.gimbal_pitch_filtered = (
                        self.ema_alpha * pitch_target
                        + (1.0 - self.ema_alpha) * self.gimbal_pitch_filtered
                    )
            elif (
                self.search_hold_active
                and now - self.search_hold_last_seen > self.search_hold_timeout
            ):
                self.search_hold_active = False
                # Restart the next sweep from actual yaw, not from a stale
                # command accumulated during the previous sweep.
                self.yaw_cmd = self.drone_yaw

            if self.search_entry_hold_active:
                # Give PX4 time to brake at the phase-entry yaw before sweep.
                self.yaw_cmd = self.search_entry_hold_yaw
            elif self.search_hold_active:
                # Keep a fixed setpoint so the vehicle controller can brake;
                # do not follow actual yaw every cycle.
                self.yaw_cmd = self.search_hold_yaw
            else:
                self.yaw_cmd = _wrap_angle(
                    self.yaw_cmd + self.search_yaw_rate * dt
                )
            self._publish_commands()

        elif self.phase == 'IDLE':
            if not self.touchdown_confirmed:
                self.gimbal_pitch = self.default_pitch
                self.gimbal_pitch_filtered = self.default_pitch
            self.yaw_cmd = self.drone_yaw
            self._publish_commands()

        elif self.phase in ('FOLLOW', 'APPROACH'):
            has_fresh_pixel = self.detected and bbox_fresh
            if has_fresh_pixel:
                if not self.visual_measurement_active:
                    # Resume from actual vehicle heading, never from a stale
                    # setpoint left behind during a tracking interruption.
                    self.yaw_cmd = self.drone_yaw
                self.visual_measurement_active = True
                pixel_u = self.last_pixel_u
                pixel_v = self.last_pixel_v
            else:
                if self.visual_measurement_active:
                    # Brake at the current heading as soon as the visual
                    # measurement expires; do not keep chasing stale pixels.
                    self.yaw_cmd = self.drone_yaw
                self.visual_measurement_active = False
                pixel_u = None
                pixel_v = None

            self._compute_ibvs(
                pixel_u,
                pixel_v,
                dt
            )

        elif self.phase == 'LAND':
            self._compute_ibvs(
                self.last_pixel_u if self.detected and bbox_fresh else None,
                self.last_pixel_v if self.detected and bbox_fresh else None,
                dt,
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
