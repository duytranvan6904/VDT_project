#!/usr/bin/env python3
"""Mission FSM Node — State machine IDLE → SEARCH → FOLLOW → APPROACH → LAND.

Trách nhiệm:
  1. Quản lý chuyển pha dựa trên detection state, operator command, alignment
  2. Publish /mission/phase (String) để các node khác biết pha hiện tại
  3. Tổng hợp velocity + yaw thành setpoint cuối cùng gửi Offboard Commander

Transition rules:
  IDLE     → SEARCH  : sau khi takeoff (drone altitude > 80% takeoff_alt)
  SEARCH   → REACQUIRE_HOLD → FOLLOW : raw detection + accepted EKF + settled yaw
  FOLLOW   → SEARCH  : EKF EXPIRED (mất > search_timeout)
  FOLLOW   → APPROACH: Operator LAND command
  APPROACH → FOLLOW  : Mất ArUco > approach_timeout HOẶC Operator cancel
  APPROACH → LAND    : alignment < threshold AND alt < land_altitude
  LAND     → IDLE    : touchdown (alt ≈ 0)

ROS 2 interface:
  Subscribe: /ekf/tracking_mode, /ekf/target_state, /hpad/detected, /odom,
             /operator/land_command, /apf/velocity_cmd, /ibvs/yaw_cmd
  Publish:   /mission/phase, /mission/velocity_setpoint, /mission/yaw_setpoint
"""

from __future__ import annotations

import math
import time
from enum import Enum

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String
from vision_msgs.msg import BoundingBox2D


class MissionPhase(str, Enum):
    IDLE = 'IDLE'
    SEARCH = 'SEARCH'
    FOLLOW = 'FOLLOW'
    APPROACH = 'APPROACH'
    LAND = 'LAND'


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class MissionFSMNode(Node):
    """Mission state machine that orchestrates all autonomous components."""

    def __init__(self):
        super().__init__('mission_fsm')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('follow_distance', 3.5)
        self.declare_parameter('alignment_threshold', 0.3)
        self.declare_parameter('land_altitude', 0.5)
        self.declare_parameter('search_timeout', 3.0)
        self.declare_parameter('approach_timeout', 3.0)
        self.declare_parameter('takeoff_altitude', 3.0)
        self.declare_parameter('land_descent_speed', 0.3)
        self.declare_parameter('yaw_align_enable', True)
        self.declare_parameter('yaw_align_enter_deg', 20.0)
        self.declare_parameter('yaw_align_exit_deg', 8.0)
        self.declare_parameter('yaw_align_u0_px', 320.0)
        self.declare_parameter('yaw_align_v0_px', 240.0)
        self.declare_parameter('yaw_align_u_tolerance_px', 35.0)
        self.declare_parameter('yaw_align_u_stop_px', 220.0)
        self.declare_parameter('yaw_align_min_speed_scale', 0.45)
        self.declare_parameter('yaw_align_speed_filter_alpha', 0.35)
        self.declare_parameter('reacquire_confirm_time', 0.20)
        self.declare_parameter('reacquire_hold_timeout', 4.00)
        self.declare_parameter('reacquire_lost_timeout', 2.00)
        self.declare_parameter('reacquire_bbox_timeout', 1.00)
        self.declare_parameter('reacquire_max_yaw_rate_deg', 25.0)
        self.declare_parameter('follow_entry_hold_time', 0.35)
        self.declare_parameter('search_entry_hold_time', 0.30)

        self.follow_dist = self.get_parameter('follow_distance').value
        self.align_thresh = self.get_parameter('alignment_threshold').value
        self.land_alt = self.get_parameter('land_altitude').value
        self.search_timeout = self.get_parameter('search_timeout').value
        self.approach_timeout = self.get_parameter('approach_timeout').value
        self.takeoff_alt = self.get_parameter('takeoff_altitude').value
        self.descent_speed = self.get_parameter('land_descent_speed').value
        self.yaw_align_enable = bool(self.get_parameter('yaw_align_enable').value)
        self.yaw_align_enter = math.radians(
            float(self.get_parameter('yaw_align_enter_deg').value)
        )
        self.yaw_align_exit = math.radians(
            float(self.get_parameter('yaw_align_exit_deg').value)
        )
        self.yaw_align_u0 = float(self.get_parameter('yaw_align_u0_px').value)
        self.yaw_align_v0 = float(self.get_parameter('yaw_align_v0_px').value)
        self.yaw_align_u_tol = float(
            self.get_parameter('yaw_align_u_tolerance_px').value
        )
        self.yaw_align_u_stop = max(
            self.yaw_align_u_tol + 1.0,
            float(self.get_parameter('yaw_align_u_stop_px').value),
        )
        self.yaw_align_min_speed = float(np.clip(
            self.get_parameter('yaw_align_min_speed_scale').value,
            0.0, 1.0,
        ))
        self.yaw_align_filter_alpha = float(np.clip(
            self.get_parameter('yaw_align_speed_filter_alpha').value,
            0.0, 1.0,
        ))
        self.reacquire_confirm_time = float(
            self.get_parameter('reacquire_confirm_time').value
        )
        self.reacquire_hold_timeout = float(
            self.get_parameter('reacquire_hold_timeout').value
        )
        self.reacquire_lost_timeout = float(
            self.get_parameter('reacquire_lost_timeout').value
        )
        self.reacquire_bbox_timeout = float(
            self.get_parameter('reacquire_bbox_timeout').value
        )
        self.reacquire_max_yaw_rate = math.radians(float(
            self.get_parameter('reacquire_max_yaw_rate_deg').value
        ))
        self.follow_entry_hold_time = float(
            self.get_parameter('follow_entry_hold_time').value
        )
        self.search_entry_hold_time = float(
            self.get_parameter('search_entry_hold_time').value
        )

        # ── State ────────────────────────────────────────────────────────
        self.phase = MissionPhase.IDLE
        self.detected = False
        self.tracking_mode = 'EXPIRED'
        self.drone_pos = np.zeros(3)
        self.drone_yaw = 0.0
        self.drone_yaw_rate = 0.0
        self.last_odom_time = 0.0
        self.target_pos = np.zeros(3)
        self.has_odom = False
        self.has_target = False
        self.land_requested = False
        self.last_detection_time = 0.0

        # APF + IBVS outputs
        self.apf_velocity = np.zeros(3)
        self.ibvs_yaw = 0.0
        self.yaw_align_active = False
        self.xy_speed_scale = 0.0
        self.has_bbox = False
        self.bbox_u = self.yaw_align_u0
        self.bbox_v = self.yaw_align_v0
        self.last_bbox_time = 0.0
        self.reacquire_active = False
        self.reacquire_started = 0.0
        self.reacquire_stable_since = 0.0
        self.reacquire_last_detection = 0.0
        self.reacquire_hold_yaw = 0.0
        self.reacquire_timeout_warned = False
        self.follow_entry_hold_until = 0.0
        self.search_entry_hold_yaw = 0.0
        self.search_entry_hold_until = 0.0

        # Precision landing & Touchdown state
        self.touchdown_detected = False
        self.safe_to_land = False
        self.has_smc_cmd = False
        self.smc_velocity = np.zeros(3)
        self.smc_yaw_rate = 0.0
        self.last_smc_time = 0.0

        # ── Subscribers ──────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )

        self.create_subscription(Odometry, '/odom', self.odom_cb, sensor_qos)
        self.create_subscription(
            Odometry, '/ekf/target_state', self.target_cb, 10,
        )
        self.create_subscription(Bool, '/hpad/detected', self.detected_cb, 10)
        self.create_subscription(BoundingBox2D, '/hpad/bbox', self.bbox_cb, 10)
        self.create_subscription(
            String, '/ekf/tracking_mode', self.tracking_mode_cb, 10,
        )
        self.create_subscription(
            Bool, '/operator/land_command', self.land_cmd_cb, 10,
        )
        self.create_subscription(
            Twist, '/apf/velocity_cmd', self.apf_vel_cb, 10,
        )
        self.create_subscription(
            Float64, '/ibvs/yaw_cmd', self.ibvs_yaw_cb, 10,
        )
        self.create_subscription(
            Bool, '/landing/touchdown', self.touchdown_cb, 10,
        )
        self.create_subscription(
            Bool, '/landing/safe_to_land', self.safe_to_land_cb, 10,
        )
        self.create_subscription(
            Twist, '/landing/velocity_cmd', self.smc_vel_cb, 10,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.phase_pub = self.create_publisher(String, '/mission/phase', 10)
        self.vel_pub = self.create_publisher(
            Twist, '/mission/velocity_setpoint', 10,
        )
        self.yaw_pub = self.create_publisher(
            Float64, '/mission/yaw_setpoint', 10,
        )

        # ── Main FSM timer (20 Hz) & Diagnostic timer (1 Hz) ───────────
        self.create_timer(0.05, self.fsm_update)
        self.create_timer(1.0, self.status_log_update)

        self.get_logger().info('Mission FSM ready. Starting in IDLE phase.')

    def status_log_update(self):
        """In log chẩn đoán trạng thái định kỳ 1 giây/lần để người dùng theo dõi."""
        if not self.has_odom:
            return
        dist_str = "N/A"
        if self.has_target:
            d = np.linalg.norm(self.drone_pos - self.target_pos)
            dist_str = f"{d:.2f}m"
        bbox_str = (
            f"({self.bbox_u:.0f},{self.bbox_v:.0f})"
            if self.has_bbox else "N/A"
        )
        self.get_logger().info(
            f"[STATUS] Phase: {self.phase.value:<7} | "
            f"Drone: ({self.drone_pos[0]:.1f}, {self.drone_pos[1]:.1f}, {self.drone_pos[2]:.1f})m | "
            f"Target: ({self.target_pos[0]:.1f}, {self.target_pos[1]:.1f}) | "
            f"Dist: {dist_str} | "
            f"EKF: {self.tracking_mode} | Det: {self.detected} | "
            f"BBox: {bbox_str} | XY scale: {self.xy_speed_scale:.2f} | "
            f"Reacq: {self.reacquire_active}"
        )

    # ── Input callbacks ──────────────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        now = time.monotonic()
        p = msg.pose.pose.position
        self.drone_pos = np.array([p.x, p.y, p.z])
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        new_yaw = math.atan2(siny, cosy)
        if self.has_odom and self.last_odom_time > 0.0:
            dt = now - self.last_odom_time
            if dt > 1e-3:
                # Prefer odometry's angular velocity (less sensitive to
                # callback jitter); fall back to differentiated yaw if absent.
                measured_rate = float(msg.twist.twist.angular.z)
                yaw_delta_rate = _wrap_angle(new_yaw - self.drone_yaw) / dt
                if not math.isfinite(measured_rate) or (
                    abs(measured_rate) < 1e-3 and abs(yaw_delta_rate) > 0.02
                ):
                    measured_rate = yaw_delta_rate
                alpha = 0.25
                self.drone_yaw_rate = (
                    alpha * measured_rate + (1.0 - alpha) * self.drone_yaw_rate
                )
        self.drone_yaw = new_yaw
        self.last_odom_time = now
        self.has_odom = True

    def target_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.target_pos = np.array([p.x, p.y, p.z])
        self.has_target = True

    def detected_cb(self, msg: Bool):
        self.detected = msg.data
        if msg.data:
            now = time.monotonic()
            self.last_detection_time = now
            # Detection is an event, not something to wait for the 20 Hz FSM
            # loop to notice. Latch the current heading here so the next
            # published setpoint immediately brakes the search sweep.
            if self.phase == MissionPhase.SEARCH and not self.reacquire_active:
                self._start_reacquire(now)

    def bbox_cb(self, msg: BoundingBox2D):
        """Keep image alignment independent from the yaw command topic."""
        self.bbox_u = float(msg.center.position.x)
        self.bbox_v = float(msg.center.position.y)
        self.last_bbox_time = time.monotonic()
        self.has_bbox = True

    def tracking_mode_cb(self, msg: String):
        self.tracking_mode = msg.data

    def land_cmd_cb(self, msg: Bool):
        self.land_requested = msg.data

    def apf_vel_cb(self, msg: Twist):
        self.apf_velocity = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
        ])

    def ibvs_yaw_cb(self, msg: Float64):
        self.ibvs_yaw = msg.data

    def touchdown_cb(self, msg: Bool):
        if self.phase == MissionPhase.LAND:
            self.touchdown_detected = msg.data
        else:
            self.touchdown_detected = False

    def safe_to_land_cb(self, msg: Bool):
        self.safe_to_land = msg.data

    def smc_vel_cb(self, msg: Twist):
        self.smc_velocity[0] = msg.linear.x
        self.smc_velocity[1] = msg.linear.y
        self.smc_velocity[2] = msg.linear.z
        self.smc_yaw_rate = msg.angular.z
        self.last_smc_time = time.monotonic()
        self.has_smc_cmd = True

    # ── FSM Logic ────────────────────────────────────────────────────────

    def fsm_update(self):
        """Main state machine update at 20 Hz."""
        prev_phase = self.phase

        # ── Transition logic ─────────────────────────────────────────────
        if self.phase == MissionPhase.IDLE:
            self._handle_idle()

        elif self.phase == MissionPhase.SEARCH:
            self._handle_search()

        elif self.phase == MissionPhase.FOLLOW:
            self._handle_follow()

        elif self.phase == MissionPhase.APPROACH:
            self._handle_approach()

        elif self.phase == MissionPhase.LAND:
            self._handle_land()

        # ── Log transition ───────────────────────────────────────────────
        if self.phase != prev_phase:
            self.get_logger().info(
                f'[FSM] Phase transition: {prev_phase.value} → {self.phase.value}'
            )

        # ── Publish phase ────────────────────────────────────────────────
        self.phase_pub.publish(String(data=self.phase.value))

        # ── Publish setpoints ────────────────────────────────────
        self._publish_setpoints()

    def _handle_idle(self):
        """Wait for takeoff. Chuyển pha khi drone lên trên 2.5m (gần hoàn tất cất cánh)."""
        if not self.has_odom:
            return
        if self.drone_pos[2] >= 2.5:
            # Always enter SEARCH first.  Initial detection must use the same
            # stable reacquisition handshake as a target found after loss.
            self.search_entry_hold_yaw = self.drone_yaw
            self.search_entry_hold_until = (
                time.monotonic() + self.search_entry_hold_time
            )
            self.phase = MissionPhase.SEARCH
            self.get_logger().info(
                f'Drone cất cánh đạt độ cao {self.drone_pos[2]:.2f}m. '
                f'Bắt đầu SEARCH và xác nhận mục tiêu ổn định.'
            )

    def _handle_search(self):
        """Search, then stop rotation and confirm a stable reacquisition."""
        now = time.monotonic()

        # A raw detector hit is only a trigger to stop the search sweep.  It
        # is not yet permission to enter FOLLOW: the target can be crossing
        # the image while the vehicle is still rotating.
        if self.detected and not self.reacquire_active:
            self._start_reacquire(now)

        if not self.reacquire_active:
            return

        if self.detected:
            self.reacquire_last_detection = now
        detection_recent = (
            now - self.reacquire_last_detection <= self.reacquire_lost_timeout
        )
        if not detection_recent:
            self.reacquire_active = False
            self.reacquire_stable_since = 0.0
            self.reacquire_timeout_warned = False
            self.get_logger().info(
                '[REACQUIRE] detection lost before confirmation; resume SEARCH'
            )
            return

        if (
            now - self.reacquire_started > self.reacquire_hold_timeout
            and not self.reacquire_timeout_warned
        ):
            # Never restart the sweep while the detector still sees a target:
            # doing so recreates the spin/reacquire loop. Keep hovering at the
            # latched heading and wait for either valid tracking or a real loss.
            self.reacquire_timeout_warned = True
            self.get_logger().warning(
                '[REACQUIRE] still waiting after timeout; sweep remains stopped '
                'while detection is present'
            )

        # Once the fixed yaw target has been sent, verify target tracking and
        # bounded yaw rate so the vehicle transitions cleanly to FOLLOW.
        bbox_fresh = (
            self.last_bbox_time >= self.reacquire_started
            and now - self.last_bbox_time <= self.reacquire_bbox_timeout
        )
        bbox_in_roi = (
            20.0 <= self.bbox_u <= 620.0
            and 20.0 <= self.bbox_v <= 460.0
        )
        tracking_ok = self.tracking_mode in ('TRACKING', 'PREDICTING')
        yaw_settled = abs(self.drone_yaw_rate) <= self.reacquire_max_yaw_rate

        if bbox_fresh and bbox_in_roi and tracking_ok and yaw_settled and detection_recent:
            if self.reacquire_stable_since <= 0.0:
                self.reacquire_stable_since = now
        else:
            self.reacquire_stable_since = 0.0

        confirmed = (
            self.reacquire_stable_since > 0.0
            and tracking_ok
            and detection_recent
            and now - self.reacquire_stable_since >= self.reacquire_confirm_time
            and now - self.last_bbox_time <= self.reacquire_bbox_timeout
        )
        if confirmed:
            self.reacquire_active = False
            self.phase = MissionPhase.FOLLOW
            self.follow_entry_hold_until = now + self.follow_entry_hold_time
            self.follow_entry_hold_yaw = self.drone_yaw
            self.get_logger().info(
                f'[REACQUIRE] confirmed after '
                f'{now - self.reacquire_started:.2f}s '
                f'| bbox=({self.bbox_u:.0f},{self.bbox_v:.0f}) '
                f'| EKF={self.tracking_mode}; entering FOLLOW'
            )

    def _start_reacquire(self, now: float):
        """Latch a fixed yaw immediately on the first SEARCH detection."""
        self.reacquire_active = True
        self.reacquire_started = now
        self.reacquire_stable_since = 0.0
        self.reacquire_last_detection = now
        self.reacquire_hold_yaw = self.drone_yaw
        self.reacquire_timeout_warned = False
        self.get_logger().info(
            f'[REACQUIRE] detection trigger; stop sweep and hold yaw '
            f'| yaw={math.degrees(self.reacquire_hold_yaw):.1f}° '
            f'| bbox=({self.bbox_u:.0f},{self.bbox_v:.0f})'
        )

    def _handle_follow(self):
        """Follow target with APF obstacle avoidance."""
        # Check for target loss
        if self.tracking_mode == 'EXPIRED':
            age = time.monotonic() - self.last_detection_time
            if age > self.search_timeout:
                self.search_entry_hold_yaw = self.drone_yaw
                self.search_entry_hold_until = (
                    time.monotonic() + self.search_entry_hold_time
                )
                self.phase = MissionPhase.SEARCH
                self.get_logger().warning(
                    f'Target lost for {age:.1f}s, returning to SEARCH.'
                )
                return

        # Check for operator LAND command
        if self.land_requested:
            self.phase = MissionPhase.APPROACH
            self.get_logger().info('Operator LAND command received.')

    def _handle_approach(self):
        """Approach target for landing."""
        # Cancel approach if target lost too long
        if self.tracking_mode == 'EXPIRED':
            age = time.monotonic() - self.last_detection_time
            if age > self.approach_timeout:
                self.phase = MissionPhase.FOLLOW
                self.land_requested = False
                self.get_logger().warning(
                    f'Target lost during APPROACH for {age:.1f}s, '
                    f'returning to FOLLOW.'
                )
                return

        # Cancel approach if operator retracts LAND command
        if not self.land_requested:
            self.phase = MissionPhase.FOLLOW
            self.get_logger().info('Operator cancelled LAND, returning to FOLLOW.')
            return

        # Check alignment and altitude for transition to LAND
        if self.has_target and self.has_odom:
            horizontal_error = np.linalg.norm(
                self.drone_pos[:2] - self.target_pos[:2]
            )
            altitude = self.drone_pos[2]

            covariance_safe = bool(self.safe_to_land)
            legacy_safe = (horizontal_error < self.align_thresh and altitude < self.land_alt)

            if covariance_safe or legacy_safe:
                self.phase = MissionPhase.LAND
                self.touchdown_detected = False
                self.get_logger().info(
                    f'Landing condition met (cov_safe={covariance_safe}, '
                    f'error={horizontal_error:.2f}m, alt={altitude:.2f}m), starting LAND.'
                )

    def _handle_land(self):
        """Vertical descent until touchdown."""
        # Multi-tiered drift-invariant touchdown detection:
        # Prioritize confirmed touchdown signal from TouchdownDetector,
        # with secondary fallback on low altitude.
        if self.touchdown_detected or (self.has_odom and self.drone_pos[2] < 0.12):
            self.phase = MissionPhase.IDLE
            self.land_requested = False
            self.touchdown_detected = False
            self.get_logger().info('🏆 Touchdown confirmed! Returning to IDLE.')

    # ── Setpoint composition ─────────────────────────────────────────────

    def _bbox_is_recent(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return (
            self.has_bbox
            and self.last_bbox_time > 0.0
            and now - self.last_bbox_time <= self.reacquire_bbox_timeout
        )

    def _detection_is_recent(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return (
            self.detected
            or (
                self.last_detection_time > 0.0
                and now - self.last_detection_time <= self.reacquire_lost_timeout
            )
        )

    def _tracking_input_is_valid(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return (
            self.has_odom
            and self.tracking_mode in ('TRACKING', 'PREDICTING')
            and self._bbox_is_recent(now)
            and self._detection_is_recent(now)
        )

    def _translation_speed_scale(self) -> float:
        """Smoothly scale APF translation while IBVS aligns the view.

        IBVS owns yaw and gimbal pitch; APF owns world-frame translation.
        A hard zero-velocity gate can deadlock the system because vertical
        image error is primarily corrected by the gimbal, not by yaw/APF.
        Invalid tracking is still an immediate safety stop.
        """
        valid_tracking = self._tracking_input_is_valid()
        if not valid_tracking:
            self.xy_speed_scale = 0.0
            self.yaw_align_active = True
            return 0.0

        if not self.yaw_align_enable:
            raw_scale = 1.0
        else:
            eu = abs(self.bbox_u - self.yaw_align_u0)
            if eu <= self.yaw_align_u_tol:
                pixel_scale = 1.0
            elif eu >= self.yaw_align_u_stop:
                pixel_scale = self.yaw_align_min_speed
            else:
                span = self.yaw_align_u_stop - self.yaw_align_u_tol
                progress = (eu - self.yaw_align_u_tol) / span
                pixel_scale = 1.0 - progress * (
                    1.0 - self.yaw_align_min_speed
                )

            yaw_error = abs(_wrap_angle(self.ibvs_yaw - self.drone_yaw))
            if yaw_error <= self.yaw_align_exit:
                yaw_scale = 1.0
            elif yaw_error >= self.yaw_align_enter:
                yaw_scale = self.yaw_align_min_speed
            else:
                span = self.yaw_align_enter - self.yaw_align_exit
                progress = (yaw_error - self.yaw_align_exit) / span
                yaw_scale = 1.0 - progress * (
                    1.0 - self.yaw_align_min_speed
                )

            raw_scale = min(pixel_scale, yaw_scale)

        # Smooth only valid-operation changes. A tracking loss must stop
        # translation immediately, above.
        alpha = self.yaw_align_filter_alpha
        self.xy_speed_scale = (
            alpha * raw_scale + (1.0 - alpha) * self.xy_speed_scale
        )
        self.xy_speed_scale = float(np.clip(self.xy_speed_scale, 0.0, 1.0))

        alignment_limited = self.xy_speed_scale < 0.995
        if alignment_limited != self.yaw_align_active:
            self.yaw_align_active = alignment_limited
            self.get_logger().info(
                f'[YAW_ALIGN] {"LIMIT XY" if alignment_limited else "FULL XY"} '
                f'| scale={self.xy_speed_scale:.2f} '
                f'| tracking={self.tracking_mode} | detected={self.detected} '
                f'| yaw_err={math.degrees(_wrap_angle(self.ibvs_yaw - self.drone_yaw)):.1f}° '
                f'| pixel_err=({self.bbox_u - self.yaw_align_u0:.0f},'
                f'{self.bbox_v - self.yaw_align_v0:.0f})'
            )
        return self.xy_speed_scale

    def _publish_setpoints(self):
        """Compose final velocity and yaw setpoints based on current phase."""
        vel = Twist()
        yaw = Float64()

        if self.phase == MissionPhase.IDLE:
            # Hover at current position (zero velocity)
            yaw.data = float(self.drone_yaw)

        elif self.phase == MissionPhase.SEARCH:
            # Hover + IBVS provides yaw rotation
            # During reacquisition, hold the fixed yaw captured at the first
            # detection so PX4 can actually brake the ongoing rotation.
            now = time.monotonic()
            if self.reacquire_active:
                search_yaw = self.reacquire_hold_yaw
            elif now < self.search_entry_hold_until:
                # Do not publish stale IBVS FOLLOW yaw during the first
                # SEARCH cycle.  IBVS resets asynchronously on phase change.
                search_yaw = self.search_entry_hold_yaw
            else:
                search_yaw = self.ibvs_yaw
            yaw.data = float(search_yaw)

        elif self.phase == MissionPhase.FOLLOW:
            # IBVS owns yaw/gimbal and APF owns translation. During visual
            # alignment, smoothly reduce APF speed instead of freezing XY.
            # Invalid tracking remains an immediate zero-velocity condition.
            speed_scale = self._translation_speed_scale()
            entry_hold = time.monotonic() < self.follow_entry_hold_until
            if not entry_hold:
                vel.linear.x = float(self.apf_velocity[0] * speed_scale)
                vel.linear.y = float(self.apf_velocity[1] * speed_scale)
            # Keep APF/offboard altitude regulation independent from the
            # horizontal visual-alignment scale.
            vel.linear.z = float(self.apf_velocity[2])
            yaw.data = float(
                self.follow_entry_hold_yaw
                if entry_hold
                else self.ibvs_yaw
            )

        elif self.phase == MissionPhase.APPROACH:
            # APF velocity (reduced gain handled by APF node) + descent
            tracking_active = self.tracking_mode in ('TRACKING', 'PREDICTING')
            if tracking_active:
                vel.linear.x = float(self.apf_velocity[0])
                vel.linear.y = float(self.apf_velocity[1])
                # Gradual descent toward target altitude
                alt_error = self.drone_pos[2] - self.target_pos[2] if self.has_target else 0.0
                if alt_error > 0.2:
                    vel.linear.z = float(-self.descent_speed)
                else:
                    vel.linear.z = float(self.apf_velocity[2])
                yaw.data = float(self.ibvs_yaw)
            else:
                # Never continue descending on a stale target estimate.
                # Wait for reacquisition or let the timeout return to FOLLOW.
                yaw.data = float(self.drone_yaw)

        elif self.phase == MissionPhase.LAND:
            now = time.monotonic()
            if self.has_smc_cmd and (now - self.last_smc_time < 0.5):
                vel.linear.x = float(self.smc_velocity[0])
                vel.linear.y = float(self.smc_velocity[1])
                vel.linear.z = float(self.smc_velocity[2])
                yaw.data = float(self.ibvs_yaw if self.detected else self.drone_yaw)
            else:
                # Straight down fallback, lock yaw
                vel.linear.z = float(-self.descent_speed)
                yaw.data = float(self.drone_yaw)  # hold current yaw

        self.vel_pub.publish(vel)
        self.yaw_pub.publish(yaw)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = MissionFSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[Mission FSM] Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
