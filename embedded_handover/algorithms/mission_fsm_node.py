#!/usr/bin/env python3
"""Mission FSM Node — mission FSM with a gated precision-landing sub-FSM.

Trách nhiệm:
  1. Quản lý chuyển pha dựa trên tracking state và operator command
  2. Publish /mission/phase (String) để các node khác biết pha hiện tại
  3. Tổng hợp velocity + yaw thành setpoint cuối cùng gửi Offboard Commander

Transition rules:
  IDLE     → SEARCH  : sau khi drone đạt độ cao takeoff an toàn
  SEARCH   → REACQUIRE_HOLD → FOLLOW : raw detection + accepted EKF + settled yaw
  FOLLOW   → SEARCH  : EKF EXPIRED (mất > search_timeout)
  FOLLOW   → APPROACH: Operator LAND command
  APPROACH → FOLLOW  : Mất ArUco > approach_timeout HOẶC Operator cancel
  APPROACH → LAND    : GLIDE_SLOPE + Rxy/altitude/uncertainty gates all pass
  LAND     → IDLE    : touchdown (alt ≈ 0)

ROS 2 interface:
  Subscribe: /ekf/tracking_mode, /ekf/target_state, /hpad/detected, /odom,
             /operator/land_command, APF guidance/feedforward, /ibvs/yaw_cmd
  Publish:   /mission/phase, /mission/velocity_setpoint, /mission/yaw_setpoint
"""

from __future__ import annotations

import math
import os
import sys
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

try:
    from simulation.control.tracking_control import (
        choose_landing_yaw,
        compose_follow_velocity,
        follow_motion_yaw,
        forward_heading_guard,
        reacquire_candidate_is_valid,
        reacquire_should_resume_search,
        slew_yaw_reference,
        FollowYawReferenceSelector,
        target_velocity_feedforward,
    )
except ImportError:
    repo_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    )
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)
    from simulation.control.tracking_control import (
        choose_landing_yaw,
        compose_follow_velocity,
        follow_motion_yaw,
        forward_heading_guard,
        reacquire_candidate_is_valid,
        reacquire_should_resume_search,
        slew_yaw_reference,
        FollowYawReferenceSelector,
        target_velocity_feedforward,
    )


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
        self.declare_parameter('follow_distance', 2.2)
        self.declare_parameter('search_timeout', 3.0)
        self.declare_parameter('approach_timeout', 3.0)
        self.declare_parameter('smc_command_timeout', 0.20)
        self.declare_parameter('apf_command_timeout', 0.25)
        self.declare_parameter('covariance_status_timeout', 0.30)
        self.declare_parameter('covariance_revert_timeout', 3.0)
        self.declare_parameter('max_uncertainty_enter_glide_2sigma_m', 0.85)
        self.declare_parameter('max_uncertainty_continue_glide_2sigma_m', 0.95)
        self.declare_parameter('final_descent_rxy_m', 0.85)
        self.declare_parameter('final_descent_alt_m', 0.60)
        self.declare_parameter('final_uncertainty_max_2sigma_m', 0.95)
        self.declare_parameter('takeoff_altitude', 3.0)
        self.declare_parameter('yaw_align_enable', True)
        self.declare_parameter('yaw_align_enter_deg', 20.0)
        self.declare_parameter('yaw_align_exit_deg', 8.0)
        self.declare_parameter('yaw_align_u0_px', 320.0)
        self.declare_parameter('yaw_align_v0_px', 240.0)
        self.declare_parameter('yaw_align_u_tolerance_px', 35.0)
        self.declare_parameter('yaw_align_u_stop_px', 220.0)
        self.declare_parameter('yaw_align_min_speed_scale', 0.60)
        self.declare_parameter('yaw_align_speed_filter_alpha', 0.35)
        self.declare_parameter('reacquire_confirm_time', 0.30)
        self.declare_parameter('reacquire_hold_timeout', 4.00)
        self.declare_parameter('reacquire_retry_delay', 0.75)
        self.declare_parameter('reacquire_lost_timeout', 2.00)
        self.declare_parameter('reacquire_bbox_timeout', 0.80)
        self.declare_parameter('follow_bbox_timeout', 0.40)
        self.declare_parameter('follow_detection_hold_timeout', 0.25)
        self.declare_parameter('landing_tracking_timeout', 0.60)
        self.declare_parameter('reacquire_max_yaw_rate_deg', 20.0)
        self.declare_parameter('follow_entry_hold_time', 0.10)
        self.declare_parameter('search_entry_hold_time', 0.30)
        self.declare_parameter('follow_speed_limit', 2.20)
        self.declare_parameter('follow_forward_heading_gate_deg', 85.0)
        self.declare_parameter('course_yaw_rate_limit', 0.60)
        self.declare_parameter('follow_yaw_motion_min_speed', 0.30)
        self.declare_parameter('follow_yaw_motion_enter_confirm_s', 2.00)
        self.declare_parameter('follow_yaw_motion_exit_confirm_s', 2.00)
        self.declare_parameter('follow_yaw_velocity_filter_tau_s', 0.25)
        self.declare_parameter('follow_yaw_course_consistency_deg', 40.0)
        self.declare_parameter('align_yaw_with_target_motion', False)
        # Landing coast duration is expressed in ROS time so target velocity
        # (estimated against simulation timestamps) is integrated in matching
        # units even when Gazebo runs slower than wall time.
        self.declare_parameter('land_coast_timeout', 5.00)
        self.declare_parameter('follow_coast_timeout', 3.00)
        self.declare_parameter('land_fallback_target_velocity_min_speed', 0.25)
        self.declare_parameter('land_fallback_xy_speed_limit', 1.35)
        self.declare_parameter('land_waveoff_near_ground_alt', 0.20)
        self.declare_parameter('auto_reland_after_abort', False)

        self.follow_dist = self.get_parameter('follow_distance').value
        self.search_timeout = self.get_parameter('search_timeout').value
        self.approach_timeout = self.get_parameter('approach_timeout').value
        self.smc_command_timeout = float(
            self.get_parameter('smc_command_timeout').value
        )
        self.apf_command_timeout = float(
            self.get_parameter('apf_command_timeout').value
        )
        self.covariance_status_timeout = float(
            self.get_parameter('covariance_status_timeout').value
        )
        self.covariance_revert_timeout = float(
            self.get_parameter('covariance_revert_timeout').value
        )
        self.max_uncertainty_enter_glide = float(
            self.get_parameter('max_uncertainty_enter_glide_2sigma_m').value
        )
        self.max_uncertainty_continue_glide = float(
            self.get_parameter('max_uncertainty_continue_glide_2sigma_m').value
        )
        self.final_descent_rxy = float(
            self.get_parameter('final_descent_rxy_m').value
        )
        self.final_descent_alt = float(
            self.get_parameter('final_descent_alt_m').value
        )
        self.final_uncertainty_max = float(
            self.get_parameter('final_uncertainty_max_2sigma_m').value
        )
        self.takeoff_alt = self.get_parameter('takeoff_altitude').value
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
        self.reacquire_retry_delay = max(
            0.0, float(self.get_parameter('reacquire_retry_delay').value)
        )
        self.reacquire_lost_timeout = float(
            self.get_parameter('reacquire_lost_timeout').value
        )
        self.reacquire_bbox_timeout = float(
            self.get_parameter('reacquire_bbox_timeout').value
        )
        self.follow_bbox_timeout = max(
            0.05, float(self.get_parameter('follow_bbox_timeout').value)
        )
        self.follow_detection_hold_timeout = max(
            0.0, float(self.get_parameter('follow_detection_hold_timeout').value)
        )
        self.landing_tracking_timeout = max(
            0.05, float(self.get_parameter('landing_tracking_timeout').value)
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
        self.follow_speed_limit = max(
            0.0, float(self.get_parameter('follow_speed_limit').value)
        )
        self.follow_forward_heading_gate = math.radians(float(np.clip(
            self.get_parameter('follow_forward_heading_gate_deg').value,
            0.0, 90.0,
        )))
        self.course_yaw_rate_limit = max(
            0.0, float(self.get_parameter('course_yaw_rate_limit').value)
        )
        self.follow_yaw_motion_min_speed = max(
            0.0, float(self.get_parameter('follow_yaw_motion_min_speed').value)
        )
        self.follow_yaw_motion_enter_confirm_s = max(
            0.0, float(self.get_parameter('follow_yaw_motion_enter_confirm_s').value)
        )
        self.follow_yaw_motion_exit_confirm_s = max(
            0.0, float(self.get_parameter('follow_yaw_motion_exit_confirm_s').value)
        )
        self.follow_yaw_velocity_filter_tau_s = max(
            0.01, float(self.get_parameter('follow_yaw_velocity_filter_tau_s').value)
        )
        self.follow_yaw_course_consistency_deg = max(
            0.0, float(self.get_parameter('follow_yaw_course_consistency_deg').value)
        )
        self.align_yaw_with_target_motion = bool(
            self.get_parameter('align_yaw_with_target_motion').value
        )
        self.land_coast_timeout = max(
            0.0, float(self.get_parameter('land_coast_timeout').value)
        )
        self.follow_coast_timeout = max(
            0.0, float(self.get_parameter('follow_coast_timeout').value)
        )
        self.land_fallback_min_target_speed = max(
            0.0,
            float(self.get_parameter('land_fallback_target_velocity_min_speed').value),
        )
        self.land_fallback_xy_speed_limit = max(
            0.0, float(self.get_parameter('land_fallback_xy_speed_limit').value)
        )
        self.land_waveoff_near_ground_alt = max(
            0.0, float(self.get_parameter('land_waveoff_near_ground_alt').value)
        )
        self.auto_reland_after_abort = bool(
            self.get_parameter('auto_reland_after_abort').value
        )

        # ── State ────────────────────────────────────────────────────────
        self.phase = MissionPhase.IDLE
        self.detected = False
        self.tracking_mode = 'EXPIRED'
        self.drone_pos = np.zeros(3)
        self.drone_vel = np.zeros(3)
        self.drone_yaw = 0.0
        self.drone_yaw_rate = 0.0
        self.last_odom_time = 0.0
        self.target_pos = np.zeros(3)
        self.target_vel = np.zeros(3)
        self.has_odom = False
        self.has_target = False

        self.last_target_time = 0.0
        self.land_requested = False
        self.last_detection_time = 0.0

        # APF + IBVS outputs
        self.apf_velocity = np.zeros(3)  # Combined APF command used in APPROACH
        self.apf_guidance_velocity = np.zeros(3)
        self.apf_target_velocity_ff = np.zeros(2)
        self.last_apf_time = 0.0
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
        self.reacquire_retry_after = 0.0
        self.follow_entry_hold_until = 0.0
        self.follow_forward_blocked = False
        self.follow_coast_active = False
        self.follow_last_confirmed_target_time = 0.0
        self.follow_last_confirmed_target_pos: Optional[np.ndarray] = None
        self.follow_last_confirmed_target_vel: Optional[np.ndarray] = None
        self.search_entry_hold_yaw = 0.0
        self.search_entry_hold_until = 0.0

        # Precision landing & Touchdown state
        self.touchdown_detected = False
        self.safe_to_land = False
        self.has_smc_cmd = False
        self.smc_velocity = np.zeros(3)
        self.smc_yaw_rate = 0.0
        self.last_smc_time = 0.0
        self.last_smc_phase_time = 0.0
        self.smc_sub_phase = 'HOLD'
        self.rswitch_adaptive = 3.0
        self.uncertainty_radius = float('inf')
        self.continue_uncertainty_max = self.max_uncertainty_continue_glide
        self.last_safe_to_land_time = 0.0
        self.last_uncertainty_time = 0.0
        self.landing_glide_active = False
        self.landing_gate_lost_since = 0.0
        self.landing_visual_loss_active = False
        self.landing_hold_yaw = 0.0
        self.landing_hold_initialized = False
        self.landing_course_heading_active = False
        self.course_yaw_setpoint = 0.0
        self.course_yaw_initialized = False
        self.course_yaw_update_time = 0.0
        self.follow_yaw_selector = FollowYawReferenceSelector(
            enter_speed=max(0.35, self.follow_yaw_motion_min_speed),
            exit_speed=0.18,
            enter_confirm_duration_s=self.follow_yaw_motion_enter_confirm_s,
            # Keep moving-course heading through short camera/EKF dropouts and
            # velocity-estimate dips during a turn.
            exit_confirm_duration_s=self.follow_yaw_motion_exit_confirm_s,
            velocity_filter_time_constant_s=self.follow_yaw_velocity_filter_tau_s,
            course_consistency_deg=self.follow_yaw_course_consistency_deg,
        )
        self.follow_yaw_source = ''
        self.mission_completed = False
        # Coast state: last confirmed target position/velocity for dead-reckoning
        # when camera tracking is briefly lost near touchdown.
        self.last_confirmed_target_pos: np.ndarray = np.zeros(3)
        self.last_confirmed_target_vel: np.ndarray = np.zeros(3)
        self.last_confirmed_target_time: float = 0.0
        self.last_target_sim_time: float = 0.0
        self.land_coast_active: bool = False

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
            Twist, '/apf/guidance_velocity_cmd', self.apf_guidance_vel_cb, 10,
        )
        self.create_subscription(
            Twist, '/apf/target_velocity_ff', self.apf_target_ff_cb, 10,
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
            Float64, '/landing/uncertainty_radius', self.uncertainty_cb, 10,
        )
        self.create_subscription(
            Float64, '/landing/rswitch_adaptive', self.rswitch_cb, 10,
        )
        self.create_subscription(
            String, '/landing/sub_phase', self.smc_phase_cb, 10,
        )
        self.create_subscription(
            Twist, '/landing/velocity_cmd', self.smc_vel_cb, 10,
        )
        self.create_subscription(
            Bool, '/safety/manual_override', self.manual_override_cb, 10,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.phase_pub = self.create_publisher(String, '/mission/phase', 10)
        self.vel_pub = self.create_publisher(
            Twist, '/mission/velocity_setpoint', 10,
        )
        self.yaw_pub = self.create_publisher(
            Float64, '/mission/yaw_setpoint', 10,
        )
        self.follow_yaw_source_pub = self.create_publisher(
            String, '/mission/follow_yaw_source', 10,
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
        landing_stage = (
            f"{self.smc_sub_phase}/σ2={self.uncertainty_radius:.2f}m"
            if self.phase in (MissionPhase.APPROACH, MissionPhase.LAND)
            else "-"
        )
        self.get_logger().info(
            f"[STATUS] Phase: {self.phase.value:<7} | "
            f"Drone: ({self.drone_pos[0]:.1f}, {self.drone_pos[1]:.1f}, {self.drone_pos[2]:.1f})m | "
            f"Target: ({self.target_pos[0]:.1f}, {self.target_pos[1]:.1f}) | "
            f"Dist: {dist_str} | "
            f"EKF: {self.tracking_mode} | Det: {self.detected} | "
            f"BBox: {bbox_str} | XY scale: {self.xy_speed_scale:.2f} | "
            f"Reacq: {self.reacquire_active} | Landing: {landing_stage}"
        )

    # ── Input callbacks ──────────────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        now = time.monotonic()
        p = msg.pose.pose.position
        self.drone_pos = np.array([p.x, p.y, p.z])
        v = msg.twist.twist.linear
        self.drone_vel = np.array([v.x, v.y, v.z], dtype=float)
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
        v = msg.twist.twist.linear
        self.target_vel = np.array([v.x, v.y, v.z])
        self.has_target = True
        self.last_target_time = time.monotonic()
        stamp = msg.header.stamp
        msg_sim_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if msg_sim_time > 0.0:
            self.last_target_sim_time = msg_sim_time
        else:
            self.last_target_sim_time = self._simulation_time_now()

    def _simulation_time_now(self) -> float:
        """Return the ROS clock time used by target state and velocity."""
        return max(0.0, float(self.get_clock().now().nanoseconds) * 1e-9)


    def detected_cb(self, msg: Bool):
        self.detected = msg.data
        if msg.data:
            now = time.monotonic()
            self.last_detection_time = now
            # Detection is an event, not something to wait for the 20 Hz FSM
            # loop to notice. Latch the current heading here so the next
            # published setpoint immediately brakes the search sweep.
            if (
                self.phase == MissionPhase.SEARCH
                and not self.reacquire_active
                and now >= self.reacquire_retry_after
            ):
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
        if msg.data:
            valid = self._landing_tracking_is_valid()
            self.get_logger().info(
                f'📥 [LAND CMD] Nhận lệnh hạ cánh (/operator/land_command=True) | '
                f'Pha hiện tại: {self.phase.name} | Tracking Lock: {valid}'
            )
            if not valid and self.phase == MissionPhase.SEARCH:
                self.get_logger().warning(
                    '[LAND CMD] Command queued; holding/searching until a stable '
                    'target lock is reacquired.'
                )

    def manual_override_cb(self, msg: Bool):
        if msg.data and self.phase != MissionPhase.IDLE:
            self.get_logger().error(
                '🚨 [FSM SAFETY] PHÁT HIỆN CAN THIỆP TỪ PHI CÔNG / QGC / RC! '
                'Lập tức chuyển FSM sang IDLE và giải phóng quyền điều khiển!'
            )
            self.phase = MissionPhase.IDLE
            self.land_requested = False
            self.landing_glide_active = False
            self.landing_hold_initialized = False
            self.reacquire_active = False

    def apf_vel_cb(self, msg: Twist):
        self.apf_velocity = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
        ])
        self.last_apf_time = time.monotonic()

    def apf_guidance_vel_cb(self, msg: Twist):
        self.apf_guidance_velocity = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
        ])

    def apf_target_ff_cb(self, msg: Twist):
        self.apf_target_velocity_ff = np.array([
            msg.linear.x, msg.linear.y,
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
        self.last_safe_to_land_time = time.monotonic()

    def uncertainty_cb(self, msg: Float64):
        self.uncertainty_radius = float(msg.data)
        self.last_uncertainty_time = time.monotonic()

    def rswitch_cb(self, msg: Float64):
        self.rswitch_adaptive = max(0.0, float(msg.data))

    def smc_phase_cb(self, msg: String):
        self.smc_sub_phase = msg.data
        self.last_smc_phase_time = time.monotonic()

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
        if self.drone_pos[2] >= 2.5 and not self.mission_completed:
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
        if (
            self.detected
            and not self.reacquire_active
            and now >= self.reacquire_retry_after
        ):
            self._start_reacquire(now)

        if not self.reacquire_active:
            if self.land_requested and self._landing_tracking_is_valid(now):
                self._begin_approach()
            return

        if self.detected:
            self.reacquire_last_detection = now
        detection_lost = reacquire_should_resume_search(
            detected=self.detected,
            now=now,
            last_detection_time=self.reacquire_last_detection,
            lost_timeout=self.reacquire_lost_timeout,
        )
        detection_recent = not detection_lost
        if detection_lost:
            self.reacquire_active = False
            self.reacquire_stable_since = 0.0
            self.reacquire_timeout_warned = False
            self.reacquire_retry_after = now
            self.get_logger().info(
                '[REACQUIRE] detection lost before confirmation; resume SEARCH'
            )
            return

        # Once the fixed yaw target has been sent, verify target tracking and
        # bounded yaw rate so the vehicle transitions cleanly to FOLLOW.
        max_u = max(632.0, 2.0 * self.yaw_align_u0 - 8.0)
        max_v = max(472.0, 2.0 * self.yaw_align_v0 - 8.0)
        bbox_in_roi = (
            8.0 <= self.bbox_u <= max_u
            and 8.0 <= self.bbox_v <= max_v
        )
        # A detection that arrived before the sweep was stopped cannot confirm
        # reacquisition.  Wait for a new image plus an accepted EKF update so
        # the first FOLLOW command is based on the view that actually locked.
        stable_candidate = reacquire_candidate_is_valid(
            now=now,
            reacquire_started=self.reacquire_started,
            last_bbox_time=self.last_bbox_time if self.has_bbox else 0.0,
            bbox_timeout=self.reacquire_bbox_timeout,
            bbox_in_roi=bbox_in_roi,
            detected=self.detected,
            tracking_mode=self.tracking_mode,
            yaw_rate=self.drone_yaw_rate,
            max_yaw_rate=self.reacquire_max_yaw_rate,
            detection_recent=detection_recent,
        )

        if stable_candidate:
            if self.reacquire_stable_since <= 0.0:
                self.reacquire_stable_since = now
        else:
            self.reacquire_stable_since = 0.0

        confirmed = (
            self.reacquire_stable_since > 0.0
            and stable_candidate
            and now - self.reacquire_stable_since >= self.reacquire_confirm_time
        )
        if confirmed:
            self.reacquire_active = False
            self.reacquire_retry_after = 0.0
            if self.land_requested and self._landing_tracking_is_valid(now):
                self._begin_approach()
                self.get_logger().info(
                    '[REACQUIRE] stable lock restored with LAND pending; '
                    'entering APPROACH.'
                )
            else:
                self.phase = MissionPhase.FOLLOW
                self.follow_entry_hold_until = now + self.follow_entry_hold_time
                self.follow_entry_hold_yaw = self.drone_yaw
                self.course_yaw_initialized = False
                self.course_yaw_update_time = now
                self.follow_yaw_selector.reset(self.drone_yaw)
                self.follow_yaw_source = ''
                self.get_logger().info(
                    f'[REACQUIRE] confirmed after '
                    f'{now - self.reacquire_started:.2f}s '
                    f'| bbox=({self.bbox_u:.0f},{self.bbox_v:.0f}) '
                    f'| EKF={self.tracking_mode}; entering FOLLOW'
                )
        elif now - self.reacquire_started > self.reacquire_hold_timeout:
            # A raw detector hit can persist while EKF rejects its pose. Retry
            # after a short cooldown instead of hovering forever with the yaw
            # sweep permanently latched off.
            self.reacquire_active = False
            self.reacquire_stable_since = 0.0
            self.reacquire_retry_after = now + self.reacquire_retry_delay
            self.reacquire_timeout_warned = False
            self.get_logger().warning(
                '[REACQUIRE] no accepted EKF lock before timeout; retrying '
                'after cooldown while maintaining visual centering.'
            )

    def _start_reacquire(self, now: float):
        """Latch a fixed yaw immediately on the first SEARCH detection."""
        self.reacquire_active = True
        self.reacquire_started = now
        self.reacquire_retry_after = 0.0
        self.reacquire_stable_since = 0.0
        self.reacquire_last_detection = now
        self.reacquire_hold_yaw = self.drone_yaw
        self.reacquire_timeout_warned = False
        self.get_logger().info(
            f'[REACQUIRE] detection trigger; stop sweep and hold yaw '
            f'| yaw={math.degrees(self.reacquire_hold_yaw):.1f}° '
            f'| bbox=({self.bbox_u:.0f},{self.bbox_v:.0f})'
        )

    def _begin_approach(self):
        """Latch the operator's landing request after stable target lock."""
        previous_phase = self.phase
        self.phase = MissionPhase.APPROACH
        self.follow_coast_active = False
        self.landing_glide_active = False
        self.landing_gate_lost_since = 0.0
        self.landing_visual_loss_active = False
        target_speed = float(np.linalg.norm(self.target_vel[:2]))
        self.landing_course_heading_active = (
            self.has_target and target_speed >= self.land_fallback_min_target_speed
        )
        if self.landing_course_heading_active:
            # Keep the moving-pad course from FOLLOW through APPROACH. Switching
            # straight back to IBVS bearing here can command a large yaw reversal
            # when the camera is pitched down and sees the pad off-axis.
            if previous_phase == MissionPhase.SEARCH:
                # SEARCH may have changed the aircraft heading while reacquiring;
                # seed the course filter from the current body heading.
                self.course_yaw_setpoint = _wrap_angle(self.drone_yaw)
                self.course_yaw_initialized = True
                self.course_yaw_update_time = time.monotonic()
            if self.course_yaw_initialized:
                self.landing_hold_yaw = self.course_yaw_setpoint
            else:
                self.landing_hold_yaw = _wrap_angle(follow_motion_yaw(
                    self.target_vel[:2], self.drone_yaw,
                    min_speed=self.land_fallback_min_target_speed,
                ))
                self.course_yaw_setpoint = self.landing_hold_yaw
                self.course_yaw_initialized = True
                self.course_yaw_update_time = time.monotonic()
            self.landing_hold_initialized = True
            self.get_logger().info(
                '[LANDING] Carrying moving-target course heading into APPROACH '
                f'| course={math.degrees(self.landing_hold_yaw):.1f}° '
                f'| target_speed={target_speed:.2f}m/s.'
            )
        else:
            self.landing_hold_yaw = _wrap_angle(
                self.ibvs_yaw if self.has_odom else self.drone_yaw
            )
            self.landing_hold_initialized = False

    def _update_course_yaw_reference(self, target_yaw: float, now: float | None = None) -> float:
        """Rate-limit course reference updates before Offboard's yaw slew cap."""
        now = time.monotonic() if now is None else float(now)
        if not self.course_yaw_initialized:
            self.course_yaw_setpoint = _wrap_angle(self.drone_yaw)
            self.course_yaw_initialized = True
            dt = 0.0
        else:
            dt = min(0.10, max(0.0, now - self.course_yaw_update_time))
        self.course_yaw_setpoint = _wrap_angle(slew_yaw_reference(
            self.course_yaw_setpoint,
            target_yaw,
            self.course_yaw_rate_limit,
            dt,
        ))
        self.course_yaw_update_time = now
        return self.course_yaw_setpoint

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
                self.reacquire_active = False
                self.reacquire_retry_after = 0.0
                self.follow_coast_active = False
                self.follow_last_confirmed_target_time = 0.0
                self.get_logger().warning(
                    f'Target lost for {age:.1f}s, returning to SEARCH.'
                )
                return

        # Check for operator LAND command
        if self.land_requested:
            # A land command received during SEARCH/reacquisition is latched,
            # but it cannot bypass the stable visual/EKF lock handshake.
            if not self._landing_tracking_is_valid():
                return
            self._begin_approach()
            # A landing run uses the current vehicle heading as a fixed
            # reference. Pixel centering is handled by translation and the
            # gimbal; chasing pixel yaw while the camera looks steeply down
            # turns the aircraft around the pad.
            self.get_logger().info('Operator LAND command received.')

    def _handle_approach(self):
        """APF approach, then covariance-gated SMC glide and final descent."""
        now = time.monotonic()

        # Touchdown confirmed during approach completes the mission immediately
        if self.touchdown_detected:
            self.phase = MissionPhase.IDLE
            self.land_requested = False
            self.touchdown_detected = False
            self.landing_hold_initialized = False
            self.landing_glide_active = False
            self.mission_completed = True
            self.get_logger().info('🏆 Precision landing completed! Drone safely stopped on landing pad.')
            return

        # Check if drone is already at terminal landing envelope directly over pad
        h_err = float(np.linalg.norm(
            self.drone_pos[:2] - self.target_pos[:2]
        )) if self.has_target and self.has_odom else float('inf')
        rel_alt = float(abs(self.drone_pos[2] - self.target_pos[2])) if self.has_target and self.has_odom else (self.drone_pos[2] if self.has_odom else float('inf'))
        is_terminal = bool(self.has_odom and (self.drone_pos[2] <= self.final_descent_alt or rel_alt <= self.final_descent_alt) and h_err <= self.final_descent_rxy)

        # Cancel approach if the live visual lock has been absent too long at higher altitudes.
        # If already at terminal landing altitude (<= final_descent_alt over pad), commit to landing instead of aborting!
        age = now - self.last_detection_time if self.last_detection_time > 0.0 else float('inf')
        if not self._landing_tracking_is_valid(now) and age > self.approach_timeout:
            if is_terminal:
                self.phase = MissionPhase.LAND
                self.touchdown_detected = False
                self.get_logger().info(
                    f'[LANDING] Terminal altitude reached (z={self.drone_pos[2]:.2f}m <= {self.final_descent_alt:.2f}m): '
                    'committing to terminal touchdown descent.'
                )
                return
            self.phase = MissionPhase.FOLLOW
            if not self.auto_reland_after_abort:
                self.land_requested = False
            self.follow_entry_hold_yaw = self.drone_yaw
            self.follow_entry_hold_until = now + self.follow_entry_hold_time
            self.course_yaw_setpoint = self.drone_yaw
            self.course_yaw_initialized = True
            self.follow_yaw_selector.reset(self.drone_yaw)
            self.landing_glide_active = False
            self.landing_gate_lost_since = 0.0
            self.landing_hold_initialized = False
            self.get_logger().warning(
                f'Target visual lock lost during APPROACH for {age:.1f}s, '
                f'returning to FOLLOW (auto_reland={self.auto_reland_after_abort}).'
            )
            return

        # Cancel approach if operator retracts LAND command
        if not self.land_requested:
            self.phase = MissionPhase.FOLLOW
            self.landing_glide_active = False
            self.landing_gate_lost_since = 0.0
            self.landing_hold_initialized = False
            self.get_logger().info('Operator cancelled LAND, returning to FOLLOW.')
            return

        if not self.has_target or not self.has_odom:
            return

        horizontal_error = float(np.linalg.norm(
            self.drone_pos[:2] - self.target_pos[:2]
        ))
        vertical_error = float(abs(self.drone_pos[2] - self.target_pos[2]))
        tracking_valid = self._landing_tracking_is_valid(now)
        smc_fresh = self._smc_command_is_fresh(now)
        covariance_fresh = self._covariance_status_is_fresh(now)
        gate_ok = (
            tracking_valid
            and self.safe_to_land
            and covariance_fresh
            and self.uncertainty_radius <= self.max_uncertainty_continue_glide
        )
        entry_gate_ok = (
            gate_ok
            and self.uncertainty_radius <= self.max_uncertainty_enter_glide
        )

        if self.landing_glide_active and not gate_ok:
            if self.landing_gate_lost_since <= 0.0:
                self.landing_gate_lost_since = now
            elif now - self.landing_gate_lost_since >= self.covariance_revert_timeout:
                self.landing_glide_active = False
                self.landing_hold_initialized = False
                self.landing_gate_lost_since = 0.0
                self.get_logger().warning(

                    '[LANDING] Covariance/vision gate failed for '
                    f'{self.covariance_revert_timeout:.1f}s; reverting to APF approach.'
                )
        elif gate_ok:
            self.landing_gate_lost_since = 0.0

        # APF_APPROACH -> GLIDE_SLOPE: current visible marker, covariance
        # authorization, adaptive range reached, and a live SMC solution.
        if (
            not self.landing_glide_active
            and entry_gate_ok
            and horizontal_error <= self.rswitch_adaptive
            and smc_fresh
            and self.smc_sub_phase in ('GLIDE_SLOPE', 'FINAL_DESCENT')
        ):
            self.landing_glide_active = True
            self.get_logger().info(
                f'[LANDING] Entering SMC glide slope at Rxy={horizontal_error:.2f}m '
                f'(Rswitch={self.rswitch_adaptive:.2f}m, '
                f'uncertainty={self.uncertainty_radius:.3f}m).'
            )

        # GLIDE_SLOPE -> FINAL_DESCENT/LAND: all geometry and uncertainty
        # conditions from the landing plan must hold at the same time.
        final_ready = (
            self.landing_glide_active
            and gate_ok
            and smc_fresh
            and self.smc_sub_phase in ('FINAL_DESCENT', 'FINAL_GATE_WAIT')
            and horizontal_error <= self.final_descent_rxy
            and vertical_error <= self.final_descent_alt
            and self.uncertainty_radius <= self.final_uncertainty_max
        )
        if final_ready:
            self.phase = MissionPhase.LAND
            if not self.landing_course_heading_active:
                self.landing_hold_yaw = _wrap_angle(
                    self.ibvs_yaw if tracking_valid else self.drone_yaw
                )
                self.landing_hold_initialized = True
            self.touchdown_detected = False
            self.get_logger().info(
                '[LANDING] Final descent authorized: '
                f'Rxy={horizontal_error:.2f}m, Rz={vertical_error:.2f}m, '
                f'uncertainty={self.uncertainty_radius:.3f}m.'
            )

    def _handle_land(self):
        """Remain in gated final descent until touchdown is confirmed, or wave-off."""
        now = time.monotonic()
        sim_now = self._simulation_time_now()
        if self.touchdown_detected:
            self.phase = MissionPhase.IDLE
            self.land_requested = False
            self.touchdown_detected = False
            self.landing_hold_initialized = False
            self.landing_glide_active = False
            self.mission_completed = True
            self.get_logger().info('🏆 Precision landing completed! Drone safely stopped on landing pad.')
            return

        tracking_valid = self._landing_tracking_is_valid(now)
        h_err = float(np.linalg.norm(
            self.drone_pos[:2] - self.target_pos[:2]
        )) if self.has_target and self.has_odom else float('inf')

        # Update coasting state: latch the last confirmed target position for
        # dead-reckoning while tracking is briefly interrupted near touchdown.
        if tracking_valid and self.has_target:
            self.last_confirmed_target_pos = self.target_pos.copy()
            self.last_confirmed_target_vel = self.target_vel.copy() if self.target_vel is not None else np.zeros(3)
            self.last_confirmed_target_time = (
                self.last_target_sim_time or self._simulation_time_now()
            )
            self.land_coast_active = False
        elif (
            not tracking_valid
            and self.last_confirmed_target_time > 0.0
            and sim_now - self.last_confirmed_target_time <= self.land_coast_timeout
        ):
            self.land_coast_active = True
        else:
            self.land_coast_active = False

        # Safety Wave-off / Abort:
        # When very close to ground (< land_waveoff_near_ground_alt), suppress
        # wave-off to avoid aborting during the disarm confirmation buffer window.
        # This is safe because: (a) we are already nearly landed, (b) the coast
        # dead-reckoning continues to guide the drone to the last known position.
        near_ground = self.has_odom and self.drone_pos[2] <= max(0.35, self.land_waveoff_near_ground_alt)
        coast_valid = self.land_coast_active  # brief tracking loss, still coasting

        if near_ground:
            self.landing_gate_lost_since = 0.0
        elif not tracking_valid and not coast_valid:
            if self.landing_gate_lost_since <= 0.0:
                self.landing_gate_lost_since = now
            elif now - self.landing_gate_lost_since >= 1.5:
                if not self.auto_reland_after_abort:
                    self.land_requested = False
                    self.phase = MissionPhase.FOLLOW
                    self.follow_entry_hold_yaw = self.drone_yaw
                    self.follow_entry_hold_until = now + self.follow_entry_hold_time
                    self.course_yaw_setpoint = self.drone_yaw
                    self.course_yaw_initialized = True
                    self.follow_yaw_selector.reset(self.drone_yaw)
                else:
                    self.phase = MissionPhase.APPROACH
                self.landing_glide_active = False
                self.landing_gate_lost_since = 0.0
                self.get_logger().warning(
                    f'[LANDING] Lost tracking in LAND phase for > 1.5s (coast expired); waving off (auto_reland={self.auto_reland_after_abort}).'
                )
        elif not tracking_valid and coast_valid:
            # Coasting: tracking briefly lost but dead-reckoning still valid
            coast_age = sim_now - self.last_confirmed_target_time
            if self.landing_gate_lost_since <= 0.0:
                self.landing_gate_lost_since = now
                self.get_logger().info(
                    f'[LANDING] Tracking lost — coasting to last known target pos '
                    f'(coast timeout={self.land_coast_timeout:.2f}s).'
                )
            _ = coast_age  # log/debug value; the active gate uses ROS time above
        elif h_err > 1.25 and not near_ground:
            if not self.auto_reland_after_abort:
                self.land_requested = False
                self.phase = MissionPhase.FOLLOW
                self.follow_entry_hold_yaw = self.drone_yaw
                self.follow_entry_hold_until = now + self.follow_entry_hold_time
                self.course_yaw_setpoint = self.drone_yaw
                self.course_yaw_initialized = True
                self.follow_yaw_selector.reset(self.drone_yaw)
            else:
                self.phase = MissionPhase.APPROACH
            self.landing_glide_active = False
            self.landing_gate_lost_since = 0.0
            self.get_logger().warning(
                f'[LANDING] Target moved away (Herr={h_err:.2f}m > 1.25m) during LAND; waving off (auto_reland={self.auto_reland_after_abort}).'
            )
        else:
            self.landing_gate_lost_since = 0.0

    # ── Setpoint composition ─────────────────────────────────────────────

    def _bbox_is_recent(
        self, now: float | None = None, timeout: float | None = None,
    ) -> bool:
        now = time.monotonic() if now is None else now
        timeout = self.reacquire_bbox_timeout if timeout is None else timeout
        return (
            self.has_bbox
            and self.last_bbox_time > 0.0
            and now - self.last_bbox_time <= timeout
        )

    def _detection_is_recent(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return (
            self.detected
            or (
                self.last_detection_time > 0.0
                and now - self.last_detection_time <= self.follow_detection_hold_timeout
            )
        )

    def _tracking_input_is_valid(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return (
            self.has_odom
            and self.tracking_mode in ('TRACKING', 'PREDICTING', 'PREDICTING_DEGRADED')
            and self._bbox_is_recent(now, self.follow_bbox_timeout)
            and self._detection_is_recent(now)
        )

    def _landing_tracking_is_valid(self, now: float | None = None) -> bool:
        """Require a current camera detection and accepted, fresh state for landing."""
        now = time.monotonic() if now is None else now
        return (
            self.has_odom
            and self.has_target
            and self.tracking_mode == 'TRACKING'
            and self._bbox_is_recent(now, self.landing_tracking_timeout)
            and now - self.last_odom_time <= self.landing_tracking_timeout
            and now - self.last_target_time <= self.landing_tracking_timeout
        )

    def _smc_command_is_fresh(self, now: float | None = None) -> bool:
        """A recently published HOLD is not an active landing guidance command."""
        now = time.monotonic() if now is None else now
        return (
            self.has_smc_cmd
            and self.last_smc_time > 0.0
            and now - self.last_smc_time <= self.smc_command_timeout
            and self.last_smc_phase_time > 0.0
            and now - self.last_smc_phase_time <= self.smc_command_timeout
            and self.smc_sub_phase not in ('HOLD', 'INACTIVE')
        )

    def _covariance_status_is_fresh(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return (
            self.last_safe_to_land_time > 0.0
            and self.last_uncertainty_time > 0.0
            and now - self.last_safe_to_land_time <= self.covariance_status_timeout
            and now - self.last_uncertainty_time <= self.covariance_status_timeout
        )

    def _landing_yaw_setpoint(self, tracking_valid: bool) -> float:
        """Hold heading or align with target motion through glide slope and touchdown."""
        target_vel_xy = (
            (float(self.target_vel[0]), float(self.target_vel[1]))
            if (self.has_target and hasattr(self, 'target_vel') and self.target_vel is not None)
            else None
        )

        if self.landing_course_heading_active:
            # Track the filtered pad course while it is confidently moving;
            # hold the last course through low-speed estimates or brief visual
            # dropouts. OffboardCommander applies the configured yaw slew cap.
            if (
                tracking_valid
                and target_vel_xy is not None
                and float(np.linalg.norm(target_vel_xy)) >= self.land_fallback_min_target_speed
            ):
                target_course = follow_motion_yaw(
                    target_vel_xy,
                    self.landing_hold_yaw,
                    min_speed=self.land_fallback_min_target_speed,
                )
                self.landing_hold_yaw = self._update_course_yaw_reference(
                    target_course,
                )
            elif self.course_yaw_initialized:
                self.landing_hold_yaw = self.course_yaw_setpoint
            self.landing_hold_initialized = True
            self.landing_visual_loss_active = not tracking_valid
            return self.landing_hold_yaw

        # Initial standoff approach at cruise altitude: keep camera visually aligned with target only when far away (>1.0m)
        if self.phase == MissionPhase.APPROACH and tracking_valid and not self.landing_glide_active:
            h_err = (
                float(np.linalg.norm(self.drone_pos[:2] - self.target_pos[:2]))
                if (self.has_target and self.has_odom) else float('inf')
            )
            if h_err > 1.0:
                self.landing_hold_yaw = _wrap_angle(float(self.ibvs_yaw))
                self.landing_visual_loss_active = False
                return self.landing_hold_yaw

        yaw, self.landing_hold_yaw, self.landing_hold_initialized = choose_landing_yaw(
            tracking_valid=tracking_valid,
            ibvs_yaw=self.ibvs_yaw,
            drone_yaw=self.drone_yaw,
            held_yaw=self.landing_hold_yaw,
            hold_initialized=self.landing_hold_initialized,
            target_velocity_xy=target_vel_xy,
            align_with_target_motion=self.align_yaw_with_target_motion,
        )
        self.landing_visual_loss_active = not tracking_valid
        return yaw


    def _translation_speed_scale(self) -> float:
        """Smoothly scale APF translation while IBVS aligns the view.

        IBVS owns yaw and gimbal pitch; APF owns world-frame translation.
        A hard zero-velocity gate can deadlock the system because vertical
        image error is primarily corrected by the gimbal, not by yaw/APF.
        Invalid tracking is still an immediate safety stop.
        """
        valid_tracking = self._tracking_input_is_valid()
        if not valid_tracking:
            if self.phase == MissionPhase.FOLLOW and self.follow_coast_active:
                raw_scale = 1.0
            else:
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
                span = max(1.0, self.yaw_align_u_stop - self.yaw_align_u_tol)
                progress = (eu - self.yaw_align_u_tol) / span
                # Smooth S-curve transition eliminates slope jumps that kick pitch/roll
                s_curve = 0.5 * (1.0 - math.cos(math.pi * progress))
                pixel_scale = 1.0 - s_curve * (
                    1.0 - self.yaw_align_min_speed
                )

            # Camera-to-target yaw alignment: ensure drone heading points toward
            # the target (IBVS yaw) to maintain camera lock. Do not check motion_xy
            # heading because multirotors can translate laterally (crab) to evade
            # obstacles while keeping the camera firmly locked on the target.
            yaw_error = abs(_wrap_angle(self.ibvs_yaw - self.drone_yaw))
            if yaw_error <= self.yaw_align_exit:
                yaw_scale = 1.0
            elif yaw_error >= self.yaw_align_enter:
                yaw_scale = self.yaw_align_min_speed
            else:
                span = max(1e-3, self.yaw_align_enter - self.yaw_align_exit)
                progress = (yaw_error - self.yaw_align_exit) / span
                s_curve = 0.5 * (1.0 - math.cos(math.pi * progress))
                yaw_scale = 1.0 - s_curve * (
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
            # During reacquisition, IBVS actively centers the target.
            now = time.monotonic()
            if now < self.search_entry_hold_until:
                # Do not publish stale IBVS FOLLOW yaw during the first
                # SEARCH cycle.  IBVS resets asynchronously on phase change.
                search_yaw = self.search_entry_hold_yaw
            else:
                search_yaw = self.ibvs_yaw
            yaw.data = float(search_yaw)

        elif self.phase == MissionPhase.FOLLOW:
            # Scale APF position/obstacle guidance while preserving the
            # measured target velocity feedforward. Scaling both together made
            # the vehicle move slower than a 0.5 m/s pad whenever yaw/pixel
            # alignment was active.
            speed_scale = self._translation_speed_scale()
            now = time.monotonic()
            entry_hold = now < self.follow_entry_hold_until
            apf_fresh = (
                self.last_apf_time > 0.0
                and now - self.last_apf_time <= self.apf_command_timeout
            )
            tracking_valid = self._tracking_input_is_valid(now)
            if tracking_valid and self.has_target:
                self.follow_last_confirmed_target_time = now
                self.follow_last_confirmed_target_pos = self.target_pos.copy()
                self.follow_last_confirmed_target_vel = (
                    self.target_vel.copy() if self.target_vel is not None else np.zeros(3)
                )
                self.follow_coast_active = False
            elif (
                not tracking_valid
                and self.follow_last_confirmed_target_time > 0.0
                and now - self.follow_last_confirmed_target_time <= self.follow_coast_timeout
            ):
                self.follow_coast_active = True
            else:
                self.follow_coast_active = False

            if not entry_hold and apf_fresh:
                follow_xy = compose_follow_velocity(
                    self.apf_guidance_velocity[:2],
                    self.apf_target_velocity_ff,
                    speed_scale,
                    self.follow_speed_limit,
                )
            else:
                follow_xy = np.zeros(2, dtype=float)

            if not tracking_valid and not self.follow_coast_active:
                follow_xy[:] = 0.0
            elif not tracking_valid and self.follow_coast_active:
                coast_dt = now - self.follow_last_confirmed_target_time
                decay = max(0.50, 1.0 - 0.50 * (coast_dt / self.follow_coast_timeout))
                follow_xy *= decay

            target_state_fresh = (
                self.has_target
                and self.last_target_time > 0.0
                and now - self.last_target_time <= self.follow_bbox_timeout
            )
            # APF position corrections are deliberately excluded from yaw:
            # around a static pad they may point back and forth. The selector
            # confirms motion from EKF velocity and holds its source through
            # brief prediction frames to avoid a course/IBVS yaw reversal.
            effective_tracking = tracking_valid or self.follow_coast_active
            follow_yaw, yaw_source = self.follow_yaw_selector.update(
                ibvs_yaw=self.ibvs_yaw,
                target_velocity_xy=self.target_vel[:2],
                current_yaw=self.drone_yaw,
                previous_reference_yaw=(
                    self.course_yaw_setpoint
                    if self.course_yaw_initialized else self.drone_yaw
                ),
                tracking_valid=effective_tracking,
                tracking_mode=self.tracking_mode,
                target_state_fresh=target_state_fresh or self.follow_coast_active,
                target_sample_time=self.last_target_time,
            )

            if yaw_source != self.follow_yaw_source:
                self.follow_yaw_source = yaw_source
                self.get_logger().info(
                    f'[FOLLOW][YAW] source={yaw_source} '
                    f'| target_speed={np.linalg.norm(self.target_vel[:2]):.2f}m/s '
                    f'| reference={math.degrees(follow_yaw):.1f}°'
                )
            self.follow_yaw_source_pub.publish(String(data=yaw_source))
            follow_yaw = self._update_course_yaw_reference(follow_yaw, now)
            # Camera FOV lock: ensure commanded yaw never rotates the camera away from marker.
            # RealSense D435i horizontal half-FOV is ~34.5 deg; enforce a strict 12.0 deg clamp
            # to guarantee the marker remains firmly centered in the camera image at all times,
            # leaving a massive >22 deg safety buffer against roll/pitch banking maneuvers.
            if effective_tracking:
                max_fov_margin = math.radians(12.0)
                dev = _wrap_angle(follow_yaw - self.ibvs_yaw)
                if abs(dev) > max_fov_margin:
                    follow_yaw = _wrap_angle(self.ibvs_yaw + math.copysign(max_fov_margin, dev))
                self.course_yaw_setpoint = follow_yaw

            # Forward flight sector guard with lateral evasion preservation
            course_speed = float(np.linalg.norm(follow_xy))
            if course_speed > 1e-4:
                course_angle = math.atan2(follow_xy[1], follow_xy[0])
                course_err = abs(_wrap_angle(course_angle - self.drone_yaw))
                if course_err > self.follow_forward_heading_gate:
                    # Decompose into body forward and lateral components:
                    # Suppress backward translation while preserving full lateral evasion
                    body_fwd = np.array([math.cos(self.drone_yaw), math.sin(self.drone_yaw)])
                    body_lat = np.array([-math.sin(self.drone_yaw), math.cos(self.drone_yaw)])
                    v_fwd = float(np.dot(follow_xy, body_fwd))
                    v_lat = float(np.dot(follow_xy, body_lat))
                    guarded_follow_xy = max(0.0, v_fwd) * body_fwd + v_lat * body_lat
                else:
                    guarded_follow_xy = follow_xy.copy()
            else:
                guarded_follow_xy = follow_xy.copy()

            forward_blocked = (
                course_speed > 1e-3
                and float(np.linalg.norm(guarded_follow_xy)) < 1e-3
            )
            if forward_blocked != self.follow_forward_blocked:
                self.follow_forward_blocked = forward_blocked
                if forward_blocked:
                    self.get_logger().warning(
                        '[FOLLOW] Translation held while yaw turns into the '
                        'commanded course; reverse flight is blocked.'
                    )
                else:
                    self.get_logger().info(
                        '[FOLLOW] Course is inside the forward flight sector; '
                        'translation resumed.'
                    )
            vel.linear.x = float(guarded_follow_xy[0])
            vel.linear.y = float(guarded_follow_xy[1])
            # OffboardCommander is the sole altitude controller in FOLLOW.
            # APF's separate z loop crossed its 0.05 m/s handoff threshold
            # against Offboard's altitude hold and caused vertical bobbing.
            vel.linear.z = 0.0
            yaw.data = float(
                self.follow_entry_hold_yaw
                if entry_hold
                else follow_yaw
            )

        elif self.phase == MissionPhase.APPROACH:
            now = time.monotonic()
            tracking_valid = self._landing_tracking_is_valid(now)
            h_err = float(np.linalg.norm(
                self.drone_pos[:2] - self.target_pos[:2]
            )) if self.has_target and self.has_odom else float('inf')
            rel_alt = float(abs(self.drone_pos[2] - self.target_pos[2])) if self.has_target and self.has_odom else (self.drone_pos[2] if self.has_odom else float('inf'))
            is_terminal = bool(self.has_odom and (self.drone_pos[2] <= self.final_descent_alt or rel_alt <= self.final_descent_alt) and h_err <= self.final_descent_rxy)

            if is_terminal and not tracking_valid:
                # Terminal touchdown safeguard in APPROACH: maintain gentle vertical descent to ground
                vel.linear.x = 0.0
                vel.linear.y = 0.0
                vel.linear.z = -0.15
                yaw.data = self._landing_yaw_setpoint(False)
            elif not tracking_valid:
                # If target was confirmed moving, maintain pursuit and climb to reopen camera FOV
                # rather than freezing in midair at low altitude where camera FOV is constricted.
                has_moving_target = (
                    self.has_odom and (
                        self.land_coast_active
                        or (self.last_confirmed_target_vel is not None and np.linalg.norm(self.last_confirmed_target_vel[:2]) >= 0.20)
                    )
                )
                if has_moving_target:
                    coast_dt = max(
                        0.0,
                        self._simulation_time_now() - self.last_confirmed_target_time,
                    ) if self.last_confirmed_target_time > 0.0 else 0.0
                    eff_target_pos = (
                        self.last_confirmed_target_pos
                        + self.last_confirmed_target_vel * coast_dt
                    )
                    dx = eff_target_pos[0] - self.drone_pos[0]
                    dy = eff_target_pos[1] - self.drone_pos[1]
                    corr = np.clip(0.85 * np.array([dx, dy]), -0.60, 0.60)
                    fwd_vel = target_velocity_feedforward(
                        self.last_confirmed_target_vel[:2],
                        min_speed=self.land_fallback_min_target_speed,
                    )
                    approach_xy = compose_follow_velocity(
                        corr, fwd_vel, 1.0, self.follow_speed_limit,
                    )
                    vel.linear.x = float(approach_xy[0])
                    vel.linear.y = float(approach_xy[1])
                    if self.drone_pos[2] < 2.2 and math.hypot(dx, dy) > 1.0:
                        vel.linear.z = 0.35  # Climb to reopen camera FOV
                    else:
                        vel.linear.z = 0.0
                else:
                    # Hover in place for static targets waiting for reacquire
                    vel.linear.x = 0.0
                    vel.linear.y = 0.0
                    vel.linear.z = 0.0
                yaw.data = self._landing_yaw_setpoint(False)
            elif self.landing_glide_active:
                # Once SMC owns the descent, only accept its live glide command.
                # A stale command or a closed covariance gate means hover.
                if (
                    self.safe_to_land
                    and self._covariance_status_is_fresh(now)
                    and self._smc_command_is_fresh(now)
                    and self.smc_sub_phase in ('GLIDE_SLOPE', 'FINAL_DESCENT', 'FINAL_GATE_WAIT')
                ):
                    vel.linear.x = float(self.smc_velocity[0])
                    vel.linear.y = float(self.smc_velocity[1])
                    vel.linear.z = float(self.smc_velocity[2])
                    vel.angular.z = float(self.smc_yaw_rate)
                yaw.data = self._landing_yaw_setpoint(True)
            else:
                # APF brings the drone over the pad at the current altitude.
                # Vertical motion starts only after the covariance-gated SMC
                # glide handover has been accepted.
                apf_fresh = (
                    self.last_apf_time > 0.0
                    and now - self.last_apf_time <= self.apf_command_timeout
                )
                if apf_fresh:
                    vel.linear.x = float(self.apf_velocity[0])
                    vel.linear.y = float(self.apf_velocity[1])
                # If horizontally aligned over pad, descend responsively to allow covariance to shrink and SMC glide to engage
                if h_err <= 0.45 and self.has_odom and self.drone_pos[2] > 0.60:
                    vel.linear.z = -0.50 if self.drone_pos[2] > 1.20 else -0.35
                elif h_err > 1.2 and self.has_odom and self.drone_pos[2] < 2.2:
                    # Target moved away; climb to reopen camera FOV and facilitate reacquisition
                    vel.linear.z = 0.35
                else:
                    vel.linear.z = 0.0
                yaw.data = self._landing_yaw_setpoint(True)

        elif self.phase == MissionPhase.LAND:
            now = time.monotonic()
            tracking_valid_land = self._landing_tracking_is_valid(now)

            # Resolve effective target position: live tracking OR dead-reckoned coast.
            # Coast propagates the last confirmed position forward with target velocity
            # to compensate for target motion during the brief disarm confirmation window.
            if tracking_valid_land and self.has_target and self.has_odom:
                eff_target_pos = self.target_pos
            elif self.land_coast_active and self.last_confirmed_target_time > 0.0 and self.has_odom:
                coast_dt = max(
                    0.0,
                    self._simulation_time_now() - self.last_confirmed_target_time,
                )
                eff_target_pos = (
                    self.last_confirmed_target_pos
                    + self.last_confirmed_target_vel * coast_dt
                )
            else:
                eff_target_pos = self.target_pos if (self.has_target and self.has_odom) else self.drone_pos

            h_err = float(np.linalg.norm(
                self.drone_pos[:2] - eff_target_pos[:2]
            )) if self.has_odom else 0.0
            rel_alt = float(abs(self.drone_pos[2] - eff_target_pos[2])) if self.has_odom else (self.drone_pos[2] if self.has_odom else 0.0)
            is_terminal = bool(self.has_odom and (self.drone_pos[2] <= self.final_descent_alt or rel_alt <= self.final_descent_alt) and h_err <= self.final_descent_rxy)

            # Use coasted tracking validity for SMC command gating when live
            # tracking is briefly lost but coast is still active.
            tracking_for_smc = tracking_valid_land or self.land_coast_active
            if tracking_valid_land and self.target_vel is not None:
                fallback_target_velocity = target_velocity_feedforward(
                    self.target_vel[:2],
                    min_speed=self.land_fallback_min_target_speed,
                )
            elif self.land_coast_active:
                fallback_target_velocity = target_velocity_feedforward(
                    self.last_confirmed_target_vel[:2],
                    min_speed=self.land_fallback_min_target_speed,
                )
            else:
                fallback_target_velocity = np.zeros(2, dtype=float)

            final_command_valid = (
                tracking_for_smc
                and self.safe_to_land
                and self._covariance_status_is_fresh(now)
                and self._smc_command_is_fresh(now)
                and self.smc_sub_phase in ('FINAL_DESCENT', 'FINAL_GATE_WAIT', 'GLIDE_SLOPE')
                and self.uncertainty_radius <= self.continue_uncertainty_max
            )
            if final_command_valid:
                vel.linear.x = float(self.smc_velocity[0])
                vel.linear.y = float(self.smc_velocity[1])
                vel.linear.z = float(self.smc_velocity[2])
                vel.angular.z = float(self.smc_yaw_rate)
            elif is_terminal:
                # Keep matching a moving pad even when covariance temporarily
                # closes the SMC command gate. Position correction alone lets
                # a 0.5 m/s platform move away while the vehicle descends.
                kp = 0.85
                dx = eff_target_pos[0] - self.drone_pos[0]
                dy = eff_target_pos[1] - self.drone_pos[1]
                # Taper position correction near ground to level attitude and prevent nose-dive tipping
                corr_fade = float(np.clip((rel_alt - 0.10) / 0.10, 0.0, 1.0))
                correction = np.clip(kp * corr_fade * np.array([dx, dy]), -0.35, 0.35)
                landing_xy = compose_follow_velocity(
                    correction, fallback_target_velocity, 1.0,
                    self.land_fallback_xy_speed_limit,
                )
                vel.linear.x = float(landing_xy[0])
                vel.linear.y = float(landing_xy[1])
                vel.linear.z = -0.40 if rel_alt > 0.40 else -0.20
                vel.angular.z = 0.0
            elif self.land_coast_active and self.has_odom:
                # Coast fallback: tracking briefly lost but dead-reckoning is valid.
                # Continue matching the last accepted target course and centering.
                kp = 0.75
                dx = eff_target_pos[0] - self.drone_pos[0]
                dy = eff_target_pos[1] - self.drone_pos[1]
                corr_fade = float(np.clip((rel_alt - 0.10) / 0.10, 0.0, 1.0))
                correction = np.clip(kp * corr_fade * np.array([dx, dy]), -0.30, 0.30)
                landing_xy = compose_follow_velocity(
                    correction, fallback_target_velocity, 1.0,
                    self.land_fallback_xy_speed_limit,
                )
                vel.linear.x = float(landing_xy[0])
                vel.linear.y = float(landing_xy[1])
                vel.linear.z = -0.35 if rel_alt > 0.40 else -0.20
                vel.angular.z = 0.0
            else:
                # Active fallback in LAND: commit to descent and center over pad
                kp = 0.75
                dx = (eff_target_pos[0] - self.drone_pos[0]) if self.has_odom else 0.0
                dy = (eff_target_pos[1] - self.drone_pos[1]) if self.has_odom else 0.0
                corr_fade = float(np.clip((rel_alt - 0.10) / 0.10, 0.0, 1.0))
                correction = np.clip(kp * corr_fade * np.array([dx, dy]), -0.35, 0.35)
                landing_xy = compose_follow_velocity(
                    correction, fallback_target_velocity, 1.0,
                    self.land_fallback_xy_speed_limit,
                )
                vel.linear.x = float(landing_xy[0])
                vel.linear.y = float(landing_xy[1])
                vel.linear.z = -0.40 if rel_alt > 0.40 else -0.20
                vel.angular.z = 0.0
            yaw.data = self._landing_yaw_setpoint(tracking_valid_land)

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
