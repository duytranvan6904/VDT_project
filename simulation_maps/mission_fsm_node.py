#!/usr/bin/env python3
"""Mission FSM Node — State machine IDLE → SEARCH → FOLLOW → APPROACH → LAND.

Trách nhiệm:
  1. Quản lý chuyển pha dựa trên detection state, operator command, alignment
  2. Publish /mission/phase (String) để các node khác biết pha hiện tại
  3. Tổng hợp velocity + yaw thành setpoint cuối cùng gửi Offboard Commander

Transition rules:
  IDLE     → SEARCH  : sau khi takeoff (drone altitude > 80% takeoff_alt)
  SEARCH   → FOLLOW  : ArUco detected ≥ N frames liên tiếp
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


class MissionPhase(str, Enum):
    IDLE = 'IDLE'
    SEARCH = 'SEARCH'
    FOLLOW = 'FOLLOW'
    APPROACH = 'APPROACH'
    LAND = 'LAND'


class MissionFSMNode(Node):
    """Mission state machine that orchestrates all autonomous components."""

    def __init__(self):
        super().__init__('mission_fsm')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('detection_confirm_frames', 10)
        self.declare_parameter('follow_distance', 3.5)
        self.declare_parameter('alignment_threshold', 0.3)
        self.declare_parameter('land_altitude', 0.5)
        self.declare_parameter('search_timeout', 3.0)
        self.declare_parameter('approach_timeout', 3.0)
        self.declare_parameter('takeoff_altitude', 3.0)
        self.declare_parameter('land_descent_speed', 0.3)

        self.detection_confirm = self.get_parameter('detection_confirm_frames').value
        self.follow_dist = self.get_parameter('follow_distance').value
        self.align_thresh = self.get_parameter('alignment_threshold').value
        self.land_alt = self.get_parameter('land_altitude').value
        self.search_timeout = self.get_parameter('search_timeout').value
        self.approach_timeout = self.get_parameter('approach_timeout').value
        self.takeoff_alt = self.get_parameter('takeoff_altitude').value
        self.descent_speed = self.get_parameter('land_descent_speed').value

        # ── State ────────────────────────────────────────────────────────
        self.phase = MissionPhase.IDLE
        self.detected = False
        self.consecutive_detections = 0
        self.tracking_mode = 'EXPIRED'
        self.drone_pos = np.zeros(3)
        self.drone_yaw = 0.0
        self.target_pos = np.zeros(3)
        self.has_odom = False
        self.has_target = False
        self.land_requested = False
        self.last_detection_time = 0.0

        # APF + IBVS outputs
        self.apf_velocity = np.zeros(3)
        self.ibvs_yaw = 0.0

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
        self.get_logger().info(
            f"[STATUS] Phase: {self.phase.value:<7} | "
            f"Drone: ({self.drone_pos[0]:.1f}, {self.drone_pos[1]:.1f}, {self.drone_pos[2]:.1f})m | "
            f"Target: ({self.target_pos[0]:.1f}, {self.target_pos[1]:.1f}) | "
            f"Dist: {dist_str} | "
            f"EKF: {self.tracking_mode} | Det: {self.detected}"
        )

    # ── Input callbacks ──────────────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.drone_pos = np.array([p.x, p.y, p.z])
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.drone_yaw = math.atan2(siny, cosy)
        self.has_odom = True

    def target_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.target_pos = np.array([p.x, p.y, p.z])
        self.has_target = True

    def detected_cb(self, msg: Bool):
        self.detected = msg.data
        if msg.data:
            self.consecutive_detections += 1
            self.last_detection_time = time.monotonic()
        else:
            self.consecutive_detections = 0

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
            if self.consecutive_detections >= 5:
                self.phase = MissionPhase.FOLLOW
                self.get_logger().info(
                    f'Drone cất cánh đạt độ cao {self.drone_pos[2]:.2f}m và thấy mục tiêu! '
                    f'Chuyển IDLE → FOLLOW.'
                )
            else:
                self.phase = MissionPhase.SEARCH
                self.get_logger().info(
                    f'Drone cất cánh đạt độ cao {self.drone_pos[2]:.2f}m. '
                    f'Bắt đầu quét tìm kiếm SEARCH.'
                )

    def _handle_search(self):
        """Rotate yaw, wait for stable ArUco detection."""
        if self.consecutive_detections >= 5:
            self.phase = MissionPhase.FOLLOW
            self.get_logger().info(
                f'Target confirmed after {self.consecutive_detections} '
                f'consecutive detections. Bắt đầu FOLLOW.'
            )

    def _handle_follow(self):
        """Follow target with APF obstacle avoidance."""
        # Check for target loss
        if self.tracking_mode == 'EXPIRED':
            age = time.monotonic() - self.last_detection_time
            if age > self.search_timeout:
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

            if horizontal_error < self.align_thresh and altitude < self.land_alt:
                self.phase = MissionPhase.LAND
                self.get_logger().info(
                    f'Alignment OK (error={horizontal_error:.2f}m, '
                    f'alt={altitude:.2f}m), starting LAND.'
                )

    def _handle_land(self):
        """Vertical descent until touchdown."""
        if self.has_odom and self.drone_pos[2] < 0.1:
            self.phase = MissionPhase.IDLE
            self.land_requested = False
            self.get_logger().info('Touchdown detected! Returning to IDLE.')

    # ── Setpoint composition ─────────────────────────────────────────────

    def _publish_setpoints(self):
        """Compose final velocity and yaw setpoints based on current phase."""
        vel = Twist()
        yaw = Float64()

        if self.phase == MissionPhase.IDLE:
            # Hover at current position (zero velocity)
            yaw.data = float(self.drone_yaw)

        elif self.phase == MissionPhase.SEARCH:
            # Hover + IBVS provides yaw rotation
            yaw.data = float(self.ibvs_yaw)

        elif self.phase == MissionPhase.FOLLOW:
            # APF velocity + IBVS yaw
            vel.linear.x = float(self.apf_velocity[0])
            vel.linear.y = float(self.apf_velocity[1])
            vel.linear.z = float(self.apf_velocity[2])
            yaw.data = float(self.ibvs_yaw)

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
            # Straight down, lock yaw
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
