#!/usr/bin/env python3
"""Auto Mission Commander Node.

Tự động hóa toàn bộ trình tự điều khiển nhiệm vụ bay mô phỏng:
1. Chờ kết nối ROS 2 & PX4 SITL MAVLink
2. Gửi lệnh Auto-Takeoff lên độ cao an toàn (3.0m)
3. Chờ FSM chuyển sang pha FOLLOW (bám marker thành công)
4. Tùy chọn: Điều khiển H-Pad di chuyển theo kịch bản (nếu bật target_motion)
5. Duy trì pha FOLLOW trong khoảng thời gian mong muốn (follow_duration_s)
6. Phát lệnh hạ cánh (/operator/land_command = True)
7. Giám sát quá trình hạ cánh cho tới khi TOUCHDOWN và FSM về IDLE
8. Báo tín hiệu kết thúc chuyến bay để orchestrator tiến hành thu dọn.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

os.environ['MAVLINK_DIALECT'] = 'common'
px4_mav_path = '/home/duy/VDT_project/PX4-Autopilot/src/modules/mavlink/mavlink'
if px4_mav_path not in sys.path:
    sys.path.insert(0, px4_mav_path)

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped


try:
    from simulation.core.takeoff import takeoff
except ImportError:
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)
    from simulation.core.takeoff import takeoff


class AutoMissionCommander(Node):
    def __init__(self, takeoff_alt: float = 3.0, follow_duration: float = 12.0, max_mission_time: float = 80.0, move_target: bool = False, target_speed: float = 0.5):
        super().__init__('auto_mission_commander')

        self.takeoff_alt = takeoff_alt
        self.follow_duration = follow_duration
        self.max_mission_time = max_mission_time
        self.move_target = move_target
        self.target_speed = target_speed

        self.phase = "IDLE"
        self.drone_alt = 0.0
        self.has_odom = False
        self.touchdown = False
        self.mission_complete = False
        self.takeoff_sent = False
        self.land_cmd_sent = False
        self.follow_start_time = None

        self.start_wall_time = time.time()

        # ROS 2 Subscriptions & Publishers
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.create_subscription(Odometry, '/odom', self._odom_cb, sensor_qos)
        self.create_subscription(String, '/mission/phase', self._phase_cb, 10)
        self.create_subscription(Bool, '/landing/touchdown', self._touchdown_cb, 10)

        self.land_cmd_pub = self.create_publisher(Bool, '/operator/land_command', 10)
        self.hpad_cmd_pub = self.create_publisher(Twist, '/hpad/cmd_vel', 10)

        # Control loop @ 5 Hz
        self.timer = self.create_timer(0.2, self._control_loop)
        self.get_logger().info("Auto Mission Commander initialized.")

    def _odom_cb(self, msg: Odometry):
        self.drone_alt = float(msg.pose.pose.position.z)
        self.has_odom = True

    def _phase_cb(self, msg: String):
        prev = self.phase
        self.phase = msg.data
        if prev != self.phase:
            self.get_logger().info(f"FSM Phase change: {prev} -> {self.phase}")

    def _touchdown_cb(self, msg: Bool):
        if msg.data and self.phase in ('LAND', 'TOUCHDOWN'):
            self.touchdown = True

    def _send_takeoff_mavlink(self):
        try:
            self.get_logger().info(f"Calling takeoff(target_alt={self.takeoff_alt:.1f}m)...")
            res = takeoff(target_alt=self.takeoff_alt)
            if res:
                self.takeoff_sent = True
                self.get_logger().info("✅ Takeoff command accepted by PX4!")
                return True
            else:
                self.get_logger().error("❌ Takeoff command returned False!")
                return False
        except Exception as e:
            self.get_logger().error(f"Error executing takeoff: {e}")
            return False

    def _control_loop(self):
        elapsed = time.time() - self.start_wall_time
        if elapsed > self.max_mission_time:
            self.get_logger().warn(f"Max mission timeout reached ({self.max_mission_time}s). Ending mission.")
            self.mission_complete = True
            return

        if not self.has_odom:
            return

        # Step 1: Send Takeoff after 4.5 seconds of sensor and EKF2 stabilization
        if not self.takeoff_sent and elapsed > 4.5:
            self.get_logger().info("Initiating Auto-Takeoff...")
            self._send_takeoff_mavlink()
            return

        # Step 2: In FOLLOW phase, track duration and optionally move H-Pad
        if self.phase == "FOLLOW":
            if self.follow_start_time is None:
                self.follow_start_time = time.time()
                self.get_logger().info(f"Entered FOLLOW phase. Maintaining follow for {self.follow_duration}s...")

            follow_elapsed = time.time() - self.follow_start_time

            # Move target if requested
            if self.move_target and follow_elapsed < self.follow_duration:
                twist = Twist()
                twist.linear.x = self.target_speed
                twist.linear.y = 0.2 * math.sin(follow_elapsed * 1.5)
                self.hpad_cmd_pub.publish(twist)

            # Once follow duration expires, trigger Land command!
            if follow_elapsed >= self.follow_duration and not self.land_cmd_sent:
                # Stop target motion before landing
                stop_twist = Twist()
                self.hpad_cmd_pub.publish(stop_twist)

                self.get_logger().info("Follow duration completed. Sending /operator/land_command = True!")
                msg = Bool()
                msg.data = True
                self.land_cmd_pub.publish(msg)
                self.land_cmd_sent = True

        # Step 3: Check completion only when drone is in LAND or has completed descent
        if self.land_cmd_sent:
            # Re-publish land command periodically until phase leaves FOLLOW
            if self.phase == "FOLLOW" and int(elapsed * 5) % 5 == 0:
                msg = Bool()
                msg.data = True
                self.land_cmd_pub.publish(msg)

            if (self.phase in ('LAND', 'TOUCHDOWN', 'IDLE') and self.touchdown) or \
               (self.drone_alt <= 0.15 and self.phase in ('LAND', 'TOUCHDOWN', 'IDLE')):
                self.get_logger().info("🏆 Mission Successful: Touchdown detected and drone safe on pad!")
                self.mission_complete = True


def main():
    parser = argparse.ArgumentParser(description="Auto Mission Commander")
    parser.add_argument('--takeoff-alt', type=float, default=3.0)
    parser.add_argument('--follow-duration', type=float, default=12.0)
    parser.add_argument('--max-time', type=float, default=160.0)
    parser.add_argument('--move-target', action='store_true')
    parser.add_argument('--target-speed', type=float, default=0.5)
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = AutoMissionCommander(
        takeoff_alt=args.takeoff_alt,
        follow_duration=args.follow_duration,
        max_mission_time=args.max_time,
        move_target=args.move_target,
        target_speed=args.target_speed
    )

    try:
        while rclpy.ok() and not node.mission_complete:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
