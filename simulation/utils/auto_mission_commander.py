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
import os
import sys
import time
import math

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
    def __init__(
        self,
        takeoff_alt: float = 3.0,
        follow_duration: float = 12.0,
        max_mission_time: float = 80.0,
        move_target: bool = False,
        target_speed: float = 0.5,
        keep_moving_through_landing: bool = False,
        target_turn_after_s: float = 0.0,
        follow_only: bool = False,
        trajectory: str = "default",
    ):
        super().__init__('auto_mission_commander')

        self.takeoff_alt = takeoff_alt
        self.follow_duration = follow_duration
        self.max_mission_time = max_mission_time
        self.move_target = move_target
        self.target_speed = target_speed
        self.keep_moving_through_landing = keep_moving_through_landing
        self.target_turn_after_s = max(0.0, float(target_turn_after_s))
        self.follow_only = follow_only
        self.trajectory = trajectory
        self.target_turn_reported = False

        self.phase = "IDLE"
        self.drone_alt = 0.0
        self.has_odom = False
        self.touchdown = False
        self.mission_complete = False
        self.takeoff_sent = False
        self.last_takeoff_attempt = 0.0
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
        self.hpad_cmd_pub = self.create_publisher(Twist, '/hpad/cmd_vel_cmd', 10)

        # Control loop @ 5 Hz
        self.timer = self.create_timer(0.2, self._control_loop)
        self.get_logger().info(f"Auto Mission Commander initialized (follow_only={self.follow_only}, trajectory={self.trajectory}).")

    def _odom_cb(self, msg: Odometry):
        self.drone_alt = float(msg.pose.pose.position.z)
        self.has_odom = True

    def _phase_cb(self, msg: String):
        prev = self.phase
        self.phase = msg.data
        if prev != self.phase:
            self.get_logger().info(f"FSM Phase change: {prev} -> {self.phase}")

    def _touchdown_cb(self, msg: Bool):
        if msg.data and self.phase in ('LAND', 'TOUCHDOWN', 'IDLE'):
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
            self.hpad_cmd_pub.publish(Twist())
            self.mission_complete = True
            return

        if not self.has_odom:
            return

        # Step 1: Send Takeoff after 4.5 seconds of sensor and EKF2 stabilization
        if (
            not self.takeoff_sent
            and elapsed > 4.5
            and time.time() - self.last_takeoff_attempt >= 5.0
        ):
            self.last_takeoff_attempt = time.time()
            self.get_logger().info("Initiating Auto-Takeoff...")
            self._send_takeoff_mavlink()
            return

        # Step 2: In FOLLOW phase, track duration and optionally move H-Pad
        if self.phase == "FOLLOW":
            if self.follow_start_time is None:
                self.follow_start_time = time.time()
                self.get_logger().info(f"Entered FOLLOW phase. Maintaining follow for {self.follow_duration}s...")

            follow_elapsed = time.time() - self.follow_start_time

            # Once follow duration expires, handle land command or follow-only completion
            if follow_elapsed >= self.follow_duration:
                if self.follow_only and not self.mission_complete:
                    self.hpad_cmd_pub.publish(Twist())
                    self.get_logger().info("🏆 Follow-only duration completed successfully! Mission complete.")
                    self.mission_complete = True
                    return
                elif not self.land_cmd_sent:
                    if not self.keep_moving_through_landing:
                        self.hpad_cmd_pub.publish(Twist())

                    self.get_logger().info("Follow duration completed. Sending /operator/land_command = True!")
                    msg = Bool()
                    msg.data = True
                    self.land_cmd_pub.publish(msg)
                    self.land_cmd_sent = True
                elif self.land_cmd_sent and not self.touchdown and not self.mission_complete:
                    if not hasattr(self, '_waveoff_start') or self._waveoff_start is None:
                        self._waveoff_start = time.time()
                        self.get_logger().warning("⚠️ Landing wave-off detected (reverted to FOLLOW). Stabilizing for 3.0s before re-commanding landing...")
                    elif time.time() - self._waveoff_start >= 3.0:
                        self.get_logger().info("📥 Re-issuing /operator/land_command = True after wave-off stabilization!")
                        msg = Bool(data=True)
                        self.land_cmd_pub.publish(msg)
                        self._waveoff_start = time.time()
        else:
            self._waveoff_start = None

        # A moving-pad landing test must keep the platform moving after the
        # land request, through APPROACH and LAND, then stop only on touchdown.
        if self.move_target and self.follow_start_time is not None and not self.touchdown and not self.mission_complete:
            follow_elapsed = time.time() - self.follow_start_time
            should_move = (
                self.phase == 'FOLLOW' and (
                    follow_elapsed < self.follow_duration
                    or (self.keep_moving_through_landing and self.land_cmd_sent)
                )
            ) or (
                self.keep_moving_through_landing
                and self.phase in ('APPROACH', 'LAND')
            )
            if should_move:
                twist = Twist()
                if self.trajectory == 'slalom':
                    if self.follow_duration > 40.0:
                        # 14-leg obstacle avoidance corridor course (~69m at 1.0 m/s)
                        # Weaves through the 30-pillar forest with minimum clearance >= 0.61m
                        wps = [
                            (3.0, 1.0), (9.0, 0.0), (11.0, 5.0), (8.0, 11.0),
                            (3.0, 10.0), (0.0, 5.0), (1.0, -2.0), (6.0, -3.0),
                            (10.0, -2.0), (11.0, 3.0), (8.0, 6.0), (5.0, 4.0),
                            (3.0, 1.0), (6.0, 0.0), (10.0, 0.0),
                        ]
                        t_acc = 0.0
                        vx_cmd, vy_cmd = self.target_speed, 0.0
                        for i in range(len(wps) - 1):
                            dx = wps[i + 1][0] - wps[i][0]
                            dy = wps[i + 1][1] - wps[i][1]
                            dist = math.hypot(dx, dy)
                            dur = dist / max(0.1, self.target_speed)
                            if t_acc <= follow_elapsed < t_acc + dur:
                                vx_cmd = (dx / dist) * self.target_speed
                                vy_cmd = (dy / dist) * self.target_speed
                                break
                            t_acc += dur
                        else:
                            vx_cmd = self.target_speed
                            vy_cmd = 0.0
                        twist.linear.x = vx_cmd
                        twist.linear.y = vy_cmd
                    else:
                        progress = follow_elapsed / max(1.0, self.follow_duration)
                        if progress < 0.25:
                            twist.linear.x = self.target_speed * 0.707
                            twist.linear.y = self.target_speed * 0.707
                        elif progress < 0.50:
                            twist.linear.x = -self.target_speed * 0.35
                            twist.linear.y = self.target_speed * 0.85
                        elif progress < 0.75:
                            twist.linear.x = self.target_speed * 0.40
                            twist.linear.y = -self.target_speed * 0.85
                        else:
                            twist.linear.x = -self.target_speed * 0.50
                            twist.linear.y = -self.target_speed * 0.50
                elif (
                    self.target_turn_after_s > 0.0
                    and follow_elapsed >= self.target_turn_after_s
                ):
                    twist.linear.y = self.target_speed
                    if not self.target_turn_reported:
                        self.get_logger().info(
                            f"H-Pad turning from +X to +Y after "
                            f"{follow_elapsed:.1f}s at {self.target_speed:.2f}m/s."
                        )
                        self.target_turn_reported = True
                else:
                    twist.linear.x = self.target_speed
                self.hpad_cmd_pub.publish(twist)
            elif (
                (self.phase == 'FOLLOW' and follow_elapsed >= self.follow_duration)
                or (
                    self.keep_moving_through_landing
                    and self.land_cmd_sent
                    and self.phase not in ('FOLLOW', 'APPROACH', 'LAND')
                )
            ):
                self.hpad_cmd_pub.publish(Twist())

        # Step 3: Check completion only when drone is in LAND or has completed descent
        if self.land_cmd_sent:
            # Re-publish land command periodically until phase leaves FOLLOW
            if self.phase == "FOLLOW" and int(elapsed * 5) % 5 == 0:
                msg = Bool()
                msg.data = True
                self.land_cmd_pub.publish(msg)

            if (self.phase in ('LAND', 'TOUCHDOWN', 'IDLE') and self.touchdown) or (self.phase == 'IDLE' and self.land_cmd_sent and elapsed > 20.0):
                self.hpad_cmd_pub.publish(Twist())
                self.get_logger().info("🏆 Mission Successful: Touchdown detected and drone safe on pad!")
                self.mission_complete = True


def main():
    parser = argparse.ArgumentParser(description="Auto Mission Commander")
    parser.add_argument('--takeoff-alt', type=float, default=3.0)
    parser.add_argument('--follow-duration', type=float, default=12.0)
    parser.add_argument('--max-time', type=float, default=160.0)
    parser.add_argument('--move-target', action='store_true')
    parser.add_argument('--target-speed', type=float, default=0.5)
    parser.add_argument(
        '--target-turn-after-s', type=float, default=0.0,
        help='Turn the moving H-Pad from +X to +Y this many seconds after FOLLOW starts.',
    )
    parser.add_argument(
        '--keep-moving-through-landing', action='store_true',
        help='Continue H-Pad motion through APPROACH/LAND; stop at touchdown.',
    )
    parser.add_argument(
        '--follow-only', action='store_true',
        help='Maintain FOLLOW phase without landing for the duration, then complete mission.',
    )
    parser.add_argument(
        '--trajectory', default='default', choices=['default', 'turning', 'slalom'],
        help='Target motion trajectory pattern.',
    )
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = AutoMissionCommander(
        takeoff_alt=args.takeoff_alt,
        follow_duration=args.follow_duration,
        max_mission_time=args.max_time,
        move_target=args.move_target,
        target_speed=args.target_speed,
        keep_moving_through_landing=args.keep_moving_through_landing,
        target_turn_after_s=args.target_turn_after_s,
        follow_only=args.follow_only,
        trajectory=args.trajectory,
    )

    try:
        while rclpy.ok() and not node.mission_complete:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
