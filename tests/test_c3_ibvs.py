#!/usr/bin/env python3
"""Automated Test Runner for C3: IBVS Open-Loop with Mock Target.

Kiểm thử độc lập node `ibvs_controller.py` theo đúng kịch bản mục C3 trong COMPONENT_TEST_PLAN.md.
Tự động giả lập:
  - /mission/phase = "FOLLOW"
  - /ekf/tracking_mode = "TRACKING"
  - /odom: Drone tại (0, 0, 3.0m), yaw = 0 rad
  - /ekf/target_state: Các tọa độ [5,0,0], [0,5,0], [-5,0,0], [0,-5,0] và kiểm tra wrap-around (+179° -> -179°).

Lắng nghe:
  - /ibvs/yaw_cmd
  - /ibvs/gimbal_pitch

Chấm điểm PASS / FAIL tự động.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String


class C3TestHarness(Node):
    def __init__(self):
        super().__init__('c3_ibvs_test_harness')

        # Publishers
        self.phase_pub = self.create_publisher(String, '/mission/phase', 10)
        self.mode_pub = self.create_publisher(String, '/ekf/tracking_mode', 10)
        self.target_pub = self.create_publisher(Odometry, '/ekf/target_state', 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', sensor_qos)

        # Subscribers
        self.latest_yaw_cmd: float | None = None
        self.latest_gimbal_pitch: float | None = None
        self.yaw_history: list[tuple[float, float]] = []  # (timestamp, yaw_cmd)

        self.create_subscription(Float64, '/ibvs/yaw_cmd', self.yaw_cb, 10)
        self.create_subscription(Float64, '/ibvs/gimbal_pitch', self.pitch_cb, 10)

        # State to publish continuously
        self.target_x = 5.0
        self.target_y = 0.0
        self.target_z = 0.0

        # 10 Hz continuous publisher timer
        self.timer = self.create_timer(0.1, self.publish_loop)

    def yaw_cb(self, msg: Float64):
        self.latest_yaw_cmd = float(msg.data)
        self.yaw_history.append((time.monotonic(), self.latest_yaw_cmd))
        if len(self.yaw_history) > 200:
            self.yaw_history.pop(0)

    def pitch_cb(self, msg: Float64):
        self.latest_gimbal_pitch = float(msg.data)

    def set_target(self, x: float, y: float, z: float = 0.0):
        self.target_x = float(x)
        self.target_y = float(y)
        self.target_z = float(z)

    def publish_loop(self):
        now = self.get_clock().now().to_msg()

        # 1. Mission Phase = FOLLOW
        self.phase_pub.publish(String(data='FOLLOW'))

        # 2. Tracking Mode = TRACKING
        self.mode_pub.publish(String(data='TRACKING'))

        # 3. Drone Odometry at (0, 0, 3.0), yaw = 0.0
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'world'
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 3.0
        odom_msg.pose.pose.orientation.w = 1.0  # yaw = 0 rad
        self.odom_pub.publish(odom_msg)

        # 4. Target Odometry
        target_msg = Odometry()
        target_msg.header.stamp = now
        target_msg.header.frame_id = 'world'
        target_msg.pose.pose.position.x = self.target_x
        target_msg.pose.pose.position.y = self.target_y
        target_msg.pose.pose.position.z = self.target_z
        target_msg.pose.pose.orientation.w = 1.0
        self.target_pub.publish(target_msg)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def run_tests():
    parser = argparse.ArgumentParser(description='C3 IBVS Automated Test Runner')
    parser.add_argument('--settle-time', type=float, default=3.5,
                        help='Time to wait for slew rate & EMA filter to settle (seconds)')
    args = parser.parse_args()

    rclpy.init()
    node = C3TestHarness()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('=' * 75)
    print('   KIỂM THỬ TỰ ĐỘNG C3: IBVS CONTROLLER OPEN-LOOP (TARGET GIẢ)')
    print('=' * 75)
    print('Đang kiểm tra kết nối tới node `ibvs_controller`...')

    # Wait for first response
    t0 = time.time()
    while node.latest_yaw_cmd is None or node.latest_gimbal_pitch is None:
        if time.time() - t0 > 5.0:
            print('\n[LỖI] Không nhận được phản hồi từ /ibvs/yaw_cmd hoặc /ibvs/gimbal_pitch!')
            print('Hãy đảm bảo bạn đã chạy node IBVS ở Terminal 1:')
            print('  python3 simulation_maps/ibvs_controller.py\n')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        time.sleep(0.2)

    print('Đã kết nối thành công với node IBVS Controller!\n')

    # Test cases: (name, x, y, z, expected_yaw_rad, expected_pitch_rad)
    # Expected pitch calculation:
    #   dx, dy -> dist_h; dz = 3.0 - 0.0 = 3.0m
    #   dist_h = 5.0m -> pitch = -atan2(3.0, 5.0) = -0.5404 rad (-30.96°)
    expected_pitch = -math.atan2(3.0, 5.0)  # ~ -30.96°

    test_cases = [
        ('1. Hướng Đông  (+X): [ 5.0,  0.0, 0.0]',  5.0,  0.0, 0.0, 0.0,             expected_pitch),
        ('2. Hướng Bắc   (+Y): [ 0.0,  5.0, 0.0]',  0.0,  5.0, 0.0, math.pi / 2,     expected_pitch),
        ('3. Hướng Tây   (-X): [-5.0,  0.0, 0.0]', -5.0,  0.0, 0.0, math.pi,         expected_pitch),
        ('4. Hướng Nam   (-Y): [ 0.0, -5.0, 0.0]',  0.0, -5.0, 0.0, -math.pi / 2,    expected_pitch),
    ]

    all_passed = True
    results = []

    for name, tx, ty, tz, exp_yaw, exp_pitch in test_cases:
        print(f'--> Đang chạy test: {name}')
        node.set_target(tx, ty, tz)

        # Wait for slew rate and EMA filter to settle
        settle_time = args.settle_time
        for _ in range(int(settle_time * 10)):
            time.sleep(0.1)

        actual_yaw = node.latest_yaw_cmd
        actual_pitch = node.latest_gimbal_pitch

        # Yaw error check (modulo 2pi)
        yaw_err = abs(wrap_pi(actual_yaw - exp_yaw))
        pitch_err = abs(actual_pitch - exp_pitch)

        # Tolerance: yaw <= 0.15 rad (~8.5° due to settle slew time), pitch <= 0.08 rad (~4.5°)
        passed = (yaw_err < 0.15) and (pitch_err < 0.08)
        if not passed:
            all_passed = False

        status_str = 'PASS' if passed else 'FAIL'
        results.append((name, exp_yaw, actual_yaw, yaw_err, exp_pitch, actual_pitch, pitch_err, status_str))
        print(f'    Yaw: đo được {math.degrees(actual_yaw):+6.1f}° (Kỳ vọng: {math.degrees(exp_yaw):+6.1f}°, Lệch: {math.degrees(yaw_err):.1f}°)')
        print(f'    Pitch: đo được {math.degrees(actual_pitch):+6.1f}° (Kỳ vọng: {math.degrees(exp_pitch):+6.1f}°, Lệch: {math.degrees(pitch_err):.1f}°)')
        print(f'    Kết quả: [{status_str}]\n')

    # Test case 5: Wrap-around test (+179° -> -179°)
    print('--> Đang chạy test 5: Wrap-around test biên (+179° -> -179°)')
    # Step 5a: Target at +179.4°: x = -5.0, y = +0.05
    node.set_target(-5.0, 0.05, 0.0)
    time.sleep(3.0)
    yaw_before = node.latest_yaw_cmd

    # Step 5b: Instant flip target to -179.4°: x = -5.0, y = -0.05
    node.yaw_history.clear()
    node.set_target(-5.0, -0.05, 0.0)

    # Monitor for 2.0s
    time.sleep(2.0)
    yaw_after = node.latest_yaw_cmd

    # Check yaw delta: error should be small (~2° = ~0.035 rad), not ~358° (~6.2 rad)
    delta_direct = abs(yaw_after - yaw_before)
    shortest_delta = abs(wrap_pi(yaw_after - yaw_before))

    # PASS when shortest_delta is small (< 10°) and node does not jump a full 358°
    wrap_passed = (shortest_delta < math.radians(10.0)) and (delta_direct < math.radians(180.0) or abs(abs(yaw_before) - math.pi) < 0.2)
    if not wrap_passed:
        all_passed = False

    status_str = 'PASS' if wrap_passed else 'FAIL'
    print(f'    Yaw trước khi đảo: {math.degrees(yaw_before):+6.1f}°')
    print(f'    Yaw sau khi đảo  : {math.degrees(yaw_after):+6.1f}°')
    print(f'    Độ lệch ngắn nhất (Wrap error): {math.degrees(shortest_delta):.2f}° (Kỳ vọng < 5°)')
    print(f'    Kết quả: [{status_str}]\n')

    # Summary table
    print('=' * 75)
    print(f"{'BÀI TEST':<40} | {'YAW KỲ VỌNG':<12} | {'YAW THỰC TẾ':<12} | {'KẾT QUẢ'}")
    print('-' * 75)
    for name, ey, ay, _, ep, ap, _, st in results:
        print(f"{name:<40} | {math.degrees(ey):+6.1f}°      | {math.degrees(ay):+6.1f}°      | [{st}]")
    print(f"{'5. Wrap-around (+179° -> -179°)':<40} | {'~ -179.4°':<12} | {math.degrees(yaw_after):+6.1f}°      | [{status_str}]")
    print('=' * 75)

    if all_passed:
        print(' TẤT CẢ TEST CASES C3 ĐÃ PASS HOÀN TOÀN! Node IBVS tính toán chuẩn xác.')
    else:
        print('⚠️ MỘT SỐ TEST CASE CHƯA ĐẠT. Vui lòng kiểm tra lại cấu hình góc trong IBVS.')
    print('=' * 75)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    run_tests()
