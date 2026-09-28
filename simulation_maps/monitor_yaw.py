#!/usr/bin/env python3
"""Monitor real-time Yaw response: CMD Yaw vs Drone Actual Yaw vs Error.

Tự động chẩn đoán và hiển thị rõ ràng nguyên nhân nếu các topic bị thiếu.
"""

import math
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class YawMonitor(Node):
    def __init__(self):
        super().__init__('yaw_monitor')
        self.cmd = 0.0
        self.drone = 0.0
        self.has_cmd = False
        self.has_odom = False
        self.tick_count = 0

        # Subscriptions
        # 1. Yaw command: lắng nghe cả /ibvs/yaw_cmd (Test A) và /mission/yaw_setpoint (Test B)
        self.create_subscription(Float64, '/ibvs/yaw_cmd', self.c_cb, 10)
        self.create_subscription(Float64, '/mission/yaw_setpoint', self.c_cb, 10)

        # 2. Odometry: lắng nghe /odom (chuẩn) và topic thô từ Gazebo
        self.create_subscription(Odometry, '/odom', self.o_cb, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/model/x500_depth_0/odometry_with_covariance',
            self.o_cb, qos_profile_sensor_data
        )

        self.create_timer(0.2, self.log)
        self.get_logger().info("YawMonitor started. Kiểm tra các topic...")

    def c_cb(self, m: Float64):
        self.cmd = m.data
        self.has_cmd = True

    def o_cb(self, m: Odometry):
        q = m.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.drone = math.atan2(siny, cosy)
        self.has_odom = True

    def log(self):
        self.tick_count += 1
        err = wrap_pi(self.cmd - self.drone)
        cmd_str = f"{math.degrees(self.cmd):6.1f}°" if self.has_cmd else "   N/A "
        drone_str = f"{math.degrees(self.drone):6.1f}°" if self.has_odom else "   N/A "
        err_str = f"{math.degrees(err):6.1f}°" if (self.has_cmd and self.has_odom) else "   N/A "

        print(f"[YAW] CMD: {cmd_str} | DRONE: {drone_str} | ERR: {err_str}", flush=True)

        # Định kỳ mỗi 2 giây (10 ticks), nếu vẫn N/A thì in hướng dẫn chẩn đoán cụ thể:
        if (not self.has_cmd or not self.has_odom) and (self.tick_count % 10 == 0):
            n_odom = self.count_publishers('/odom') + self.count_publishers('/model/x500_depth_0/odometry_with_covariance')
            n_cmd = self.count_publishers('/ibvs/yaw_cmd') + self.count_publishers('/mission/yaw_setpoint')

            diag = []
            if not self.has_odom:
                if n_odom == 0:
                    diag.append("[!] Chưa có publisher /odom (0 pub) -> Kiểm tra Terminal 2 (launch_simulation.py)")
                else:
                    diag.append(f"[!] /odom có {n_odom} pub nhưng chưa có tin nhắn -> Kiểm tra Gazebo đã RUN (Play) chưa")
            if not self.has_cmd:
                if n_cmd == 0:
                    diag.append("[!] Chưa có publisher /ibvs/yaw_cmd (0 pub) -> Kiểm tra Terminal 4 (ibvs_controller.py)")
                else:
                    diag.append(f"[!] /ibvs/yaw_cmd có {n_cmd} pub nhưng chưa phát tin")

            if diag:
                print("     " + "\n     ".join(diag), flush=True)


def main():
    rclpy.init()
    node = YawMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
