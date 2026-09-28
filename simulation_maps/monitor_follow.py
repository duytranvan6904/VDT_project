#!/usr/bin/env python3
"""Monitor real-time Follow metrics: Phase, EKF Mode, Distance, Pitch CMD vs Req."""

import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry


class FollowMonitor(Node):
    def __init__(self):
        super().__init__('follow_monitor')
        self.phase = 'IDLE'
        self.mode = 'INIT'
        self.pitch = 0.0
        self.drone_pos = [0.0, 0.0, 0.0]
        self.tgt_pos = [5.0, 2.0, 0.0]
        self.has_odom = False
        self.has_tgt = False
        self.has_pitch = False

        self.create_subscription(String, '/mission/phase', lambda m: setattr(self, 'phase', m.data), 10)
        self.create_subscription(String, '/ekf/tracking_mode', lambda m: setattr(self, 'mode', m.data), 10)
        self.create_subscription(Float64, '/ibvs/gimbal_pitch', self.p_cb, 10)
        self.create_subscription(Odometry, '/odom', self.o_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ekf/target_state', self.t_cb, 10)
        self.create_timer(0.25, self.log)
        self.get_logger().info("FollowMonitor started. Waiting for topics...")

    def p_cb(self, m: Float64):
        self.pitch = m.data
        self.has_pitch = True

    def o_cb(self, m: Odometry):
        p = m.pose.pose.position
        self.drone_pos = [p.x, p.y, p.z]
        self.has_odom = True

    def t_cb(self, m: Odometry):
        p = m.pose.pose.position
        self.tgt_pos = [p.x, p.y, p.z]
        self.has_tgt = True

    def log(self):
        d_h = math.hypot(self.drone_pos[0] - self.tgt_pos[0], self.drone_pos[1] - self.tgt_pos[1])
        dz = max(0.1, self.drone_pos[2] - self.tgt_pos[2])
        req_pitch = -math.atan2(dz, max(0.1, d_h))
        pitch_str = f"{math.degrees(self.pitch):5.1f}°" if self.has_pitch else " N/A "
        print(
            f"[{self.phase:7}] EKF: {self.mode:12} | Dist: {d_h:4.2f}m | "
            f"Gimbal CMD: {pitch_str} | Geometric Req: {math.degrees(req_pitch):5.1f}°",
            flush=True
        )


def main():
    rclpy.init()
    node = FollowMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
