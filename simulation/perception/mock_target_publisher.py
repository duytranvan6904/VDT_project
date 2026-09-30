#!/usr/bin/env python3
"""Mock Target Publisher for Pure APF Flight Testing.

Publishes a fixed 3D target state, phase=IDLE (while taking off) -> FOLLOW (when airborne),
and tracking_mode=TRACKING so that APF and Offboard Commander can be tested in flight completely
independent of vision (ArUco detector) and gimbal servos (IBVS).

Safety Interlock:
  Monitors drone altitude via /odom. While altitude < 2.0m, it publishes phase=IDLE
  to allow PX4 to complete vertical takeoff cleanly. Once altitude >= 2.0m, it
  automatically engages phase=FOLLOW to let APF fly the drone.

Usage:
    python3 simulation_maps/mock_target_publisher.py
    python3 simulation_maps/mock_target_publisher.py --x 5.0 --y 2.0 --z 0.02
"""

import argparse
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class MockTargetPublisher(Node):
    def __init__(self, tx: float, ty: float, tz: float, min_alt: float = 2.0):
        super().__init__('mock_target_publisher')
        self.tx = float(tx)
        self.ty = float(ty)
        self.tz = float(tz)
        self.min_alt = float(min_alt)

        self.current_alt = 0.0
        self.has_odom = False
        self.follow_engaged = False

        self.target_pub = self.create_publisher(Odometry, '/ekf/target_state', 10)
        self.phase_pub = self.create_publisher(String, '/mission/phase', 10)
        self.mode_pub = self.create_publisher(String, '/ekf/tracking_mode', 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Odometry, '/odom', self.odom_cb, sensor_qos)

        # 10 Hz publish loop
        self.timer = self.create_timer(0.1, self.publish_loop)
        self.get_logger().info(
            f'Mock Target Publisher active!\n'
            f'  Target Position      : ({self.tx:.2f}, {self.ty:.2f}, {self.tz:.2f})\n'
            f'  Min Takeoff Altitude : {self.min_alt:.1f} m\n'
            f'  Status               : Chờ drone cất cánh đạt độ cao an toàn (z >= {self.min_alt:.1f}m)...'
        )

    def odom_cb(self, msg: Odometry):
        self.current_alt = float(msg.pose.pose.position.z)
        self.has_odom = True

    def publish_loop(self):
        now = self.get_clock().now().to_msg()

        # 1. Target Odometry
        target_msg = Odometry()
        target_msg.header.stamp = now
        target_msg.header.frame_id = 'world'
        target_msg.pose.pose.position.x = self.tx
        target_msg.pose.pose.position.y = self.ty
        target_msg.pose.pose.position.z = self.tz
        target_msg.pose.pose.orientation.w = 1.0
        self.target_pub.publish(target_msg)

        # 2. Safety Interlock with latching:
        # If drone is still on the ground (alt < min_alt), keep phase=IDLE
        # Once drone climbs >= min_alt, latch phase=FOLLOW permanently for the flight!
        if not self.follow_engaged:
            if self.current_alt >= self.min_alt:
                self.follow_engaged = True
                self.get_logger().info(
                    f'✅ Drone đã đạt độ cao an toàn ({self.current_alt:.2f}m >= {self.min_alt:.1f}m)!\n'
                    f'   Kích hoạt phase FOLLOW -> APF chính thức điều khiển bay ngang!'
                )
                phase = 'FOLLOW'
            else:
                phase = 'IDLE'
        else:
            phase = 'FOLLOW'

        self.phase_pub.publish(String(data=phase))
        self.mode_pub.publish(String(data='TRACKING'))


def main():
    parser = argparse.ArgumentParser(description='Publish mock target for independent APF test')
    parser.add_argument('--x', type=float, default=5.0, help='Target X in meters (default: 5.0)')
    parser.add_argument('--y', type=float, default=2.0, help='Target Y in meters (default: 2.0)')
    parser.add_argument('--z', type=float, default=0.02, help='Target Z in meters (default: 0.02)')
    parser.add_argument('--min-alt', type=float, default=2.0, help='Minimum altitude before engaging FOLLOW (default: 2.0m)')
    args = parser.parse_args()

    rclpy.init()
    node = MockTargetPublisher(args.x, args.y, args.z, args.min_alt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[Mock Target] Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
