#!/usr/bin/env python3
"""
Camera & Marker Monitor Node
Giám sát trạng thái hoạt động của Camera và Aruco Marker Detector.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped


class CameraMarkerMonitorNode(Node):
    def __init__(self):
        super().__init__('camera_marker_monitor')

        self.last_img_time = 0.0
        self.img_count = 0
        self.last_marker_time = 0.0
        self.marker_detected = False
        self.last_marker_pos = None

        # Camera subscriptions
        self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, '/depth_camera', self.depth_cb, qos_profile_sensor_data
        )

        # Marker subscriptions
        self.create_subscription(
            Bool, '/hpad/detected', self.marker_detected_cb, 10
        )
        self.create_subscription(
            PointStamped, '/hpad/position_camera', self.marker_pos_cb, 10
        )

        self.timer = self.create_timer(1.0, self.status_report)
        self.get_logger().info('=== Camera & Marker Monitor Node Started ===')

    def image_cb(self, _):
        self.last_img_time = time.monotonic()
        self.img_count += 1

    def depth_cb(self, _):
        self.last_img_time = time.monotonic()
        self.img_count += 1

    def marker_detected_cb(self, msg: Bool):
        self.marker_detected = msg.data
        if msg.data:
            self.last_marker_time = time.monotonic()

    def marker_pos_cb(self, msg: PointStamped):
        self.last_marker_pos = (msg.point.x, msg.point.y, msg.point.z)
        self.last_marker_time = time.monotonic()

    def status_report(self):
        now = time.monotonic()
        cam_online = (now - self.last_img_time) < 1.5 if self.last_img_time > 0 else False
        marker_fresh = (now - self.last_marker_time) < 1.0 if self.last_marker_time > 0 else False

        cam_status = f"\033[92mONLINE ({self.img_count} frames)\033[0m" if cam_online else "\033[91mNO SIGNAL\033[0m"
        
        if marker_fresh:
            pos_str = f"x={self.last_marker_pos[0]:.2f}, y={self.last_marker_pos[1]:.2f}, z={self.last_marker_pos[2]:.2f}m" if self.last_marker_pos else "yes"
            marker_status = f"\033[92mDETECTED [{pos_str}]\033[0m"
        else:
            marker_status = "\033[93mSEARCHING (Not Found)\033[0m"

        self.get_logger().info(f"[CAMERA]: {cam_status} | [MARKER]: {marker_status}")


def main():
    rclpy.init()
    node = CameraMarkerMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
