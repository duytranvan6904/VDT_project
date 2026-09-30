#!/usr/bin/env python3
"""
Converts 32FC1 Depth Images from Gazebo Sim to 8-bit Mono Images (mono8)
with BEST_EFFORT QoS Profile to guarantee compatibility with ros_gz_bridge and RViz2.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import numpy as np

class DepthToImageConverter(Node):
    def __init__(self):
        super().__init__('depth_to_image_converter')
        
        # Gazebo Sim ros_gz_bridge uses BEST_EFFORT QoS!
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(
            Image,
            '/depth_camera',
            self.depth_callback,
            qos_profile
        )
        self.pub_mono = self.create_publisher(Image, '/depth_camera/image_mono', 10)
        self.get_logger().info("Depth-to-RViz Converter active with BEST_EFFORT QoS! Publishing /depth_camera/image_mono")

    def depth_callback(self, msg: Image):
        try:
            # Parse 32FC1 float image
            if msg.encoding in ['32FC1', 'R_FLOAT32']:
                depth_data = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            elif msg.encoding in ['16UC1']:
                depth_data = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width)).astype(np.float32) / 1000.0
            else:
                return

            # Replace NaNs/Infs with 10.0m
            depth_clean = np.nan_to_num(depth_data, nan=10.0, posinf=10.0, neginf=0.0)
            depth_clipped = np.clip(depth_clean, 0.2, 8.0)

            # Normalize to 0-255 uint8 (grayscale)
            normalized = ((depth_clipped - 0.2) / (8.0 - 0.2) * 255.0).astype(np.uint8)

            # Publish Grayscale mono8 (Fully Compatible with RViz2 Image plugin)
            mono_msg = Image()
            mono_msg.header = msg.header
            mono_msg.height = msg.height
            mono_msg.width = msg.width
            mono_msg.encoding = 'mono8'
            mono_msg.is_bigendian = False
            mono_msg.step = msg.width
            mono_msg.data = normalized.tobytes()
            self.pub_mono.publish(mono_msg)

        except Exception as e:
            self.get_logger().error(f"Conversion error: {e}")

def main():
    rclpy.init()
    node = DepthToImageConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
