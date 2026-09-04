#!/usr/bin/env python3
"""
Standalone Obstacle PointCloud Generator for RViz2 and APF Testing (ROS 2 Version).
Creates 3D obstacle forest with pillars and boxes + publishes static TF for frame 'world'.
Derived from Fast-tracker (ZJU FAST-Lab) map generation specs.
"""

import math
import random
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class APFPointCloudGenerator(Node):
    def __init__(self):
        super().__init__('apf_pointcloud_generator')
        self.publisher_ = self.create_publisher(PointCloud2, '/map_generator/global_cloud', 10)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static TF for 'world' frame so RViz won't complain
        self.publish_static_tf()

        self.timer = self.create_timer(1.0, self.publish_map)
        self.points = self.generate_random_forest(num_obs=35, map_size=25.0, height=4.0)
        self.get_logger().info(f"Generated obstacle map with {len(self.points)} points for ROS 2.")

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'map'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(t)

    def generate_random_forest(self, num_obs=35, map_size=25.0, height=4.0, resolution=0.15):
        points = []
        half_size = map_size / 2.0

        for _ in range(num_obs):
            cx = random.uniform(-half_size, half_size)
            cy = random.uniform(-half_size, half_size)
            radius = random.uniform(0.3, 0.8)
            obs_h = random.uniform(2.0, height)

            # Clear space around takeoff point (0,0)
            if math.sqrt(cx**2 + cy**2) < 2.0:
                continue

            z = 0.0
            while z <= obs_h:
                num_steps = max(int(2 * math.pi * radius / resolution), 8)
                for i in range(num_steps):
                    theta = i * (2 * math.pi / num_steps)
                    px = cx + radius * math.cos(theta)
                    py = cy + radius * math.sin(theta)
                    points.append((px, py, z))
                z += resolution

        return points

    def publish_map(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'world'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = []
        for p in self.points:
            cloud_data.append(struct.pack('fff', p[0], p[1], p[2]))

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(self.points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(self.points)
        msg.is_dense = True
        msg.data = b"".join(cloud_data)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = APFPointCloudGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
