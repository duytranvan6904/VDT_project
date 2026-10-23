from __future__ import annotations

import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster


def generate_forest(rng, num_obs, map_size, height, resolution, clear_radius):
    half = map_size / 2.0
    chunks = []
    for _ in range(num_obs):
        cx = rng.uniform(-half, half)
        cy = rng.uniform(-half, half)
        radius = rng.uniform(0.3, 0.8)
        obs_h = rng.uniform(2.0, height)
        if math.hypot(cx, cy) < clear_radius:
            continue
        n = max(int(2 * math.pi * radius / resolution), 8)
        th = np.arange(n) * (2 * math.pi / n)
        zs = np.arange(0.0, obs_h + 1e-9, resolution)
        ring = np.column_stack([cx + radius * np.cos(th), cy + radius * np.sin(th)])
        chunks.append(np.column_stack([np.tile(ring, (len(zs), 1)), np.repeat(zs, n)]))
    if not chunks:
        return np.empty((0, 3), dtype=np.float32)
    return np.vstack(chunks).astype(np.float32)


class PointCloudGenerator(Node):
    def __init__(self) -> None:
        super().__init__('apf_pointcloud_generator')
        defaults = {
            'topic': '/map_generator/global_cloud',
            'frame_id': 'world',
            'rate_hz': 1.0,
            'num_obs': 35,
            'map_size': 25.0,
            'height': 4.0,
            'resolution': 0.15,
            'clear_radius': 2.0,
            'seed': -1,
            'publish_static_tf': True,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value

        seed = int(g('seed'))
        rng = random.Random(None if seed < 0 else seed)
        pts = generate_forest(
            rng, int(g('num_obs')), float(g('map_size')), float(g('height')),
            float(g('resolution')), float(g('clear_radius')))

        self.frame_id = str(g('frame_id'))
        self.msg = PointCloud2()
        self.msg.header.frame_id = self.frame_id
        self.msg.height = 1
        self.msg.width = len(pts)
        self.msg.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate('xyz')
        ]
        self.msg.is_bigendian = False
        self.msg.point_step = 12
        self.msg.row_step = 12 * len(pts)
        self.msg.is_dense = True
        self.msg.data = pts.tobytes()

        self.pub = self.create_publisher(PointCloud2, str(g('topic')), 10)
        if bool(g('publish_static_tf')):
            self._tf = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.frame_id
            t.child_frame_id = 'map'
            t.transform.rotation.w = 1.0
            self._tf.sendTransform(t)
        self.create_timer(1.0 / float(g('rate_hz')), self.publish)
        self.get_logger().info(f'Generated {len(pts)} obstacle points')

    def publish(self) -> None:
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
