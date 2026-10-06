import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import PointCloud2, PointField


def init_ros(params=None):
    args = ['--ros-args']
    for key, value in (params or {}).items():
        if isinstance(value, bool):
            value = 'true' if value else 'false'
        args += ['-p', f'{key}:={value}']
    rclpy.init(args=args)


class Rig:
    def __init__(self, node):
        self.node = node
        self.probe = rclpy.create_node('apf_test_probe')
        self.exec = SingleThreadedExecutor()
        self.exec.add_node(node)
        self.exec.add_node(self.probe)

    def spin(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            self.exec.spin_once(timeout_sec=0.005)

    def drive(self, sec, fn, hz=20.0):
        period = 1.0 / hz
        end = time.monotonic() + sec
        nxt = time.monotonic()
        while time.monotonic() < end:
            if time.monotonic() >= nxt:
                fn()
                nxt += period
            self.exec.spin_once(timeout_sec=0.005)

    def wait_match(self, probe_pubs=(), probe_subs=(), timeout=8.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.exec.spin_once(timeout_sec=0.01)
            ok_pubs = all(self.probe.count_subscribers(t) > 0 for t in probe_pubs)
            ok_subs = all(self.probe.count_publishers(t) > 0 for t in probe_subs)
            if ok_pubs and ok_subs:
                break
        self.spin(0.3)

    def close(self):
        self.exec.shutdown()
        self.node.destroy_node()
        self.probe.destroy_node()
        rclpy.shutdown()


def make_odom(pos, vel=(0.0, 0.0, 0.0), frame='world'):
    m = Odometry()
    m.header.frame_id = frame
    m.pose.pose.position.x = float(pos[0])
    m.pose.pose.position.y = float(pos[1])
    m.pose.pose.position.z = float(pos[2])
    m.pose.pose.orientation.w = 1.0
    m.twist.twist.linear.x = float(vel[0])
    m.twist.twist.linear.y = float(vel[1])
    m.twist.twist.linear.z = float(vel[2])
    return m


def make_cloud(points, frame='world'):
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    m = PointCloud2()
    m.header.frame_id = frame
    m.height = 1
    m.width = len(pts)
    m.fields = [
        PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate('xyz')
    ]
    m.is_bigendian = False
    m.point_step = 12
    m.row_step = 12 * len(pts)
    m.is_dense = True
    m.data = pts.tobytes()
    return m