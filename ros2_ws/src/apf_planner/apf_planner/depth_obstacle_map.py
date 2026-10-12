import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


def decode_depth(msg):
    if msg.encoding in ('32FC1', 'R_FLOAT32'):
        return np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
    if msg.encoding == '16UC1':
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
        return raw.astype(np.float32) / 1000.0
    return None


def depth_to_points(depth, K, stride, zmin, zmax):
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    ok = np.isfinite(z) & (z >= zmin) & (z <= zmax)
    z, u, v = z[ok], u[ok], v[ok]
    return np.column_stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z])


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class VoxelMap:
    def __init__(self, voxel, ttl):
        self.voxel = voxel
        self.ttl = ttl
        self.cells = {}

    def insert(self, pts, t):
        if len(pts) == 0:
            return
        keys = np.floor(pts / self.voxel).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        for i in idx:
            self.cells[tuple(keys[i])] = (pts[i], t)

    def expire(self, t):
        self.cells = {k: v for k, v in self.cells.items() if t - v[1] <= self.ttl}

    def points(self, center, radius):
        if not self.cells:
            return np.empty((0, 3), dtype=np.float32)
        p = np.array([v[0] for v in self.cells.values()])
        return p[np.linalg.norm(p - center, axis=1) <= radius].astype(np.float32)


class DepthObstacleMap(Node):
    def __init__(self):
        super().__init__('depth_obstacle_map')
        defaults = {
            'depth_topic': '/depth_camera',
            'info_topic': '/camera/infra1/camera_info',
            'cloud_topic': '/map_generator/global_cloud',
            'world_frame': 'world',
            'camera_frame': 'camera_optical_frame',
            'stride': 8,
            'range_min': 0.3,
            'range_max': 5.0,
            'ground_z': 0.0,
            'ground_margin': 0.3,
            'ceiling_z': 8.0,
            'voxel_size': 0.3,
            'ttl_sec': 5.0,
            'publish_radius': 6.0,
            'process_hz': 5.0,
            'publish_hz': 5.0,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.world = str(g('world_frame'))
        self.cam = str(g('camera_frame'))
        self.stride = int(g('stride'))
        self.zmin = float(g('range_min'))
        self.zmax = float(g('range_max'))
        self.zlow = float(g('ground_z')) + float(g('ground_margin'))
        self.zhigh = float(g('ceiling_z'))
        self.radius = float(g('publish_radius'))
        self.period = 1.0 / float(g('process_hz'))
        self.map = VoxelMap(float(g('voxel_size')), float(g('ttl_sec')))
        self.K = None
        self.center = np.zeros(3)
        self.last = -1e9
        self.skipped = 0
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(CameraInfo, str(g('info_topic')), self.info_cb, qos)
        self.create_subscription(Image, str(g('depth_topic')), self.depth_cb, qos)
        self.pub = self.create_publisher(PointCloud2, str(g('cloud_topic')), 1)
        self.create_timer(1.0 / float(g('publish_hz')), self.publish)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def info_cb(self, msg):
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.K = np.array(msg.k, dtype=float).reshape(3, 3)

    def depth_cb(self, msg):
        now = self.now()
        if self.K is None or now - self.last < self.period:
            return
        depth = decode_depth(msg)
        if depth is None:
            return
        try:
            tf = self.tfb.lookup_transform(
                self.world, self.cam, Time.from_msg(msg.header.stamp))
        except Exception:
            self.skipped += 1
            return
        self.last = now
        t, q = tf.transform.translation, tf.transform.rotation
        p0 = np.array([t.x, t.y, t.z])
        self.center = p0
        pc = depth_to_points(depth, self.K, self.stride, self.zmin, self.zmax)
        if len(pc) == 0:
            return
        pw = pc @ quat_to_matrix((q.x, q.y, q.z, q.w)).T + p0
        keep = (pw[:, 2] > self.zlow) & (pw[:, 2] < self.zhigh)
        self.map.insert(pw[keep], now)

    def publish(self):
        self.map.expire(self.now())
        pts = self.map.points(self.center, self.radius)
        h = Header()
        h.frame_id = self.world
        h.stamp = self.get_clock().now().to_msg()
        self.pub.publish(point_cloud2.create_cloud_xyz32(h, pts.tolist()))


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()