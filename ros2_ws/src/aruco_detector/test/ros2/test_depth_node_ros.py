import numpy as np
import pytest
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from aruco_detector.depth_to_image_node import DepthToImageNode
from marker_utils import make_image_msg

OUTSIDE = int((1.0 - 0.2) / (8.0 - 0.2) * 255)


class Bench:
    def __init__(self, rig):
        self.rig = rig
        p = rig.probe
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.depth_pub = p.create_publisher(Image, '/depth_camera', qos)
        self.poly_pub = p.create_publisher(PolygonStamped, '/hpad/mask_polygon', 10)
        self.out = []
        p.create_subscription(Image, '/depth_camera/image_mono', self.out.append, 10)
        rig.wait_match(['/depth_camera', '/hpad/mask_polygon'], ['/depth_camera/image_mono'])

    def polygon(self, sec=100, nsec=0, points=((20, 10), (40, 10), (40, 30), (20, 30))):
        msg = PolygonStamped()
        msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nsec
        msg.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in points]
        self.poly_pub.publish(msg)
        self.rig.spin(0.3)

    def send(self, arr, encoding, stamp=(100, 0), sec=0.6):
        self.out.clear()
        msg = make_image_msg(arr, encoding, stamp, 'depth_frame')
        self.rig.drive(sec, lambda: self.depth_pub.publish(msg), hz=10.0)

    def last(self):
        m = self.out[-1]
        return np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width)


@pytest.fixture
def bench(make_rig):
    return lambda: Bench(make_rig(DepthToImageNode))


def flat(value=1.0):
    return np.full((60, 80), value, dtype=np.float32)


def test_float32_depth_is_normalized(bench):
    b = bench()
    b.send(flat(), '32FC1')
    m = b.out[-1]
    assert (m.encoding, m.width, m.height, m.step) == ('mono8', 80, 60, 80)
    assert (b.last() == OUTSIDE).all()


def test_uint16_depth_in_millimeters(bench):
    b = bench()
    b.send(np.full((60, 80), 1000, dtype=np.uint16), '16UC1')
    assert (b.last() == OUTSIDE).all()


def test_header_is_preserved(bench):
    b = bench()
    b.send(flat(), '32FC1', stamp=(77, 5))
    assert b.out[-1].header.frame_id == 'depth_frame'
    assert (b.out[-1].header.stamp.sec, b.out[-1].header.stamp.nanosec) == (77, 5)


def test_unsupported_encoding_publishes_nothing(bench):
    b = bench()
    b.send(np.zeros((60, 80), dtype=np.uint8), 'mono8')
    assert not b.out


def test_fresh_polygon_masks_region_as_background(bench):
    b = bench()
    b.polygon(sec=100)
    b.send(flat(), '32FC1', stamp=(100, 0))
    img = b.last()
    assert img[20, 30] == 255
    assert img[0, 0] == OUTSIDE and img[50, 70] == OUTSIDE


def test_polygon_within_tolerance_still_applies(bench):
    b = bench()
    b.polygon(sec=100, nsec=0)
    b.send(flat(), '32FC1', stamp=(100, 150_000_000))
    assert b.last()[20, 30] == 255


def test_stale_polygon_is_ignored(bench):
    b = bench()
    b.polygon(sec=100)
    b.send(flat(), '32FC1', stamp=(101, 0))
    assert (b.last() == OUTSIDE).all()


def test_no_polygon_no_mask(bench):
    b = bench()
    b.send(flat(), '32FC1', stamp=(100, 0))
    assert (b.last() == OUTSIDE).all()


def test_degenerate_polygon_does_not_mask(bench):
    b = bench()
    b.polygon(sec=100, points=((20, 10), (40, 10)))
    b.send(flat(), '32FC1', stamp=(100, 0))
    assert (b.last() == OUTSIDE).all()


def test_mask_follows_newest_polygon(bench):
    b = bench()
    b.polygon(sec=100, points=((5, 5), (15, 5), (15, 15), (5, 15)))
    b.polygon(sec=100, points=((50, 30), (70, 30), (70, 50), (50, 50)))
    b.send(flat(), '32FC1', stamp=(100, 0))
    img = b.last()
    assert img[40, 60] == 255 and img[10, 10] == OUTSIDE