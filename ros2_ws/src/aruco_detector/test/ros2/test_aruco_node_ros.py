import numpy as np
import pytest
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from vision_msgs.msg import BoundingBox2D

from aruco_detector.aruco_node import ArucoNode
from marker_utils import (
    HEIGHT, WIDTH, expected_z, focal, make_camera_info, make_frame, make_image_msg)

BASE = {'marker_size_m': 0.3}


class Bench:
    def __init__(self, rig):
        self.rig = rig
        p = rig.probe
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=3)
        self.img_pub = p.create_publisher(Image, '/camera', qos)
        self.info_pub = p.create_publisher(CameraInfo, '/camera_info', qos)
        self.detected, self.pose, self.point, self.bbox, self.annotated = [], [], [], [], []
        p.create_subscription(Bool, '/hpad/detected', lambda m: self.detected.append(m.data), 10)
        p.create_subscription(PoseStamped, '/hpad/pose', self.pose.append, 10)
        p.create_subscription(PointStamped, '/hpad/position_camera', self.point.append, 10)
        p.create_subscription(BoundingBox2D, '/hpad/bbox', self.bbox.append, 10)
        p.create_subscription(Image, '/hpad/annotated', self.annotated.append, 10)
        rig.wait_match(['/camera', '/camera_info'],
                       ['/hpad/detected', '/hpad/pose', '/hpad/annotated'])

    def run(self, sec, frame, encoding='mono8', info=None, stamp=(0, 0)):
        for lst in (self.detected, self.pose, self.point, self.bbox, self.annotated):
            lst.clear()
        if encoding == 'mono16':
            arr = frame.astype(np.uint16) << 8
        elif encoding == 'rgb8':
            arr = np.dstack([frame] * 3)
        elif encoding == '32FC1':
            arr = frame.astype(np.float32)
        else:
            arr = frame
        image = make_image_msg(arr, encoding, stamp)

        def send():
            if info is not None:
                self.info_pub.publish(info)
            self.img_pub.publish(image)

        self.rig.drive(sec, send)


@pytest.fixture
def bench(make_rig):
    def factory(params=None):
        merged = dict(BASE)
        merged.update(params or {})
        return Bench(make_rig(ArucoNode, merged))
    return factory


def test_blank_frames_publish_false_only(bench):
    b = bench()
    b.run(1.0, make_frame(with_marker=False), info=make_camera_info())
    assert len(b.detected) >= 5 and not any(b.detected)
    assert not b.pose and not b.point and not b.bbox
    assert len(b.annotated) >= 5


def test_marker_publishes_all_outputs(bench):
    b = bench()
    b.run(1.2, make_frame(z=1.0, dx=40, dy=-20), info=make_camera_info())
    assert b.detected and b.detected[-1]
    pose, point, bbox = b.pose[-1], b.point[-1], b.bbox[-1]
    z = expected_z(1.0)
    assert pose.pose.position.z == pytest.approx(z, rel=0.05)
    assert pose.pose.position.x == pytest.approx(40 * z / focal(), abs=0.03)
    assert pose.pose.position.y == pytest.approx(-20 * z / focal(), abs=0.03)
    assert (point.point.x, point.point.y, point.point.z) == pytest.approx(
        (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
    assert bbox.center.position.x == pytest.approx(WIDTH / 2 + 40, abs=3)
    assert bbox.center.position.y == pytest.approx(HEIGHT / 2 - 20, abs=3)
    assert bbox.size_x == pytest.approx(bbox.size_y, abs=3)
    o = pose.pose.orientation
    assert np.linalg.norm([o.w, o.x, o.y, o.z]) == pytest.approx(1.0, abs=1e-6)


def test_annotated_image_format(bench):
    b = bench()
    b.run(1.0, make_frame(z=1.0), info=make_camera_info())
    img = b.annotated[-1]
    assert (img.encoding, img.width, img.height, img.step) == ('bgr8', WIDTH, HEIGHT, WIDTH * 3)
    assert len(img.data) == WIDTH * HEIGHT * 3


def test_frame_id_parameter(bench):
    b = bench({'camera_frame_id': 'my_cam'})
    b.run(1.0, make_frame(z=1.0), info=make_camera_info())
    assert b.pose[-1].header.frame_id == 'my_cam'
    assert b.point[-1].header.frame_id == 'my_cam'


def test_default_frame_id(bench):
    b = bench()
    b.run(1.0, make_frame(z=1.0), info=make_camera_info())
    assert b.pose[-1].header.frame_id == 'camera_optical_frame'


def test_zero_header_stamp_replaced_by_node_clock(bench):
    b = bench()
    b.run(1.0, make_frame(z=1.0), info=make_camera_info(), stamp=(0, 0))
    assert b.pose[-1].header.stamp.sec > 0


def test_header_stamp_preserved(bench):
    b = bench()
    b.run(1.0, make_frame(z=1.0), info=make_camera_info(), stamp=(123, 456))
    assert (b.pose[-1].header.stamp.sec, b.pose[-1].header.stamp.nanosec) == (123, 456)


def test_pose_suppressed_without_camera_info(bench):
    b = bench()
    b.run(1.0, make_frame(z=1.0), info=None)
    assert b.detected and not any(b.detected)
    assert not b.pose


def test_pose_published_with_fallback_when_info_not_required(bench):
    b = bench({'require_camera_info': False})
    b.run(1.0, make_frame(z=1.0), info=None)
    assert b.detected[-1]
    assert b.pose[-1].pose.position.z == pytest.approx(expected_z(1.0), rel=0.05)


def test_invalid_camera_info_is_ignored(bench):
    b = bench()
    info = make_camera_info()
    info.k = [0.0] * 9
    b.run(1.0, make_frame(z=1.0), info=info)
    assert not b.pose


def test_camera_info_intrinsics_change_depth(bench):
    b = bench()
    b.run(1.2, make_frame(z=1.0), info=make_camera_info(fx=2 * focal()))
    assert b.pose[-1].pose.position.z == pytest.approx(2 * expected_z(1.0), rel=0.06)


def test_min_z_filter(bench):
    b = bench({'min_z_m': 2.0})
    b.run(1.0, make_frame(z=1.0), info=make_camera_info())
    assert not any(b.detected) and not b.pose


def test_min_distance_filter(bench):
    b = bench({'min_detection_distance_m': 2.0})
    b.run(1.0, make_frame(z=1.0), info=make_camera_info())
    assert not any(b.detected)
    b2_params = None
    assert b2_params is None


def test_min_distance_below_actual_keeps_detection(bench):
    b = bench({'min_detection_distance_m': 0.5, 'min_z_m': 0.5})
    b.run(1.0, make_frame(z=1.0), info=make_camera_info())
    assert b.detected[-1]


def test_wrong_marker_id_not_detected(bench):
    b = bench({'marker_id': 7})
    b.run(1.0, make_frame(z=1.0), info=make_camera_info())
    assert not any(b.detected)


def test_custom_marker_id_detected(bench):
    b = bench({'marker_id': 7})
    b.run(1.0, make_frame(z=1.0, marker_id=7), info=make_camera_info())
    assert b.detected[-1]


def test_marker_size_parameter_scales_depth(bench):
    b = bench({'marker_size_m': 0.15})
    b.run(1.0, make_frame(z=1.0), info=make_camera_info())
    assert b.pose[-1].pose.position.z == pytest.approx(expected_z(1.0) / 2, rel=0.06)


@pytest.mark.parametrize('encoding', ['mono16', 'rgb8'])
def test_other_encodings_supported(bench, encoding):
    b = bench()
    b.run(1.0, make_frame(z=1.0), encoding=encoding, info=make_camera_info())
    assert b.detected[-1]


def test_unsupported_encoding_publishes_nothing(bench):
    b = bench()
    b.run(1.0, make_frame(z=1.0), encoding='32FC1', info=make_camera_info())
    assert not b.detected and not b.annotated


def test_detected_flag_returns_to_false_after_marker_leaves(bench):
    b = bench()
    b.run(0.8, make_frame(z=1.0), info=make_camera_info())
    assert b.detected[-1]
    b.run(0.8, make_frame(with_marker=False), info=make_camera_info())
    assert not b.detected[-1]


def test_non_default_resolution_uses_fallback_matrix(bench):
    b = bench({'require_camera_info': False})
    frame = make_frame(z=1.0, width=320, height=240)
    b.run(1.0, frame, info=None)
    assert b.detected[-1]
    assert b.pose[-1].pose.position.z == pytest.approx(expected_z(1.0, width=320), rel=0.08)