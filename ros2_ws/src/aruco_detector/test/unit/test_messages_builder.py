import numpy as np
import pytest
from builtin_interfaces.msg import Time

from aruco_detector.message_builders import (
    build_bbox_message,
    build_bgr_image_message,
    build_point_message,
    build_pose_message,
    resolve_stamp,
)

DETECTION = {
    'pose': {'x': 1.0, 'y': 2.0, 'z': 3.0, 'quaternion': [0.1, 0.2, 0.3, 0.4]},
    'corners': np.array([[10, 20], [30, 20], [30, 50], [10, 50]], dtype=np.float32),
}


def test_resolve_stamp_prefers_header():
    assert resolve_stamp(Time(sec=5, nanosec=0), Time(sec=9)).sec == 5
    assert resolve_stamp(Time(sec=0, nanosec=7), Time(sec=9)).nanosec == 7


def test_resolve_stamp_falls_back_when_zero():
    assert resolve_stamp(Time(), Time(sec=9, nanosec=1)).sec == 9


def test_pose_message_maps_wxyz_to_ros_fields():
    m = build_pose_message(DETECTION, Time(sec=3), 'cam')
    assert m.header.frame_id == 'cam' and m.header.stamp.sec == 3
    assert (m.pose.position.x, m.pose.position.y, m.pose.position.z) == (1.0, 2.0, 3.0)
    o = m.pose.orientation
    assert (o.w, o.x, o.y, o.z) == pytest.approx((0.1, 0.2, 0.3, 0.4))


def test_point_message():
    m = build_point_message(DETECTION, Time(sec=4), 'cam')
    assert m.header.frame_id == 'cam' and m.header.stamp.sec == 4
    assert (m.point.x, m.point.y, m.point.z) == (1.0, 2.0, 3.0)


def test_bbox_message():
    m = build_bbox_message(DETECTION)
    assert (m.center.position.x, m.center.position.y) == pytest.approx((20.0, 35.0))
    assert (m.size_x, m.size_y) == pytest.approx((20.0, 30.0))
    assert m.center.theta == 0.0


def test_bgr_image_message():
    from std_msgs.msg import Header
    header = Header()
    header.frame_id = 'cam'
    image = np.zeros((4, 6, 3), dtype=np.uint8)
    image[1, 2] = [1, 2, 3]
    m = build_bgr_image_message(image, header)
    assert (m.height, m.width, m.encoding, m.step) == (4, 6, 'bgr8', 18)
    assert len(m.data) == 4 * 6 * 3
    assert m.header.frame_id == 'cam'
    assert list(m.data[(1 * 6 + 2) * 3:(1 * 6 + 2) * 3 + 3]) == [1, 2, 3]