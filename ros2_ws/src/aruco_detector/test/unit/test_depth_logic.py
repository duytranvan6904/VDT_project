from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32, PolygonStamped
from std_msgs.msg import Header

from aruco_detector.depth_to_image_node import (
    apply_polygon_mask,
    build_mono8_message,
    decode_depth_meters,
    normalize_depth_to_mono8,
    stamp_to_seconds,
)


def depth_msg(arr, encoding):
    return SimpleNamespace(data=arr.tobytes(), height=arr.shape[0], width=arr.shape[1],
                           encoding=encoding)


def polygon(points):
    msg = PolygonStamped()
    msg.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in points]
    return msg


def test_decode_float32():
    arr = np.array([[1.0, 2.5]], dtype=np.float32)
    assert decode_depth_meters(depth_msg(arr, '32FC1')).tolist() == [[1.0, 2.5]]
    assert decode_depth_meters(depth_msg(arr, 'R_FLOAT32')).shape == (1, 2)


def test_decode_uint16_millimeters():
    arr = np.array([[1000, 2500]], dtype=np.uint16)
    assert decode_depth_meters(depth_msg(arr, '16UC1')) == pytest.approx(np.array([[1.0, 2.5]]))


def test_decode_unsupported_encoding():
    assert decode_depth_meters(depth_msg(np.zeros((1, 1), dtype=np.uint8), 'mono8')) is None


def test_normalize_endpoints_and_midpoint():
    out = normalize_depth_to_mono8(np.array([[0.2, 8.0, 4.1]], dtype=np.float32))
    assert out.dtype == np.uint8
    assert out.tolist() == [[0, 255, 127]]


def test_normalize_clips_out_of_range():
    out = normalize_depth_to_mono8(np.array([[0.0, 0.1, 20.0]], dtype=np.float32))
    assert out.tolist() == [[0, 0, 255]]


def test_normalize_invalid_values():
    out = normalize_depth_to_mono8(np.array([[np.nan, np.inf, -np.inf]], dtype=np.float32))
    assert out.tolist() == [[255, 255, 0]]


def test_build_mono8_message():
    header = Header()
    header.frame_id = 'depth'
    mono = np.arange(6, dtype=np.uint8).reshape(2, 3)
    m = build_mono8_message(mono, header)
    assert (m.height, m.width, m.encoding, m.step) == (2, 3, 'mono8', 3)
    assert bytes(m.data) == mono.tobytes() and m.header.frame_id == 'depth'


def test_stamp_to_seconds():
    assert stamp_to_seconds(Time(sec=3, nanosec=500_000_000)) == pytest.approx(3.5)


def test_apply_polygon_mask_sets_nan_inside_only():
    depth = np.ones((10, 10), dtype=np.float32)
    out = apply_polygon_mask(depth, polygon([(2, 2), (7, 2), (7, 7), (2, 7)]))
    assert np.isnan(out[4, 4])
    assert out[0, 0] == 1.0 and out[9, 9] == 1.0
    assert np.isnan(out).sum() > 0


def test_apply_polygon_mask_does_not_mutate_input():
    depth = np.ones((10, 10), dtype=np.float32)
    apply_polygon_mask(depth, polygon([(2, 2), (7, 2), (7, 7), (2, 7)]))
    assert not np.isnan(depth).any()


def test_apply_polygon_mask_requires_three_points():
    depth = np.ones((5, 5), dtype=np.float32)
    assert apply_polygon_mask(depth, polygon([(1, 1), (3, 3)])) is depth


def test_apply_polygon_mask_converts_dtype_to_float32():
    out = apply_polygon_mask(np.ones((10, 10), dtype=np.float64),
                             polygon([(2, 2), (7, 2), (7, 7), (2, 7)]))
    assert out.dtype == np.float32