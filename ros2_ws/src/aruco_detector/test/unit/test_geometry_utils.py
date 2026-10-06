import cv2
import numpy as np
import pytest

from aruco_detector.geometry_utils import (
    compute_corner_bounding_box,
    rotation_matrix_to_quaternion_wxyz,
    rvec_to_quaternion_wxyz,
)


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def test_identity_quaternion():
    assert rotation_matrix_to_quaternion_wxyz(np.eye(3)) == pytest.approx([1, 0, 0, 0])


@pytest.mark.parametrize('diag,expected', [
    ((1, -1, -1), [0, 1, 0, 0]),
    ((-1, 1, -1), [0, 0, 1, 0]),
    ((-1, -1, 1), [0, 0, 0, 1]),
])
def test_half_turn_branches(diag, expected):
    q = rotation_matrix_to_quaternion_wxyz(np.diag(diag).astype(float))
    assert q == pytest.approx(expected)


def test_quarter_turn_about_z():
    q = rvec_to_quaternion_wxyz(np.array([0.0, 0.0, np.pi / 2]))
    assert q == pytest.approx([np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)])


def test_random_rotations_roundtrip_and_unit_norm():
    rng = np.random.default_rng(0)
    for _ in range(300):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        rvec = axis * rng.uniform(0.0, np.pi)
        rotation, _ = cv2.Rodrigues(rvec)
        q = rotation_matrix_to_quaternion_wxyz(rotation)
        assert np.linalg.norm(q) == pytest.approx(1.0, abs=1e-6)
        assert quat_to_matrix(q) == pytest.approx(rotation, abs=1e-6)


def test_rvec_column_shape_supported():
    q = rvec_to_quaternion_wxyz(np.array([[0.0], [0.0], [0.5]]))
    assert np.linalg.norm(q) == pytest.approx(1.0)


def test_bbox_axis_aligned():
    corners = np.array([[10, 20], [30, 20], [30, 40], [10, 40]], dtype=float)
    assert compute_corner_bounding_box(corners) == pytest.approx((20.0, 30.0, 20.0, 20.0))


def test_bbox_accepts_opencv_shape():
    corners = np.array([[[10, 20], [30, 20], [30, 40], [10, 40]]], dtype=np.float32)
    assert compute_corner_bounding_box(corners) == pytest.approx((20.0, 30.0, 20.0, 20.0))


def test_bbox_rotated_diamond():
    corners = np.array([[10, 0], [20, 10], [10, 20], [0, 10]], dtype=float)
    assert compute_corner_bounding_box(corners) == pytest.approx((10.0, 10.0, 20.0, 20.0))


def test_bbox_returns_python_floats():
    out = compute_corner_bounding_box(np.zeros((4, 2)))
    assert all(isinstance(v, float) for v in out)