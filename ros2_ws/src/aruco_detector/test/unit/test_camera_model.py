from types import SimpleNamespace

import numpy as np
import pytest

from aruco_detector.camera_model import (
    camera_info_to_intrinsics,
    fallback_camera_matrix,
    zero_distortion,
)


def info(k, d=()):
    return SimpleNamespace(k=list(k), d=list(d))


def test_fallback_matrix_values():
    m = fallback_camera_matrix(640, 480, np.pi / 2)
    assert m == pytest.approx(np.array([[320, 0, 320], [0, 320, 240], [0, 0, 1]]))
    assert m.dtype == np.float64


def test_fallback_focal_decreases_with_wider_fov():
    narrow = fallback_camera_matrix(640, 480, 1.0)[0, 0]
    wide = fallback_camera_matrix(640, 480, 2.0)[0, 0]
    assert wide < narrow


def test_fallback_principal_point_tracks_image_size():
    m = fallback_camera_matrix(1280, 720, 1.52)
    assert (m[0, 2], m[1, 2]) == (640.0, 360.0)


def test_zero_distortion_shape():
    d = zero_distortion()
    assert d.shape == (5, 1) and not d.any()


def test_intrinsics_with_distortion():
    k, d = camera_info_to_intrinsics(info([500, 0, 320, 0, 510, 240, 0, 0, 1], [0.1, 0.01, 0, 0, 0]))
    assert k[0, 0] == 500 and k[1, 1] == 510 and k[0, 2] == 320
    assert d.shape == (5, 1)
    assert d[0, 0] == pytest.approx(0.1)


def test_intrinsics_without_distortion_uses_zeros():
    k, d = camera_info_to_intrinsics(info([500, 0, 320, 0, 500, 240, 0, 0, 1]))
    assert d.shape == (5, 1) and not d.any()


@pytest.mark.parametrize('k', [
    [0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 320, 0, 500, 240, 0, 0, 1],
    [500, 0, 320, 0, 0, 240, 0, 0, 1],
    [-1, 0, 320, 0, 500, 240, 0, 0, 1],
])
def test_invalid_k_returns_none(k):
    assert camera_info_to_intrinsics(info(k)) is None


def test_intrinsics_are_independent_copies():
    msg = info([500, 0, 320, 0, 500, 240, 0, 0, 1])
    k, _ = camera_info_to_intrinsics(msg)
    k[0, 0] = 1.0
    assert msg.k[0] == 500