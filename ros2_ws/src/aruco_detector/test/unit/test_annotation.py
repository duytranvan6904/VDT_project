import numpy as np
import pytest

from aruco_detector.annotation import (
    draw_detections,
    ensure_bgr,
    project_axis_points,
)
from aruco_detector.aruco_logic import ArUcoDetector
from aruco_detector.camera_model import fallback_camera_matrix, zero_distortion
from marker_utils import FOV, HEIGHT, MARKER_SIZE_M, WIDTH, make_frame


def detect(frame):
    matrix = fallback_camera_matrix(WIDTH, HEIGHT, FOV)
    detector = ArUcoDetector('DICT_6X6_50', MARKER_SIZE_M, matrix, zero_distortion(), 42)
    return detector.process_frame(frame), matrix


def count_color(image, bgr):
    return int(np.all(image == np.array(bgr, dtype=np.uint8), axis=2).sum())


def test_ensure_bgr_from_gray():
    out = ensure_bgr(np.full((4, 5), 7, dtype=np.uint8))
    assert out.shape == (4, 5, 3) and (out == 7).all()


def test_ensure_bgr_copies_color_input():
    src = np.zeros((4, 5, 3), dtype=np.uint8)
    out = ensure_bgr(src)
    out[0, 0] = 255
    assert src[0, 0].tolist() == [0, 0, 0]


def test_no_detections_returns_plain_bgr():
    frame = make_frame(with_marker=False)
    out = draw_detections(frame, [], fallback_camera_matrix(WIDTH, HEIGHT, FOV),
                          zero_distortion(), MARKER_SIZE_M)
    assert out.shape == (HEIGHT, WIDTH, 3)
    assert (out == 255).all()


def test_draw_does_not_modify_input():
    frame = make_frame(z=1.0)
    before = frame.copy()
    detections, matrix = detect(frame)
    draw_detections(frame, detections, matrix, zero_distortion(), MARKER_SIZE_M)
    assert np.array_equal(frame, before)


def test_outline_label_and_axes_are_drawn():
    frame = make_frame(z=1.0)
    detections, matrix = detect(frame)
    out = draw_detections(frame, detections, matrix, zero_distortion(), MARKER_SIZE_M)
    assert out.shape == (HEIGHT, WIDTH, 3)
    assert count_color(out, (0, 255, 0)) > 50
    assert count_color(out, (0, 0, 255)) > 10
    assert count_color(out, (255, 0, 0)) > 0


def test_label_box_near_first_corner():
    frame = make_frame(z=1.0)
    detections, matrix = detect(frame)
    out = draw_detections(frame, detections, matrix, zero_distortion(), MARKER_SIZE_M)
    cx, cy = detections[0]['corners'][0].astype(int)
    patch = out[max(cy - 25, 0):cy + 2, max(cx - 2, 0):cx + 70]
    assert count_color(patch, (0, 255, 0)) > 20


def test_axis_points_shape_and_origin():
    detections, matrix = detect(make_frame(z=1.0, dx=40, dy=20))
    pts = project_axis_points(detections[0], matrix, zero_distortion(), 0.2)
    assert pts.shape == (4, 2)
    assert pts[0] == pytest.approx([WIDTH / 2 + 40, HEIGHT / 2 + 20], abs=3)


def test_axis_length_scales_projection():
    detections, matrix = detect(make_frame(z=1.0))
    short = project_axis_points(detections[0], matrix, zero_distortion(), 0.1)
    long = project_axis_points(detections[0], matrix, zero_distortion(), 0.2)
    assert np.linalg.norm(long[1] - long[0]) > np.linalg.norm(short[1] - short[0])