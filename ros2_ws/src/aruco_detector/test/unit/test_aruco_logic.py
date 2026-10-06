import cv2
import numpy as np
import pytest

from aruco_detector.aruco_logic import (
    ArUcoDetector,
    build_pose_dict,
    build_square_object_points,
    create_dictionary,
    create_detector_parameters,
    to_grayscale,
    tune_detector_parameters,
)
from aruco_detector.camera_model import fallback_camera_matrix, zero_distortion
from marker_utils import FOV, HEIGHT, MARKER_SIZE_M, WIDTH, expected_z, focal, make_frame


def make_detector(marker_id=42, size=MARKER_SIZE_M, dictionary='DICT_6X6_50'):
    return ArUcoDetector(dictionary, size, fallback_camera_matrix(WIDTH, HEIGHT, FOV),
                         zero_distortion(), marker_id)


def test_create_dictionary_valid():
    assert create_dictionary('DICT_6X6_50') is not None
    assert create_dictionary('DICT_4X4_50') is not None


@pytest.mark.parametrize('name', ['DICT_FAKE', 'getPredefinedDictionary', '', 'CORNER_REFINE_SUBPIX'])
def test_create_dictionary_invalid(name):
    with pytest.raises(ValueError):
        create_dictionary(name)


def test_tuned_parameters():
    p = tune_detector_parameters(create_detector_parameters())
    assert p.adaptiveThreshWinSizeMin == 3 and p.adaptiveThreshWinSizeMax == 53
    assert p.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_SUBPIX
    assert p.minDistanceToBorder == 3


def test_to_grayscale():
    gray = np.zeros((4, 5), dtype=np.uint8)
    assert to_grayscale(gray) is gray
    assert to_grayscale(np.zeros((4, 5, 3), dtype=np.uint8)).shape == (4, 5)


def test_square_object_points():
    pts = build_square_object_points(0.2)
    assert pts.shape == (4, 3) and pts.dtype == np.float32
    assert pts == pytest.approx(np.array([[-.1, .1, 0], [.1, .1, 0], [.1, -.1, 0], [-.1, -.1, 0]]))


def test_pose_dict():
    pose = build_pose_dict(np.zeros(3), np.array([1.0, 2.0, 2.0]))
    assert (pose['x'], pose['y'], pose['z']) == (1.0, 2.0, 2.0)
    assert pose['distance'] == pytest.approx(3.0)
    assert pose['quaternion'] == pytest.approx([1, 0, 0, 0])


def test_detects_target_with_accurate_depth():
    det = make_detector().process_frame(make_frame(z=1.0))
    assert len(det) == 1
    assert det[0]['id'] == 42
    assert det[0]['pose']['z'] == pytest.approx(expected_z(1.0), rel=0.05)
    assert abs(det[0]['pose']['x']) < 0.03 and abs(det[0]['pose']['y']) < 0.03


@pytest.mark.parametrize('z', [0.8, 1.5])
def test_depth_scales_with_distance(z):
    det = make_detector().process_frame(make_frame(z=z))
    assert det[0]['pose']['z'] == pytest.approx(expected_z(z), rel=0.06)


def test_lateral_offset_maps_to_xy():
    det = make_detector().process_frame(make_frame(z=1.0, dx=60, dy=-40))
    z = expected_z(1.0)
    pose = det[0]['pose']
    assert pose['x'] == pytest.approx(60 * z / focal(), abs=0.03)
    assert pose['y'] == pytest.approx(-40 * z / focal(), abs=0.03)


def test_distance_consistent_with_components():
    pose = make_detector().process_frame(make_frame(z=1.0, dx=50))[0]['pose']
    assert pose['distance'] == pytest.approx(np.sqrt(pose['x'] ** 2 + pose['y'] ** 2 + pose['z'] ** 2))


def test_inplane_rotation_still_detected():
    det = make_detector().process_frame(make_frame(z=1.0, angle_deg=30))
    assert len(det) == 1
    assert det[0]['pose']['z'] == pytest.approx(expected_z(1.0), rel=0.06)


def test_quaternion_is_unit():
    q = make_detector().process_frame(make_frame(z=1.0))[0]['pose']['quaternion']
    assert np.linalg.norm(q) == pytest.approx(1.0, abs=1e-6)


def test_ignores_other_marker_ids():
    assert make_detector().process_frame(make_frame(z=1.0, marker_id=7)) == []


def test_selects_target_among_multiple():
    det = make_detector().process_frame(make_frame(z=1.5, extra=[(7, -200, 0), (9, 200, 0)]))
    assert len(det) == 1 and det[0]['id'] == 42
    assert abs(det[0]['pose']['x']) < 0.05


def test_blank_image_no_detection():
    assert make_detector().process_frame(make_frame(with_marker=False)) == []


def test_bgr_input_supported():
    frame = make_frame(z=1.0)
    assert len(make_detector().process_frame(np.dstack([frame] * 3))) == 1


def test_set_camera_parameters_copies_and_changes_depth():
    detector = make_detector()
    frame = make_frame(z=1.0)
    base = detector.process_frame(frame)[0]['pose']['z']
    matrix = fallback_camera_matrix(WIDTH, HEIGHT, FOV)
    matrix[0, 0] *= 2
    matrix[1, 1] *= 2
    detector.set_camera_parameters(matrix, zero_distortion())
    matrix[0, 0] = 1.0
    assert detector.camera_matrix[0, 0] != 1.0
    assert detector.process_frame(frame)[0]['pose']['z'] == pytest.approx(2 * base, rel=0.05)


def test_marker_size_scales_depth():
    frame = make_frame(z=1.0)
    small = make_detector(size=0.15).process_frame(frame)[0]['pose']['z']
    big = make_detector(size=0.30).process_frame(frame)[0]['pose']['z']
    assert big == pytest.approx(2 * small, rel=0.05)


def test_other_dictionary_does_not_match():
    assert make_detector(dictionary='DICT_4X4_50').process_frame(make_frame(z=1.0)) == []