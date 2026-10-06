import os
import pytest
import numpy as np
import cv2

from vision import DualScaleArUcoDetector
from vision.generate_dual_scale_board import create_dual_scale_board


@pytest.fixture
def board_artifacts(tmp_path):
    """Create test board image and verify generated structure."""
    canvas, config = create_dual_scale_board(
        big_id=42,
        big_size_m=0.40,
        small_id=43,
        small_size_m=0.05,
        small_patch_m=0.06,
        margin_m=0.05,
        dict_name="DICT_6X6_50",
        pixels_per_cm=50
    )
    img_path = str(tmp_path / "test_board.png")
    cv2.imwrite(img_path, canvas)
    return canvas, config, img_path


def test_dual_scale_board_generation(board_artifacts):
    canvas, config, img_path = board_artifacts
    assert canvas is not None
    assert canvas.shape == (2500, 2500)
    assert config["dictionary"] == "DICT_6X6_50"
    assert len(config["markers"]) == 2
    assert config["markers"][0]["id"] == 42
    assert config["markers"][0]["size_m"] == 0.40
    assert config["markers"][1]["id"] == 43
    assert config["markers"][1]["size_m"] == 0.05


def test_dual_scale_detector_initialization():
    detector = DualScaleArUcoDetector(
        big_id=42,
        big_size_m=0.40,
        small_id=43,
        small_size_m=0.05,
        dictionary_name="DICT_6X6_50"
    )
    assert detector.big_id == 42
    assert detector.small_id == 43
    assert detector.big_size_m == 0.40
    assert detector.small_size_m == 0.05


def test_detection_at_transition_altitude(board_artifacts):
    """Test detection when both markers are visible (e.g. at 1.5m)."""
    canvas, _, _ = board_artifacts
    detector = DualScaleArUcoDetector(
        config_path="vision/dual_scale_board_config.yaml"
    )

    fx = fy = 920.0
    cx = 640.0
    cy = 360.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    D = np.zeros(5, dtype=np.float32)

    z_true = 1.50
    proj_w = int(0.50 * fx / z_true)
    resized = cv2.resize(canvas, (proj_w, proj_w), interpolation=cv2.INTER_AREA)

    frame = np.ones((720, 1280), dtype=np.uint8) * 255
    x0 = int(cx - proj_w // 2)
    y0 = int(cy - proj_w // 2)
    frame[y0:y0 + proj_w, x0:x0 + proj_w] = resized

    res = detector.detect(frame, K, D)

    assert res["board_detected"] is True
    assert res["tracking_mode"] == "DUAL_FUSED"
    assert set(res["active_ids"]) == {42, 43}
    assert res["num_corners_fused"] == 8
    assert abs(res["tvec"][2, 0] - z_true) < 0.05  # Within 5cm accuracy


def test_detection_at_low_altitude_inner_marker(board_artifacts):
    """Test detection when outer marker is clipped by camera frame (e.g. at 0.25m)."""
    canvas, _, _ = board_artifacts
    detector = DualScaleArUcoDetector(
        config_path="vision/dual_scale_board_config.yaml"
    )

    fx = fy = 920.0
    cx = 640.0
    cy = 360.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    D = np.zeros(5, dtype=np.float32)

    z_true = 0.25
    proj_w = int(0.50 * fx / z_true)  # ~1840 px (larger than 1280x720 frame!)
    resized = cv2.resize(canvas, (proj_w, proj_w), interpolation=cv2.INTER_AREA)

    # Center crop into 1280x720 (simulating drone right above pad center)
    cw, ch = proj_w // 2, proj_w // 2
    frame = resized[ch - 360:ch + 360, cw - 640:cw + 640]

    res = detector.detect(frame, K, D)

    assert res["board_detected"] is True
    assert res["tracking_mode"] == "INNER_FINE"
    assert 43 in res["active_ids"]
    assert 42 not in res["active_ids"]  # Big marker clipped!
    assert abs(res["tvec"][2, 0] - z_true) < 0.02  # Within 2cm accuracy!


def test_visualizations_hud(board_artifacts):
    canvas, _, _ = board_artifacts
    detector = DualScaleArUcoDetector(
        config_path="vision/dual_scale_board_config.yaml"
    )

    K = np.array([[600.0, 0, 400], [0, 600.0, 400], [0, 0, 1]], dtype=np.float32)
    D = np.zeros(5, dtype=np.float32)

    small_view = cv2.resize(canvas, (800, 800))
    res = detector.detect(small_view, K, D)

    annotated = detector.draw_visualizations(small_view, res, K, D)
    assert annotated.shape[:2] == small_view.shape[:2]
    assert annotated.shape[2] == 3
    assert (annotated[:, :, 0] != small_view).any()
