import os
from pathlib import Path
import xml.etree.ElementTree as ET
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
        big_size_m=0.52,
        small_id=43,
        small_size_m=0.10,
        small_patch_m=0.11,
        margin_m=0.012,
        dict_name="DICT_6X6_50",
        pixels_per_cm=25,
        paper_width_m=1.189,
        paper_height_m=0.841,
    )
    img_path = str(tmp_path / "test_board.png")
    cv2.imwrite(img_path, canvas)
    return canvas, config, img_path


def test_dual_scale_board_generation(board_artifacts):
    canvas, config, img_path = board_artifacts
    assert canvas is not None
    assert canvas.shape == (2102, 2972)
    assert config["dictionary"] == "DICT_6X6_50"
    assert len(config["markers"]) == 2
    assert config["markers"][0]["id"] == 42
    assert config["markers"][0]["size_m"] == 0.52
    assert config["markers"][1]["id"] == 43
    assert config["markers"][1]["size_m"] == 0.10


def test_dual_scale_detector_initialization():
    detector = DualScaleArUcoDetector(
        big_id=42,
        big_size_m=0.52,
        small_id=43,
        small_size_m=0.10,
        dictionary_name="DICT_6X6_50"
    )
    assert detector.big_id == 42
    assert detector.small_id == 43
    assert detector.big_size_m == 0.52
    assert detector.small_size_m == 0.10


def test_detection_at_transition_altitude(board_artifacts):
    """Test detection when both markers are visible (e.g. at 1.5m)."""
    canvas, config, _ = board_artifacts
    detector = DualScaleArUcoDetector(
        config_path="vision/dual_scale_board_config.yaml"
    )

    fx = fy = 920.0
    cx = 640.0
    cy = 360.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    D = np.zeros(5, dtype=np.float32)

    z_true = 1.50
    proj_w = int(config["total_pad_width_m"] * fx / z_true)
    proj_h = int(config["total_pad_height_m"] * fy / z_true)
    resized = cv2.resize(canvas, (proj_w, proj_h), interpolation=cv2.INTER_AREA)

    frame = np.ones((720, 1280), dtype=np.uint8) * 255
    x0 = int(cx - proj_w // 2)
    y0 = int(cy - proj_h // 2)
    frame[y0:y0 + proj_h, x0:x0 + proj_w] = resized

    res = detector.detect(frame, K, D)

    assert res["board_detected"] is True
    assert res["tracking_mode"] == "OUTER_COARSE"
    assert 42 in res["active_ids"]
    assert res["num_corners_fused"] >= 4
    assert abs(res["tvec"][2, 0] - z_true) < 0.08  # Accurate 3D distance


def test_detection_at_low_altitude_inner_marker(board_artifacts):
    """Test detection when outer marker is clipped by camera frame (e.g. at 0.25m)."""
    canvas, config, _ = board_artifacts
    detector = DualScaleArUcoDetector(
        config_path="vision/dual_scale_board_config.yaml"
    )

    fx = fy = 920.0
    cx = 640.0
    cy = 360.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    D = np.zeros(5, dtype=np.float32)

    z_true = 0.25
    proj_w = int(config["total_pad_width_m"] * fx / z_true)
    proj_h = int(config["total_pad_height_m"] * fy / z_true)
    resized = cv2.resize(canvas, (proj_w, proj_h), interpolation=cv2.INTER_AREA)

    # Center crop into 1280x720 (simulating drone right above pad center)
    cw, ch = proj_w // 2, proj_h // 2
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

    K = np.array([[600.0, 0, 566], [0, 600.0, 400], [0, 0, 1]], dtype=np.float32)
    D = np.zeros(5, dtype=np.float32)

    small_view = cv2.resize(canvas, (1132, 800))
    res = detector.detect(small_view, K, D)

    annotated = detector.draw_visualizations(small_view, res, K, D)
    assert annotated.shape[:2] == small_view.shape[:2]
    assert annotated.shape[2] == 3
    assert (annotated[:, :, 0] != small_view).any()


def test_detection_at_outer_coarse_range(board_artifacts):
    """The A0 coarse tag must remain readable around the current 4.65m range."""
    canvas, config, _ = board_artifacts
    detector = DualScaleArUcoDetector(config_path="vision/dual_scale_board_config.yaml")

    fx = fy = 466.0
    cx, cy = 320.0, 240.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    D = np.zeros(5, dtype=np.float32)
    z_true = 4.65
    proj_w = int(config["total_pad_width_m"] * fx / z_true)
    proj_h = int(config["total_pad_height_m"] * fy / z_true)
    resized = cv2.resize(canvas, (proj_w, proj_h), interpolation=cv2.INTER_AREA)
    frame = np.ones((480, 640), dtype=np.uint8) * 255
    x0 = int(cx - proj_w // 2)
    y0 = int(cy - proj_h // 2)
    frame[y0:y0 + proj_h, x0:x0 + proj_w] = resized

    result = detector.detect(frame, K, D)

    assert result["board_detected"] is True
    assert result["tracking_mode"] == "OUTER_COARSE"
    assert 42 in result["active_ids"]
    assert abs(result["tvec"][2, 0] - z_true) < 0.50


def test_a0_map_model_matches_board_texture_and_geometry():
    model_dir = (
        Path(__file__).resolve().parents[1]
        / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "models" / "arucotag"
    )
    model_sdf = model_dir / "model.sdf"
    model_dae = model_dir / "hpad_aruco.dae"
    if not model_sdf.exists() or not model_dae.exists():
        pytest.skip("PX4 Gazebo model is not present in this checkout")

    sdf_root = ET.parse(model_sdf).getroot()
    expected_size = (1.189, 0.841)
    for plane_size in sdf_root.findall(".//plane/size"):
        width, height = (float(v) for v in plane_size.text.split())
        assert width == pytest.approx(expected_size[0], abs=1e-6)
        assert height == pytest.approx(expected_size[1], abs=1e-6)

    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    dae_root = ET.parse(model_dae).getroot()
    mesh_positions = dae_root.find(
        ".//c:float_array[@id='hpad_positions_array']", ns,
    )
    values = [float(v) for v in mesh_positions.text.split()]
    xs, ys = values[0::3], values[1::3]
    assert max(xs) - min(xs) == pytest.approx(expected_size[0], abs=1e-6)
    assert max(ys) - min(ys) == pytest.approx(expected_size[1], abs=1e-6)

    texture_path = model_dir / "dual_scale_aruco_board_A0_52_5cm.png"
    texture = cv2.imread(str(texture_path), cv2.IMREAD_GRAYSCALE)
    assert texture is not None
    assert texture.shape == (2102, 2972)
