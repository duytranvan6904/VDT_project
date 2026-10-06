import types

import cv2
import numpy as np
import pytest

from aruco_detector import annotation, aruco_logic, camera_model
from aruco_detector import detection_filter, geometry_utils, image_conversion

MARKER_ID = 42
MARKER_SIZE_M = 0.15
FOCAL_PX = 600.0
IMAGE_W = 640
IMAGE_H = 480
MARKER_PX = 200
DICTIONARY = "DICT_6X6_50"


def make_camera_matrix():
    return np.array(
        [[FOCAL_PX, 0.0, IMAGE_W / 2.0], [0.0, FOCAL_PX, IMAGE_H / 2.0], [0.0, 0.0, 1.0]]
    )


def render_marker(marker_id=MARKER_ID, size_px=MARKER_PX):
    dictionary = aruco_logic.create_dictionary(DICTIONARY)
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    canvas = np.full((IMAGE_H, IMAGE_W), 255, dtype=np.uint8)
    border = 40
    bordered = cv2.copyMakeBorder(marker, border, border, border, border, cv2.BORDER_CONSTANT, value=255)
    total = size_px + 2 * border
    top = (IMAGE_H - total) // 2
    left = (IMAGE_W - total) // 2
    canvas[top:top + total, left:left + total] = bordered
    return canvas


def make_detector(target_id=MARKER_ID):
    return aruco_logic.ArUcoDetector(
        dictionary_name=DICTIONARY,
        marker_size_m=MARKER_SIZE_M,
        camera_matrix=make_camera_matrix(),
        dist_coeffs=camera_model.zero_distortion(),
        target_marker_id=target_id,
    )


def make_msg(data, width, height, step, encoding, is_bigendian=False):
    return types.SimpleNamespace(
        data=bytes(data),
        width=width,
        height=height,
        step=step,
        encoding=encoding,
        is_bigendian=is_bigendian,
    )


class TestGeometryUtils:
    def test_identity_quaternion(self):
        q = geometry_utils.rotation_matrix_to_quaternion_wxyz(np.eye(3))
        assert np.allclose(q, [1.0, 0.0, 0.0, 0.0])

    def test_rotation_z_90(self):
        rvec = np.array([0.0, 0.0, np.pi / 2])
        q = geometry_utils.rvec_to_quaternion_wxyz(rvec)
        expected = [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]
        assert np.allclose(q, expected, atol=1e-9)

    @pytest.mark.parametrize(
        "rvec",
        [
            [0.3, -0.2, 0.5],
            [np.pi, 0.0, 0.0],
            [0.0, np.pi, 0.0],
            [0.0, 0.0, np.pi],
            [2.0, 1.0, -1.5],
        ],
    )
    def test_quaternion_roundtrip_and_norm(self, rvec):
        rvec = np.array(rvec, dtype=np.float64)
        rotation, _ = cv2.Rodrigues(rvec)
        q = geometry_utils.rotation_matrix_to_quaternion_wxyz(rotation)
        assert abs(np.linalg.norm(q) - 1.0) < 1e-6
        w, x, y, z = q
        rebuilt = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
        assert np.allclose(rebuilt, rotation, atol=1e-6)

    def test_corner_bounding_box(self):
        corners = np.array([[10, 20], [50, 22], [48, 80], [12, 78]], dtype=np.float32)
        cx, cy, sx, sy = geometry_utils.compute_corner_bounding_box(corners)
        assert (cx, cy) == (30.0, 50.0)
        assert (sx, sy) == (40.0, 60.0)

    def test_expand_polygon(self):
        if not hasattr(geometry_utils, "expand_polygon"):
            pytest.skip("expand_polygon not applied")
        corners = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float32)
        expanded = geometry_utils.expand_polygon(corners, 0.15)
        assert np.allclose(expanded.mean(axis=0), [5.0, 5.0])
        assert np.allclose(expanded[0], [-0.75, -0.75])
        assert np.allclose(expanded[2], [10.75, 10.75])


class TestCameraModel:
    def test_fallback_matrix(self):
        matrix = camera_model.fallback_camera_matrix(640, 480, np.pi / 2)
        assert matrix[0, 0] == pytest.approx(320.0)
        assert matrix[1, 1] == pytest.approx(320.0)
        assert matrix[0, 2] == 320.0
        assert matrix[1, 2] == 240.0

    def test_zero_distortion_shape(self):
        assert camera_model.zero_distortion().shape == (5, 1)

    def test_camera_info_invalid_k(self):
        info = types.SimpleNamespace(k=[0.0] * 9, d=[])
        assert camera_model.camera_info_to_intrinsics(info) is None

    def test_camera_info_valid(self):
        k = [600.0, 0.0, 320.0, 0.0, 601.0, 240.0, 0.0, 0.0, 1.0]
        info = types.SimpleNamespace(k=k, d=[0.1, 0.0, 0.0, 0.0, 0.0])
        matrix, dist = camera_model.camera_info_to_intrinsics(info)
        assert matrix.shape == (3, 3)
        assert matrix[1, 1] == 601.0
        assert dist.shape == (5, 1)

    def test_camera_info_empty_distortion(self):
        k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
        info = types.SimpleNamespace(k=k, d=[])
        _, dist = camera_model.camera_info_to_intrinsics(info)
        assert np.allclose(dist, 0.0)


class TestImageConversion:
    def test_mono8_with_row_padding(self):
        width, height, step = 4, 3, 6
        raw = np.zeros((height, step), dtype=np.uint8)
        raw[:, :width] = np.arange(width * height, dtype=np.uint8).reshape(height, width)
        raw[:, width:] = 255
        msg = make_msg(raw.tobytes(), width, height, step, "mono8")
        image = image_conversion.ros_image_to_array(msg)
        assert image.shape == (height, width)
        assert image.dtype == np.uint8
        assert image[2, 3] == 11

    def test_bgr8_shape(self):
        data = np.zeros((2, 3, 3), dtype=np.uint8)
        msg = make_msg(data.tobytes(), 3, 2, 9, "bgr8")
        image = image_conversion.ros_image_to_array(msg)
        assert image.shape == (2, 3, 3)

    def test_rgb8_converted_to_bgr(self):
        data = np.zeros((1, 1, 3), dtype=np.uint8)
        data[0, 0] = [255, 0, 0]
        msg = make_msg(data.tobytes(), 1, 1, 3, "rgb8")
        image = image_conversion.ros_image_to_array(msg)
        assert tuple(image[0, 0]) == (0, 0, 255)

    def test_unsupported_encoding(self):
        msg = make_msg(b"\x00" * 16, 4, 4, 4, "32FC1")
        assert image_conversion.ros_image_to_array(msg) is None

    def test_short_buffer(self):
        msg = make_msg(b"\x00" * 5, 4, 4, 4, "mono8")
        assert image_conversion.ros_image_to_array(msg) is None

    def test_zero_step(self):
        msg = make_msg(b"\x00" * 16, 4, 4, 0, "mono8")
        assert image_conversion.ros_image_to_array(msg) is None

    def test_mono16(self):
        if not hasattr(image_conversion, "mono16_to_mono8"):
            pytest.skip("mono16 support not applied")
        pixels = np.array([[0x0000, 0x7F00], [0xFF00, 0x1234]], dtype="<u2")
        msg = make_msg(pixels.tobytes(), 2, 2, 4, "mono16")
        image = image_conversion.ros_image_to_array(msg)
        assert image.dtype == np.uint8
        assert image.tolist() == [[0, 0x7F], [0xFF, 0x12]]

    def test_mono16_bigendian(self):
        if not hasattr(image_conversion, "mono16_to_mono8"):
            pytest.skip("mono16 support not applied")
        pixels = np.array([[0x1200, 0xAB00]], dtype=">u2")
        msg = make_msg(pixels.tobytes(), 2, 1, 4, "16UC1", is_bigendian=True)
        image = image_conversion.ros_image_to_array(msg)
        assert image.tolist() == [[0x12, 0xAB]]


class TestDetectionFilter:
    @staticmethod
    def detection(z, distance):
        return {"pose": {"z": z, "distance": distance}}

    def test_keeps_all_with_zero_limits(self):
        items = [self.detection(1.0, 1.2)]
        assert detection_filter.filter_detections(items, 0.0, 0.0) == items

    def test_filters_by_distance(self):
        items = [self.detection(1.0, 1.2), self.detection(3.0, 3.5)]
        result = detection_filter.filter_detections(items, 2.0, 0.0)
        assert result == [items[1]]

    def test_filters_by_z(self):
        items = [self.detection(0.5, 2.5), self.detection(2.5, 2.6)]
        result = detection_filter.filter_detections(items, 0.0, 1.0)
        assert result == [items[1]]

    def test_boundary_inclusive(self):
        items = [self.detection(1.0, 2.0)]
        assert detection_filter.filter_detections(items, 2.0, 1.0) == items


class TestArucoLogic:
    def test_unknown_dictionary(self):
        with pytest.raises(ValueError):
            aruco_logic.create_dictionary("NOT_A_DICT")

    def test_non_dict_attribute_rejected(self):
        with pytest.raises(ValueError):
            aruco_logic.create_dictionary("CORNER_REFINE_SUBPIX")

    def test_object_points_layout(self):
        points = aruco_logic.build_square_object_points(0.2)
        assert points.shape == (4, 3)
        assert np.allclose(points[0], [-0.1, 0.1, 0.0])
        assert np.allclose(points[2], [0.1, -0.1, 0.0])
        assert np.allclose(points[:, 2], 0.0)

    def test_to_grayscale_passthrough(self):
        gray = np.zeros((4, 4), dtype=np.uint8)
        assert aruco_logic.to_grayscale(gray) is gray

    def test_to_grayscale_from_bgr(self):
        bgr = np.zeros((4, 4, 3), dtype=np.uint8)
        assert aruco_logic.to_grayscale(bgr).ndim == 2

    def test_build_pose_dict(self):
        pose = aruco_logic.build_pose_dict(np.zeros(3), np.array([3.0, 4.0, 12.0]))
        assert pose["distance"] == pytest.approx(13.0)
        assert pose["quaternion"] == pytest.approx([1.0, 0.0, 0.0, 0.0])

    def test_detects_target_and_estimates_depth(self):
        detections = make_detector().process_frame(render_marker())
        assert len(detections) == 1
        detection = detections[0]
        assert detection["id"] == MARKER_ID
        expected_z = FOCAL_PX * MARKER_SIZE_M / MARKER_PX
        pose = detection["pose"]
        assert pose["z"] == pytest.approx(expected_z, rel=0.03)
        assert abs(pose["x"]) < 0.02
        assert abs(pose["y"]) < 0.02
        assert len(pose["quaternion"]) == 4

    def test_frontal_marker_has_identity_like_rotation(self):
        detection = make_detector().process_frame(render_marker())[0]
        rotation, _ = cv2.Rodrigues(detection["rvec"])
        assert rotation[2, 2] == pytest.approx(1.0, abs=0.05) or rotation[2, 2] == pytest.approx(-1.0, abs=0.05)

    def test_ignores_other_id(self):
        image = render_marker(marker_id=7)
        assert make_detector(target_id=MARKER_ID).process_frame(image) == []

    def test_blank_image(self):
        blank = np.full((IMAGE_H, IMAGE_W), 255, dtype=np.uint8)
        assert make_detector().process_frame(blank) == []

    def test_accepts_bgr_input(self):
        bgr = cv2.cvtColor(render_marker(), cv2.COLOR_GRAY2BGR)
        assert len(make_detector().process_frame(bgr)) == 1

    def test_set_camera_parameters_copies(self):
        detector = make_detector()
        matrix = make_camera_matrix()
        detector.set_camera_parameters(matrix, camera_model.zero_distortion())
        matrix[0, 0] = 1.0
        assert detector.camera_matrix[0, 0] == FOCAL_PX

    def test_focal_length_changes_scale(self):
        detector = make_detector()
        image = render_marker()
        base = detector.process_frame(image)[0]["pose"]["z"]
        doubled = make_camera_matrix()
        doubled[0, 0] *= 2
        doubled[1, 1] *= 2
        detector.set_camera_parameters(doubled, camera_model.zero_distortion())
        scaled = detector.process_frame(image)[0]["pose"]["z"]
        assert scaled == pytest.approx(base * 2, rel=0.02)


class TestAnnotation:
    def test_ensure_bgr_from_gray(self):
        gray = np.zeros((10, 10), dtype=np.uint8)
        assert annotation.ensure_bgr(gray).shape == (10, 10, 3)

    def test_ensure_bgr_returns_copy(self):
        bgr = np.zeros((10, 10, 3), dtype=np.uint8)
        assert annotation.ensure_bgr(bgr) is not bgr

    def test_draw_detections_marks_image(self):
        image = render_marker()
        detector = make_detector()
        detections = detector.process_frame(image)
        annotated = annotation.draw_detections(
            image, detections, detector.camera_matrix, detector.dist_coeffs, MARKER_SIZE_M
        )
        assert annotated.shape == (IMAGE_H, IMAGE_W, 3)
        green = (annotated[:, :, 1] == 255) & (annotated[:, :, 0] == 0) & (annotated[:, :, 2] == 0)
        assert green.any()

    def test_draw_detections_empty(self):
        image = render_marker()
        annotated = annotation.draw_detections(image, [], make_camera_matrix(), camera_model.zero_distortion(), 0.15)
        assert annotated.shape == (IMAGE_H, IMAGE_W, 3)


class TestMessageBuilders:
    @pytest.fixture
    def builders(self):
        pytest.importorskip("geometry_msgs.msg")
        pytest.importorskip("sensor_msgs.msg")
        pytest.importorskip("vision_msgs.msg")
        from aruco_detector import message_builders
        return message_builders

    @staticmethod
    def sample_detection():
        corners = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.float32)
        return {
            "id": MARKER_ID,
            "corners": corners,
            "pose": {"x": 0.1, "y": 0.2, "z": 1.5, "distance": 1.52, "quaternion": [0.5, 0.1, 0.2, 0.3]},
        }

    def test_pose_quaternion_order(self, builders):
        stamp = types.SimpleNamespace(sec=1, nanosec=2)
        message = builders.build_pose_message(self.sample_detection(), stamp, "cam")
        assert message.pose.orientation.w == 0.5
        assert message.pose.orientation.x == 0.1
        assert message.pose.orientation.z == 0.3
        assert message.header.frame_id == "cam"

    def test_bbox(self, builders):
        message = builders.build_bbox_message(self.sample_detection())
        assert message.center.position.x == 150.0
        assert message.size_x == 100.0

    def test_resolve_stamp(self, builders):
        empty = types.SimpleNamespace(sec=0, nanosec=0)
        fallback = types.SimpleNamespace(sec=5, nanosec=0)
        real = types.SimpleNamespace(sec=3, nanosec=0)
        assert builders.resolve_stamp(empty, fallback) is fallback
        assert builders.resolve_stamp(real, fallback) is real

    def test_polygon_message(self, builders):
        if not hasattr(builders, "build_polygon_message"):
            pytest.skip("polygon message not applied")
        stamp = types.SimpleNamespace(sec=1, nanosec=0)
        message = builders.build_polygon_message(self.sample_detection(), stamp, "cam", 0.15)
        assert len(message.polygon.points) == 4