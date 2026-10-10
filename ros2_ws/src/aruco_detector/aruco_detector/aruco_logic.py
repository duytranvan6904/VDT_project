from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from .geometry_utils import rvec_to_quaternion_wxyz

Detection = Dict[str, Any]


def create_dictionary(dictionary_name: str):
    dictionary_id = getattr(cv2.aruco, dictionary_name, None)
    if dictionary_id is None or not dictionary_name.startswith("DICT_"):
        raise ValueError(f"unknown ArUco dictionary: {dictionary_name}")
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dictionary_id)
    return cv2.aruco.Dictionary_get(dictionary_id)


def create_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    return cv2.aruco.DetectorParameters()


def tune_detector_parameters(parameters):
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 53
    parameters.adaptiveThreshWinSizeStep = 4
    parameters.minMarkerPerimeterRate = 0.02
    parameters.maxMarkerPerimeterRate = 4.0
    parameters.minDistanceToBorder = 3
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.polygonalApproxAccuracyRate = 0.05
    parameters.minCornerDistanceRate = 0.02
    return parameters


def to_grayscale(image: np.ndarray) -> np.ndarray:
    if image.ndim == 3:
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return image


def build_square_object_points(side_m: float) -> np.ndarray:
    half = side_m / 2.0
    return np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float32,
    )


def build_pose_dict(rvec: np.ndarray, tvec: np.ndarray) -> Dict[str, Any]:
    x, y, z = (float(value) for value in tvec.flatten())
    return {
        "x": x,
        "y": y,
        "z": z,
        "distance": float(np.sqrt(x * x + y * y + z * z)),
        "quaternion": rvec_to_quaternion_wxyz(rvec).tolist(),
    }


class ArUcoDetector:
    def __init__(
        self,
        dictionary_name: str,
        marker_size_m: float,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        target_marker_id: int,
        marker_offset_m: Tuple[float, float] = (0.0, 0.0),
    ) -> None:
        self.marker_size_m = float(marker_size_m)
        self.marker_offset_m = (float(marker_offset_m[0]), float(marker_offset_m[1]))
        self.target_marker_id = int(target_marker_id)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.dictionary = create_dictionary(dictionary_name)
        self.parameters = tune_detector_parameters(create_detector_parameters())
        self.detector_object = self.create_detector_object()

    def create_detector_object(self):
        if hasattr(cv2.aruco, "ArucoDetector"):
            return cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        return None

    def set_camera_parameters(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> None:
        self.camera_matrix = camera_matrix.copy()
        self.dist_coeffs = dist_coeffs.copy()

    def detect_markers(self, gray: np.ndarray) -> Tuple[List[np.ndarray], Optional[np.ndarray]]:
        if self.detector_object is not None:
            corners, ids, _ = self.detector_object.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        return corners, ids

    def select_target_corners(
        self, corners: List[np.ndarray], ids: Optional[np.ndarray]
    ) -> List[np.ndarray]:
        if ids is None:
            return []
        return [
            corner.reshape((4, 2))
            for corner, marker_id in zip(corners, ids.flatten())
            if int(marker_id) == self.target_marker_id
        ]

    def solve_marker_pose(self, corners: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        success, rvec, tvec = cv2.solvePnP(
            build_square_object_points(self.marker_size_m),
            corners.astype(np.float32),
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            return None
        return rvec, tvec

    def apply_marker_offset(self, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
        ox, oy = self.marker_offset_m
        if ox == 0.0 and oy == 0.0:
            return tvec
        rotation, _ = cv2.Rodrigues(rvec)
        shift = rotation @ np.array([[ox], [oy], [0.0]])
        return tvec.reshape(3, 1) + shift

    def build_detection(self, corners: np.ndarray) -> Optional[Detection]:
        solved = self.solve_marker_pose(corners)
        if solved is None:
            return None
        rvec, tvec = solved
        tvec = self.apply_marker_offset(rvec, tvec)
        return {
            "id": self.target_marker_id,
            "corners": corners,
            "rvec": rvec,
            "tvec": tvec,
            "pose": build_pose_dict(rvec, tvec),
        }

    def process_frame(self, image: np.ndarray) -> List[Detection]:
        corners, ids = self.detect_markers(to_grayscale(image))
        candidates = self.select_target_corners(corners, ids)
        detections = [self.build_detection(candidate) for candidate in candidates]
        return [detection for detection in detections if detection is not None]
