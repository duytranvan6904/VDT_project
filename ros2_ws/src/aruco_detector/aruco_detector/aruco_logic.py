from typing import Any, Dict, List, Optional, Tuple, Union

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


def build_pose_dict(rvec: np.ndarray, tvec: np.ndarray) -> Dict[str, Any]:
    x, y, z = (float(value) for value in tvec.flatten())
    return {
        "x": x,
        "y": y,
        "z": z,
        "distance": float(np.sqrt(x * x + y * y + z * z)),
        "quaternion": rvec_to_quaternion_wxyz(rvec).tolist(),
    }


class DualScaleBoardDetector:
    """
    Dual-Scale ArUco Board Detector for Precision Landing.
    Combines:
      - Outer coarse marker (ID 42, 52cm): Far range guidance (0.8m - 5.0m)
      - Inner fine marker (ID 43, 10cm): Close range touchdown guidance (0.05m - 1.2m)
    Both markers' 3D corners are defined relative to the geometric center of the pad (0,0,0).
    """

    def __init__(
        self,
        dictionary_name: str = "DICT_6X6_50",
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
        board_config: Optional[Dict[str, Any]] = None,
    ) -> None:
        self.dictionary_name = dictionary_name
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros((1, 5), dtype=np.float32)
        self.dictionary = create_dictionary(dictionary_name)
        self.parameters = tune_detector_parameters(create_detector_parameters())
        self.detector_object = self.create_detector_object()

        # Parse Board Configuration
        if board_config is None:
            # Default A0 dual scale configuration (ID 42 + ID 43)
            self.big_id = 42
            self.big_size_m = 0.52
            self.big_corners_3d = np.array([
                [-0.5825,  0.26, 0.0],
                [-0.0625,  0.26, 0.0],
                [-0.0625, -0.26, 0.0],
                [-0.5825, -0.26, 0.0]
            ], dtype=np.float32)

            self.small_id = 43
            self.small_size_m = 0.10
            self.small_corners_3d = np.array([
                [-0.05,  0.05, 0.0],
                [ 0.05,  0.05, 0.0],
                [ 0.05, -0.05, 0.0],
                [-0.05, -0.05, 0.0]
            ], dtype=np.float32)
        else:
            markers = board_config.get("markers", [])
            self.big_id = int(markers[0]["id"])
            self.big_size_m = float(markers[0]["size_m"])
            self.big_corners_3d = np.array(markers[0]["corners_3d"], dtype=np.float32)

            self.small_id = int(markers[1]["id"])
            self.small_size_m = float(markers[1]["size_m"])
            self.small_corners_3d = np.array(markers[1]["corners_3d"], dtype=np.float32)

        self.marker_corners_3d = {
            self.big_id: self.big_corners_3d,
            self.small_id: self.small_corners_3d,
        }
        self.marker_centers_3d = {
            self.big_id: np.mean(self.big_corners_3d, axis=0).reshape(3, 1),
            self.small_id: np.mean(self.small_corners_3d, axis=0).reshape(3, 1),
        }
        self.marker_sizes = {
            self.big_id: self.big_size_m,
            self.small_id: self.small_size_m,
        }

        # Create OpenCV Board
        obj_points = [self.big_corners_3d, self.small_corners_3d]
        board_ids = np.array([self.big_id, self.small_id], dtype=np.int32)
        if hasattr(cv2.aruco, "Board_create"):
            self.board = cv2.aruco.Board_create(obj_points, self.dictionary, board_ids)
        elif hasattr(cv2.aruco, "Board"):
            self.board = cv2.aruco.Board(obj_points, self.dictionary, board_ids)
        else:
            self.board = None

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

    def process_frame(self, image: np.ndarray) -> List[Detection]:
        if self.camera_matrix is None:
            return []

        gray = to_grayscale(image)
        corners, ids = self.detect_markers(gray)
        if ids is None or len(ids) == 0:
            return []

        detected_map = {}
        for c, mid in zip(corners, ids.flatten()):
            mid_int = int(mid)
            if mid_int in (self.big_id, self.small_id):
                detected_map[mid_int] = c.reshape((4, 2))

        if not detected_map:
            return []

        # 1. Prepare corners and IDs for Board PnP
        board_corners = []
        board_ids = []
        for mid in (self.big_id, self.small_id):
            if mid in detected_map:
                board_corners.append(detected_map[mid].astype(np.float32))
                board_ids.append(mid)

        board_ids_arr = np.array(board_ids, dtype=np.int32)
        retval, rvec, tvec = 0, None, None

        # 2. Solve Board PnP with OpenCV estimatePoseBoard
        if self.board is not None:
            try:
                retval, rvec, tvec = cv2.aruco.estimatePoseBoard(
                    board_corners,
                    board_ids_arr,
                    self.board,
                    self.camera_matrix,
                    self.dist_coeffs,
                    None,
                    None,
                )
            except Exception:
                retval = 0

        # 3. Fallback: solve individual marker PnP and map back to pad center (0,0,0)
        if (retval == 0 or rvec is None or tvec is None) and detected_map:
            # Prefer inner fine marker if available at close range (< 1.2m), otherwise big marker
            pref_id = self.small_id if self.small_id in detected_map else self.big_id
            c = detected_map[pref_id]
            size_m = self.marker_sizes[pref_id]
            half = size_m / 2.0
            obj_p = np.array([[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]], dtype=np.float32)

            success, rvec_m, tvec_m = cv2.solvePnP(
                obj_p, c.astype(np.float32), self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if success:
                rvec = rvec_m
                rot, _ = cv2.Rodrigues(rvec)
                # Shift tvec from marker center to geometric center of the pad
                center_offset = self.marker_centers_3d[pref_id]
                tvec = tvec_m.reshape(3, 1) - rot @ center_offset
                retval = 1

        if retval > 0 and rvec is not None and tvec is not None:
            # Active marker representation: prefer small marker's bounding box/corners if present
            active_id = self.small_id if self.small_id in detected_map else self.big_id
            rep_corners = detected_map[active_id]

            mode = "DUAL_FUSED" if (len(detected_map) == 2) else ("INNER_FINE" if self.small_id in detected_map else "OUTER_COARSE")

            return [{
                "id": active_id,
                "active_ids": list(detected_map.keys()),
                "tracking_mode": mode,
                "corners": rep_corners,
                "all_corners": detected_map,
                "rvec": rvec.reshape(3, 1),
                "tvec": tvec.reshape(3, 1),
                "pose": build_pose_dict(rvec, tvec),
            }]

        return []
