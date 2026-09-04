import cv2
import numpy as np
from typing import List, Tuple, Dict, Any, Optional, Union

from .utils import rvec_to_euler, rvec_to_quaternion, extract_bounding_box, draw_axis_3d


# Mapping dictionary names to OpenCV ArUco constants
ARUCO_DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

# Color palette for distinct marker drawing
COLOR_PALETTE = [
    (0, 255, 0),    # Green
    (255, 165, 0),  # Orange
    (255, 0, 255),  # Magenta
    (0, 255, 255),  # Cyan
    (255, 255, 0),  # Yellow
    (0, 165, 255),  # Orange-Red
    (147, 112, 219),# Purple
    (0, 215, 255),  # Gold
]


class ArUcoDetector:
    """
    ArUco Marker Detector and PnP Pose Estimator for RealSense D435 Vision Pipeline.
    Supports multi-size markers per ID and clean visualization panels.
    """
    def __init__(
        self,
        dictionary_name: str = "DICT_4X4_50",
        marker_size_meters: Union[float, Dict[int, float]] = 0.15,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None
    ):
        """
        Initialize ArUco Detector.
        
        Args:
            dictionary_name: Name of ArUco dictionary (e.g., 'DICT_4X4_50')
            marker_size_meters: Default float side length in meters, or dict mapping {marker_id: size_meters}
            camera_matrix: 3x3 Intrinsic matrix K
            dist_coeffs: 1x5 or 1x8 Distortion coefficients D
        """
        self.dictionary_name = dictionary_name
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

        if isinstance(marker_size_meters, dict):
            self.marker_sizes = marker_size_meters
            self.default_marker_size = float(marker_size_meters.get(-1, 0.15))
        else:
            self.marker_sizes = {}
            self.default_marker_size = float(marker_size_meters)

        if dictionary_name not in ARUCO_DICT_MAP:
            raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}. Choose from {list(ARUCO_DICT_MAP.keys())}")

        self.dict_id = ARUCO_DICT_MAP[dictionary_name]
        
        # Cross-version OpenCV ArUco initialization
        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.dict_id)
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(self.dict_id)

        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.parameters = cv2.aruco.DetectorParameters_create()
        elif hasattr(cv2.aruco, "DetectorParameters"):
            self.parameters = cv2.aruco.DetectorParameters()
        else:
            self.parameters = None

        # Tune DetectorParameters for multi-marker, small marker, and tight boundary robustness
        self._optimize_detector_parameters()

        # OpenCV 4.7+ ArucoDetector support
        self.use_aruco_detector_obj = hasattr(cv2.aruco, "ArucoDetector")
        if self.use_aruco_detector_obj:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def _optimize_detector_parameters(self):
        """Configure OpenCV DetectorParameters for multi-scale and high-sensitivity detection."""
        if self.parameters is None:
            return

        # Adaptive Thresholding tuning: finer step to capture both small and large markers simultaneously
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 53
        self.parameters.adaptiveThreshWinSizeStep = 4

        # Allow smaller markers (down to 1% of image perimeter)
        self.parameters.minMarkerPerimeterRate = 0.01
        self.parameters.maxMarkerPerimeterRate = 4.0

        # Allow markers near or touching image borders
        self.parameters.minDistanceToBorder = 0

        # Corner subpixel refinement for accurate PnP pose estimation
        if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
            self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        elif hasattr(cv2.aruco, "CORNER_REFINE_CONTOUR"):
            self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

        # Polygonal approximation tolerances
        self.parameters.polygonalApproxAccuracyRate = 0.05
        self.parameters.minCornerDistanceRate = 0.02

    def get_marker_size(self, marker_id: int) -> float:
        """Get physical side length in meters for a specific marker ID."""
        return float(self.marker_sizes.get(marker_id, self.default_marker_size))

    def _get_3d_obj_points(self, size_meters: float) -> np.ndarray:
        """Generate 3D corner coordinates for a marker of given size."""
        half_s = size_meters / 2.0
        return np.array([
            [-half_s,  half_s, 0.0],
            [ half_s,  half_s, 0.0],
            [ half_s, -half_s, 0.0],
            [-half_s, -half_s, 0.0]
        ], dtype=np.float32)

    def set_camera_parameters(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        """Update intrinsic camera parameters."""
        self.camera_matrix = camera_matrix.copy()
        self.dist_coeffs = dist_coeffs.copy()

    def detect(self, image: np.ndarray) -> Tuple[List[np.ndarray], Optional[np.ndarray], List[np.ndarray]]:
        """Detect ArUco markers in the image."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        
        if self.use_aruco_detector_obj:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.parameters
            )
            
        return corners, ids, rejected

    def estimate_pose_pnp(
        self,
        corners: np.ndarray,
        marker_id: int = -1,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None
    ) -> Tuple[bool, np.ndarray, np.ndarray, Dict[str, Any]]:
        """
        Estimate 6DOF Pose (Rvec, Tvec) for a single marker using its specific physical size.
        """
        K = camera_matrix if camera_matrix is not None else self.camera_matrix
        D = dist_coeffs if dist_coeffs is not None else self.dist_coeffs

        if K is None or D is None:
            raise ValueError("Camera matrix K and distortion coefficients D must be set before PnP Pose Estimation.")

        size_m = self.get_marker_size(marker_id)
        obj_points = self._get_3d_obj_points(size_m)
        image_points_2d = corners.reshape((4, 2)).astype(np.float32)

        solve_flag = cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.SOLVEPNP_ITERATIVE

        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            image_points_2d,
            K,
            D,
            flags=solve_flag
        )

        if not success:
            return False, np.zeros((3, 1)), np.zeros((3, 1)), {}

        tx, ty, tz = tvec.flatten()
        dist_3d = np.sqrt(tx**2 + ty**2 + tz**2)
        
        roll_deg, pitch_deg, yaw_deg = rvec_to_euler(rvec, degrees=True)
        quat_wxyz = rvec_to_quaternion(rvec)

        pose_info = {
            "marker_size_m": size_m,
            "x": float(tx),
            "y": float(ty),
            "z": float(tz),
            "distance": float(dist_3d),
            "roll": float(roll_deg),
            "pitch": float(pitch_deg),
            "yaw": float(yaw_deg),
            "quaternion": quat_wxyz.tolist()
        }

        return True, rvec, tvec, pose_info

    def process_frame(
        self,
        image: np.ndarray,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None
    ) -> List[Dict[str, Any]]:
        """Detect markers and calculate PnP pose for each."""
        corners, ids, _ = self.detect(image)
        results = []

        if ids is None or len(ids) == 0:
            return results

        K = camera_matrix if camera_matrix is not None else self.camera_matrix
        D = dist_coeffs if dist_coeffs is not None else self.dist_coeffs

        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i]
            mid = int(marker_id)
            bbox = extract_bounding_box(c, margin_percent=0.15, img_shape=image.shape)
            
            item = {
                "id": mid,
                "corners": c.reshape((4, 2)),
                "bbox": bbox,
                "rvec": None,
                "tvec": None,
                "pose": None
            }

            if K is not None and D is not None:
                success, rvec, tvec, pose_info = self.estimate_pose_pnp(c, mid, K, D)
                if success:
                    item["rvec"] = rvec
                    item["tvec"] = tvec
                    item["pose"] = pose_info

            results.append(item)

        results.sort(key=lambda x: x["id"])
        return results

    def draw_results(
        self,
        image: np.ndarray,
        detection_results: List[Dict[str, Any]],
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
        draw_axes: bool = True,
        draw_bbox_mask: bool = True,
        draw_summary_table: bool = True
    ) -> np.ndarray:
        """
        Annotate image cleanly without text overlapping.
        """
        annotated = image.copy()
        K = camera_matrix if camera_matrix is not None else self.camera_matrix
        D = dist_coeffs if dist_coeffs is not None else self.dist_coeffs

        for idx, res in enumerate(detection_results):
            marker_id = res["id"]
            pts = res["corners"].astype(int)
            color = COLOR_PALETTE[marker_id % len(COLOR_PALETTE)]

            # 1. Draw 2D polygon outline
            cv2.polylines(annotated, [pts], isClosed=True, color=color, thickness=2)

            # 2. Draw Top-Left Corner dot (Red)
            cv2.circle(annotated, tuple(pts[0]), 5, (0, 0, 255), -1)

            # 3. Draw Compact ID Label Box right above marker
            top_left_x, top_left_y = pts[0]
            label_str = f"ID:{marker_id}"
            (w_txt, h_txt), _ = cv2.getTextSize(label_str, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(
                annotated,
                (top_left_x, max(0, top_left_y - h_txt - 8)),
                (top_left_x + w_txt + 8, top_left_y),
                color,
                -1
            )
            cv2.putText(
                annotated,
                label_str,
                (top_left_x + 4, max(h_txt + 2, top_left_y - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                2
            )

            # 4. Draw Masking Bounding Box
            if draw_bbox_mask and "bbox" in res:
                bx, by, bw, bh = res["bbox"]
                cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (255, 0, 255), 1)

            # 5. Draw 3D Axes at marker origin
            if draw_axes and K is not None and D is not None and res["rvec"] is not None:
                size_m = self.get_marker_size(marker_id)
                draw_axis_3d(
                    annotated,
                    K,
                    D,
                    res["rvec"],
                    res["tvec"],
                    length=size_m * 0.8,
                    thickness=3
                )

        # 6. Draw Clean Summary Panel at Top-Right/Overlay
        if draw_summary_table and len(detection_results) > 0:
            self._draw_overlay_summary(annotated, detection_results)

        return annotated

    def _draw_overlay_summary(self, img: np.ndarray, results: List[Dict[str, Any]]):
        """Draw a semi-transparent HUD summary panel listing pose data for each detected marker."""
        h, w = img.shape[:2]
        panel_w = 400
        panel_h = min(h - 20, 40 + len(results) * 25)
        
        overlay = img.copy()
        cv2.rectangle(overlay, (w - panel_w - 10, 10), (w - 10, panel_h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.7, img, 0.3, 0, img)
        cv2.rectangle(img, (w - panel_w - 10, 10), (w - 10, panel_h), (255, 255, 255), 1)

        # Table Header
        header = "ID | Size | X(m)  | Y(m)  | Z(m)  | Dist"
        cv2.putText(img, header, (w - panel_w, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # Table Rows
        for idx, res in enumerate(results):
            mid = res["id"]
            color = COLOR_PALETTE[mid % len(COLOR_PALETTE)]
            y_pos = 55 + idx * 25

            if res["pose"] is not None:
                p = res["pose"]
                row_str = f"{mid:2d} | {p['marker_size_m']:.2f} | {p['x']:+.2f} | {p['y']:+.2f} | {p['z']:.2f} | {p['distance']:.2f}m"
            else:
                row_str = f"{mid:2d} | No PnP Pose"

            cv2.circle(img, (w - panel_w - 4, y_pos - 4), 5, color, -1)
            cv2.putText(img, row_str, (w - panel_w + 8, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
