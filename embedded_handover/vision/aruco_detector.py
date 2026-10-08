import os
import cv2
import numpy as np
from typing import List, Tuple, Dict, Any, Optional, Union, Iterable

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
        # The repository's aruco_marker.png uses 6x6 codes. Keeping the
        # dictionary aligned with the physical/displayed target is critical:
        # a wrong dictionary can decode IR/projector noise as false IDs.
        dictionary_name: str = "DICT_6X6_50",
        marker_size_meters: Union[float, Dict[int, float]] = 0.15,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
        target_marker_ids: Optional[Union[int, Iterable[int]]] = 42
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
        if target_marker_ids is None:
            self.target_marker_ids = None
        elif isinstance(target_marker_ids, (int, np.integer)):
            self.target_marker_ids = {int(target_marker_ids)}
        else:
            self.target_marker_ids = {int(marker_id) for marker_id in target_marker_ids}

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
        self.parameters.adaptiveThreshWinSizeMax = 35
        self.parameters.adaptiveThreshWinSizeStep = 8

        # Allow smaller markers (down to 1% of image perimeter)
        # IR projector speckles can form tiny quadrilaterals. A slightly
        # stricter lower bound avoids decoding those as false markers while
        # retaining normal screen/landing-pad marker sizes.
        self.parameters.minMarkerPerimeterRate = 0.02
        self.parameters.maxMarkerPerimeterRate = 4.0

        # Require at least 3px margin from image border to avoid boundary-clipping distortions
        self.parameters.minDistanceToBorder = 3

        # Corner subpixel refinement for accurate PnP pose estimation
        if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
            self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        elif hasattr(cv2.aruco, "CORNER_REFINE_CONTOUR"):
            self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

        # Polygonal approximation tolerances
        self.parameters.polygonalApproxAccuracyRate = 0.05
        self.parameters.minCornerDistanceRate = 0.02

        # The A0 board uses two separated tags, so keep OpenCV's normal
        # duplicate-contour spacing instead of treating one nested tag as two.
        self.parameters.minMarkerDistanceRate = 0.01

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
        dist_coeffs: Optional[np.ndarray] = None,
        depth_frame: Optional[Any] = None
    ) -> List[Dict[str, Any]]:
        """Detect markers, calculate PnP pose, and optionally attach depth.

        ``depth_frame`` must be aligned to ``image``. This is true for the
        RealSenseCamera IR fallback because it aligns depth to IR1. Depth is
        sampled from a small patch around each marker center and stored as
        ``z_depth``/``depth_m`` in meters; invalid depth never discards a
        valid ArUco/PnP measurement.
        """
        corners, ids, _ = self.detect(image)
        results = []

        if ids is None or len(ids) == 0:
            return results

        # marker42.png is ID 42. Filtering at this boundary prevents IR
        # projector texture or screen moire from becoming a target track.
        if self.target_marker_ids is not None:
            keep = [int(marker_id) in self.target_marker_ids for marker_id in ids.flatten()]
            corners = [corner for corner, selected in zip(corners, keep) if selected]
            ids = ids[np.asarray(keep, dtype=bool)].reshape((-1, 1))
            if ids.size == 0:
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
                "pose": None,
                "z_depth": None,
                "depth_m": None,
                "depth_valid": False,
                "depth_samples": 0
            }

            if depth_frame is not None:
                depth_m, sample_count = self._sample_depth_at_marker(
                    depth_frame, item["corners"], image.shape[:2]
                )
                item["z_depth"] = depth_m
                item["depth_m"] = depth_m
                item["depth_valid"] = depth_m is not None
                item["depth_samples"] = sample_count

            if K is not None and D is not None:
                success, rvec, tvec, pose_info = self.estimate_pose_pnp(c, mid, K, D)
                if success:
                    item["rvec"] = rvec
                    item["tvec"] = tvec
                    item["pose"] = pose_info

            results.append(item)

        results.sort(key=lambda x: x["id"])
        return results

    @staticmethod
    def _sample_depth_at_marker(
        depth_frame: Any,
        corners: np.ndarray,
        image_shape: Tuple[int, int],
        radius: int = 3
    ) -> Tuple[Optional[float], int]:
        """Return robust median depth at a marker center in meters."""
        h, w = image_shape
        center = np.mean(corners, axis=0)
        cx = int(round(float(center[0])))
        cy = int(round(float(center[1])))

        if hasattr(depth_frame, "get_distance"):
            values = []
            for y in range(max(0, cy - radius), min(h, cy + radius + 1)):
                for x in range(max(0, cx - radius), min(w, cx + radius + 1)):
                    try:
                        value = float(depth_frame.get_distance(x, y))
                    except Exception:
                        value = 0.0
                    if np.isfinite(value) and value > 0:
                        values.append(value)
        elif isinstance(depth_frame, np.ndarray):
            patch = depth_frame[
                max(0, cy - radius):min(h, cy + radius + 1),
                max(0, cx - radius):min(w, cx + radius + 1)
            ]
            values = patch[np.isfinite(patch) & (patch > 0)].astype(float).tolist()
            # RealSense raw depth arrays are normally uint16 in millimeters;
            # floating-point arrays are assumed to already be meters.
            if np.issubdtype(depth_frame.dtype, np.integer):
                values = [v * 0.001 for v in values]
        else:
            values = []

        if not values:
            return None, 0
        return float(np.median(values)), len(values)

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


class DualScaleArUcoDetector:
    """
    Precision Landing Detector utilizing a Dual-Scale ArUco Board.
    Combines:
      - Big Coarse Marker (e.g., ID 42, 52cm): Far range guidance (0.8m - 5.0m)
      - Small Inner Marker (e.g., ID 43, 5cm): Close range touchdown guidance (0.05m - 1.2m)
    
    The small marker defines the landing origin. The large marker is placed away
    from it on an A0 sheet. Each marker's configured board corners map detections
    back to that same origin; the larger visible tag is preferred for pose.
    """

    def __init__(
        self,
        config_path: Optional[str] = None,
        big_id: int = 42,
        big_size_m: float = 0.52,
        small_id: int = 43,
        small_size_m: float = 0.05,
        dictionary_name: str = "DICT_6X6_50",
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
    ):
        """
        Initialize Dual-Scale ArUco Board Detector.
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros(5, dtype=np.float32)
        
        # Load from config file if provided and exists
        if config_path and os.path.exists(config_path):
            import yaml
            with open(config_path, "r") as f:
                cfg = yaml.safe_load(f)
            self.dictionary_name = cfg.get("dictionary", dictionary_name)
            markers_cfg = cfg.get("markers", [])
            self.big_id = markers_cfg[0]["id"]
            self.big_size_m = float(markers_cfg[0]["size_m"])
            self.big_corners_3d = np.array(markers_cfg[0]["corners_3d"], dtype=np.float32)
            
            self.small_id = markers_cfg[1]["id"]
            self.small_size_m = float(markers_cfg[1]["size_m"])
            self.small_corners_3d = np.array(markers_cfg[1]["corners_3d"], dtype=np.float32)
            self.pad_width_m = float(cfg.get("total_pad_width_m", 1.189))
            self.pad_height_m = float(cfg.get("total_pad_height_m", 0.841))
        else:
            self.dictionary_name = dictionary_name
            self.big_id = int(big_id)
            self.big_size_m = float(big_size_m)
            self.small_id = int(small_id)
            self.small_size_m = float(small_size_m)
            self.pad_width_m = 1.189
            self.pad_height_m = 0.841
            hb = self.big_size_m / 2.0
            big_center_x = -self.pad_width_m / 2.0 + 0.0205 + hb
            self.big_corners_3d = np.array([
                [big_center_x - hb,  hb, 0.0],
                [big_center_x + hb,  hb, 0.0],
                [big_center_x + hb, -hb, 0.0],
                [big_center_x - hb, -hb, 0.0]
            ], dtype=np.float32)
            
            hs = self.small_size_m / 2.0
            self.small_corners_3d = np.array([
                [-hs,  hs, 0.0],
                [ hs,  hs, 0.0],
                [ hs, -hs, 0.0],
                [-hs, -hs, 0.0]
            ], dtype=np.float32)

        self.marker_centers_3d = {
            self.big_id: np.mean(self.big_corners_3d, axis=0).reshape(3, 1),
            self.small_id: np.mean(self.small_corners_3d, axis=0).reshape(3, 1),
        }

        if self.dictionary_name not in ARUCO_DICT_MAP:
            raise ValueError(f"Unknown dictionary: {self.dictionary_name}")
            
        self.dict_id = ARUCO_DICT_MAP[self.dictionary_name]
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.dict_id)
        
        # Build OpenCV Board object
        obj_points = [self.big_corners_3d, self.small_corners_3d]
        board_ids = np.array([self.big_id, self.small_id], dtype=np.int32)
        if hasattr(cv2.aruco, "Board_create"):
            self.board = cv2.aruco.Board_create(obj_points, self.aruco_dict, board_ids)
        elif hasattr(cv2.aruco, "Board"):
            self.board = cv2.aruco.Board(obj_points, self.aruco_dict, board_ids)
        else:
            raise AttributeError("OpenCV cv2.aruco has neither Board_create nor Board constructor")
        
        # Base ArUco detector
        marker_sizes = {self.big_id: self.big_size_m, self.small_id: self.small_size_m}
        self.base_detector = ArUcoDetector(
            dictionary_name=self.dictionary_name,
            marker_size_meters=marker_sizes,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            target_marker_ids=[self.big_id, self.small_id]
        )

    def set_camera_parameters(self, camera_matrix: np.ndarray, dist_coeffs: Optional[np.ndarray] = None):
        """Update camera intrinsic matrix and distortion coefficients."""
        self.camera_matrix = camera_matrix
        if dist_coeffs is not None:
            self.dist_coeffs = dist_coeffs
        self.base_detector.set_camera_parameters(self.camera_matrix, self.dist_coeffs)

    def detect(
        self,
        color_image: np.ndarray,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
    ) -> Dict[str, Any]:
        """
        Detect the dual-scale board in a frame.
        
        Returns:
            Dictionary with board pose, tracking mode, and individual marker info.
        """
        K = camera_matrix if camera_matrix is not None else self.camera_matrix
        D = dist_coeffs if dist_coeffs is not None else self.dist_coeffs
        if D is None:
            D = np.zeros(5, dtype=np.float32)

        # 1. Detect individual markers with base detector
        single_results = self.base_detector.process_frame(color_image, K, D)
        
        detected_ids = [res["id"] for res in single_results if res["id"] in (self.big_id, self.small_id)]
        
        result: Dict[str, Any] = {
            "board_detected": False,
            "rvec": None,
            "tvec": None,
            "euler_deg": None,
            "distance_m": 0.0,
            "active_ids": detected_ids,
            "tracking_mode": "LOST",
            "single_markers": single_results,
            "num_corners_fused": 0,
        }
        
        if not detected_ids or K is None:
            return result

        # 2. Collect corners and IDs for board PnP solve.
        # When both markers are visible, fuse both markers (8 corners) so PnP
        # transitions smoothly without depth jumps or EKF innovation spikes.
        # When only one marker is visible, use the visible marker.
        board_corners = []
        board_ids_list = []
        for mid in (self.big_id, self.small_id):
            mid_markers = [res for res in single_results if res["id"] == mid]
            if mid_markers:
                best_m = max(
                    mid_markers,
                    key=lambda res: abs(float(cv2.contourArea(np.asarray(res["corners"]).reshape(-1, 2)))),
                )
                board_corners.append(best_m["corners"])
                board_ids_list.append(mid)

        board_ids_arr = np.array(board_ids_list, dtype=np.int32)
        pose_marker_id = self.big_id if self.big_id in detected_ids else self.small_id

        # 3. Solve Board PnP
        retval, rvec, tvec = cv2.aruco.estimatePoseBoard(
            board_corners,
            board_ids_arr,
            self.board,
            K,
            D,
            None,
            None
        )

        # estimatePoseBoard maps even a single observed tag into the configured
        # board frame. Keep the marker-frame fallback only for OpenCV failures.
        if (retval == 0 or rvec is None or tvec is None) and len(detected_ids) > 0:
            preferred_id = pose_marker_id
            sel_markers = [res for res in single_results if res["id"] == preferred_id]
            if sel_markers:
                res_m = max(
                    sel_markers,
                    key=lambda res: abs(float(cv2.contourArea(np.asarray(res["corners"]).reshape(-1, 2)))),
                )
                if res_m.get("rvec") is not None and res_m.get("tvec") is not None:
                    rvec = res_m["rvec"]
                    rotation, _ = cv2.Rodrigues(rvec)
                    marker_center = self.marker_centers_3d[preferred_id]
                    tvec = np.asarray(res_m["tvec"]).reshape(3, 1) - rotation @ marker_center
                    retval = 1

        if retval > 0 and rvec is not None and tvec is not None:
            rvec_flat = rvec.reshape(3, 1)
            tvec_flat = tvec.reshape(3, 1)
            dist = float(np.linalg.norm(tvec_flat))
            euler = rvec_to_euler(rvec_flat, degrees=True)
            
            # Determine tracking mode
            if self.big_id in detected_ids:
                mode = "OUTER_COARSE"
            else:
                mode = "INNER_FINE"

            result.update({
                "board_detected": True,
                "rvec": rvec_flat,
                "tvec": tvec_flat,
                "euler_deg": euler,
                "distance_m": dist,
                "tracking_mode": mode,
                "num_corners_fused": retval * 4,
            })

        return result

    def draw_visualizations(
        self,
        color_image: np.ndarray,
        board_result: Dict[str, Any],
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        Draw visual axes and tracking HUD overlay on the image.
        """
        K = camera_matrix if camera_matrix is not None else self.camera_matrix
        D = dist_coeffs if dist_coeffs is not None else self.dist_coeffs
        # Ensure 3-channel image for drawing
        if len(color_image.shape) == 2:
            annotated_base = cv2.cvtColor(color_image, cv2.COLOR_GRAY2BGR)
        else:
            annotated_base = color_image.copy()

        # Base visualization of individual markers
        annotated = self.base_detector.draw_results(
            annotated_base,
            board_result["single_markers"],
            K,
            D,
            draw_axes=False,
            draw_summary_table=False
        )

        h, w = annotated.shape[:2]

        if board_result["board_detected"] and K is not None:
            rvec = board_result["rvec"]
            tvec = board_result["tvec"]
            mode = board_result["tracking_mode"]
            dist_m = board_result["distance_m"]
            
            # Draw board center axis (origin of H-Pad)
            axis_len = 0.20 if mode != "INNER_FINE" else 0.08
            draw_axis_3d(annotated, K, D, rvec, tvec, length=axis_len, thickness=3)

            # Project center point
            center_2d, _ = cv2.projectPoints(np.zeros((1, 3), dtype=np.float32), rvec, tvec, K, D)
            cx, cy = int(center_2d[0, 0, 0]), int(center_2d[0, 0, 1])
            if 0 <= cx < w and 0 <= cy < h:
                cv2.circle(annotated, (cx, cy), 7, (0, 0, 255), -1)
                cv2.circle(annotated, (cx, cy), 12, (0, 255, 255), 2)
                cv2.putText(
                    annotated, "PAD CENTER (0,0)",
                    (cx + 15, cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2
                )

            # Draw top HUD banner
            mode_colors = {
                "DUAL_FUSED": (0, 255, 0),     # Bright Green
                "INNER_FINE": (0, 215, 255),   # Gold / Yellow
                "OUTER_COARSE": (255, 165, 0), # Orange
            }
            color = mode_colors.get(mode, (200, 200, 200))
            
            hud_text = (
                f"BOARD: {mode} | IDs: {board_result['active_ids']} | "
                f"Z: {tvec[2,0]:.2f}m | Dist: {dist_m:.2f}m"
            )
            cv2.rectangle(annotated, (10, 10), (w - 10, 45), (20, 20, 20), -1)
            cv2.rectangle(annotated, (10, 10), (w - 10, 45), color, 2)
            cv2.putText(annotated, hud_text, (20, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        else:
            cv2.rectangle(annotated, (10, 10), (320, 45), (20, 20, 20), -1)
            cv2.rectangle(annotated, (10, 10), (320, 45), (0, 0, 255), 2)
            cv2.putText(annotated, "BOARD: TARGET LOST", (20, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        return annotated
