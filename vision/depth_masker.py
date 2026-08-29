import cv2
import numpy as np
from typing import List, Tuple, Dict, Any, Optional, Union


class DepthMasker:
    """
    Depth Masking Processor for Obstacle Avoidance (VO-APF Pipeline Task N3).
    Excludes detected H-Pad landing regions from RealSense depth maps to prevent
    the obstacle avoidance planner from treating the landing target as an obstacle.
    """
    def __init__(self, margin_percent: float = 0.15):
        """
        Initialize Depth Masker.
        
        Args:
            margin_percent: Fractional expansion margin (e.g., 0.15 = 15% expansion around ArUco bounds)
        """
        self.margin_percent = margin_percent

    def create_mask(
        self,
        image_shape: Tuple[int, int],
        detection_results: List[Dict[str, Any]],
        use_polygon: bool = True
    ) -> np.ndarray:
        """
        Create a 2D binary mask of detected H-Pad markers.
        
        Args:
            image_shape: (height, width) tuple of the frame
            detection_results: Output list from ArUcoDetector.process_frame()
            use_polygon: If True, uses dilated corner polygons; if False, uses expanded Bounding Boxes
            
        Returns:
            binary_mask: uint8 image of shape (H, W) where 255 = H-Pad region (to mask out), 0 = valid obstacle region
        """
        h, w = image_shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)

        for res in detection_results:
            if use_polygon and "corners" in res:
                corners = res["corners"].reshape((4, 2))
                center = np.mean(corners, axis=0)
                # Expand corners outward from center by (1 + margin_percent)
                expanded_corners = center + (corners - center) * (1.0 + self.margin_percent)
                pts = expanded_corners.astype(np.int32).reshape((-1, 1, 2))
                cv2.fillPoly(mask, [pts], 255)
            elif "bbox" in res:
                bx, by, bw, bh = res["bbox"]
                cv2.rectangle(mask, (bx, by), (bx + bw, by + bh), 255, -1)

        return mask

    def mask_depth_frame(
        self,
        depth_frame: Any,
        detection_results: List[Dict[str, Any]],
        fill_value: Union[int, float] = 0,
        use_polygon: bool = True
    ) -> Tuple[Optional[np.ndarray], np.ndarray]:
        """
        Apply mask to zero-out / exclude H-Pad regions from depth frame.
        Supports both numpy ndarray and pyrealsense2 rs.depth_frame objects.
        
        Args:
            depth_frame: 2D numpy array OR rs.depth_frame object
            detection_results: Output list from ArUcoDetector.process_frame()
            fill_value: Value to set for H-Pad pixels (default: 0)
            use_polygon: Use precise dilated polygon or rectangular BBox
            
        Returns:
            Tuple of (masked_depth_array, binary_mask)
        """
        if depth_frame is None:
            return None, np.zeros((1, 1), dtype=np.uint8)

        # Convert pyrealsense2 rs.depth_frame to numpy ndarray if needed
        if hasattr(depth_frame, "get_data"):
            try:
                depth_array = np.asanyarray(depth_frame.get_data())
            except Exception:
                return None, np.zeros((1, 1), dtype=np.uint8)
        elif isinstance(depth_frame, np.ndarray):
            depth_array = depth_frame
        else:
            return None, np.zeros((1, 1), dtype=np.uint8)

        if depth_array is None or depth_array.size == 0:
            return depth_array, np.zeros((1, 1), dtype=np.uint8)

        mask = self.create_mask(depth_array.shape, detection_results, use_polygon=use_polygon)
        
        masked_depth = depth_array.copy()
        masked_depth[mask == 255] = fill_value

        return masked_depth, mask

    def visualize_masked_depth(
        self,
        depth_vis: np.ndarray,
        binary_mask: np.ndarray,
        color: Tuple[int, int, int] = (0, 0, 255),
        alpha: float = 0.4
    ) -> Optional[np.ndarray]:
        """
        Overlay translucent color tint (e.g. Red) over masked H-Pad regions in depth visualization.
        
        Args:
            depth_vis: 3-channel BGR depth color map visualization
            binary_mask: uint8 mask where 255 = masked region
            color: BGR tuple for masked region highlight (default: (0, 0, 255) Red)
            alpha: Transparency factor (0.0 to 1.0)
            
        Returns:
            Annotated BGR visualization image
        """
        if depth_vis is None or len(depth_vis.shape) != 3:
            return depth_vis

        if binary_mask is None or binary_mask.size <= 1:
            return depth_vis

        vis = depth_vis.copy()
        
        # Ensure mask resolution matches depth visualization resolution
        if binary_mask.shape[:2] != vis.shape[:2]:
            mask_resized = cv2.resize(binary_mask, (vis.shape[1], vis.shape[0]), interpolation=cv2.INTER_NEAREST)
        else:
            mask_resized = binary_mask

        colored_overlay = vis.copy()
        colored_overlay[mask_resized == 255] = color

        cv2.addWeighted(colored_overlay, alpha, vis, 1.0 - alpha, 0, vis)
        
        # Contour border outline around masked regions
        contours, _ = cv2.findContours(mask_resized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, contours, -1, (255, 0, 255), 2)

        return vis
