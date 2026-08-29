import cv2
import numpy as np
from typing import Tuple, Dict, Any, Optional

def rvec_to_euler(rvec: np.ndarray, degrees: bool = True) -> Tuple[float, float, float]:
    """
    Convert Rodrigues rotation vector (rvec) to Euler angles (Roll, Pitch, Yaw).
    
    Args:
        rvec: 3x1 or 1x3 rotation vector from cv2.solvePnP
        degrees: Return angles in degrees if True, radians if False
        
    Returns:
        Tuple of (roll, pitch, yaw)
    """
    R, _ = cv2.Rodrigues(rvec)
    
    # Calculate Euler angles from rotation matrix (ZYX convention: Yaw, Pitch, Roll)
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0

    if degrees:
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)
    return roll, pitch, yaw


def rvec_to_quaternion(rvec: np.ndarray) -> np.ndarray:
    """
    Convert Rodrigues rotation vector (rvec) to Quaternion [qw, qx, qy, qz].
    
    Args:
        rvec: 3x1 or 1x3 rotation vector
        
    Returns:
        np.ndarray of shape (4,) containing [qw, qx, qy, qz]
    """
    R, _ = cv2.Rodrigues(rvec)
    tr = np.trace(R)
    
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
        
    return np.array([qw, qx, qy, qz], dtype=np.float64)


def extract_bounding_box(
    corners: np.ndarray, 
    margin_percent: float = 0.15,
    img_shape: Optional[Tuple[int, int]] = None
) -> Tuple[int, int, int, int]:
    """
    Extract 2D bounding box (x_min, y_min, width, height) from 4 ArUco corner points.
    Includes a margin expansion for Depth Masking (Task N3).
    
    Args:
        corners: 4x2 array of corner points [[u0, v0], [u1, v1], [u2, v2], [u3, v3]]
        margin_percent: Fractional expansion margin (e.g. 0.15 = 15% expansion)
        img_shape: Optional (height, width) to clip bounding box within image boundaries
        
    Returns:
        Tuple of (x_min, y_min, width, height)
    """
    pts = corners.reshape((4, 2))
    u_min, v_min = np.min(pts, axis=0)
    u_max, v_max = np.max(pts, axis=0)
    
    w = u_max - u_min
    h = v_max - v_min
    
    # Expand by margin_percent
    u_min_exp = u_min - margin_percent * w
    v_min_exp = v_min - margin_percent * h
    w_exp = w * (1.0 + 2.0 * margin_percent)
    h_exp = h * (1.0 + 2.0 * margin_percent)
    
    x_min = int(max(0, u_min_exp))
    y_min = int(max(0, v_min_exp))
    
    if img_shape is not None:
        img_h, img_w = img_shape[:2]
        x_max = int(min(img_w, u_min_exp + w_exp))
        y_max = int(min(img_h, v_min_exp + h_exp))
        return x_min, y_min, max(0, x_max - x_min), max(0, y_max - y_min)
    
    return x_min, y_min, int(w_exp), int(h_exp)


def draw_axis_3d(
    img: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    length: float = 0.1,
    thickness: int = 3
) -> np.ndarray:
    """
    Draw 3D Coordinate Axes (X=Red, Y=Green, Z=Blue) on the image at marker origin.
    Works across all OpenCV versions using cv2.projectPoints or cv2.drawFrameAxes.
    
    Args:
        img: Input BGR image
        camera_matrix: 3x3 Intrinsic matrix K
        dist_coeffs: Distortion coefficients D
        rvec: 3x1 Rotation vector
        tvec: 3x1 Translation vector (in meters)
        length: Axis length in meters
        thickness: Line thickness in pixels
        
    Returns:
        Annotated BGR image
    """
    # 3D points of origin and axes end-points in object coordinate system
    axis_points = np.float32([
        [0, 0, 0],
        [length, 0, 0],      # X-axis (Red)
        [0, length, 0],      # Y-axis (Green)
        [0, 0, length]       # Z-axis (Blue)
    ]).reshape(-1, 3)

    imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    imgpts = imgpts.astype(int).reshape(-1, 2)

    origin = tuple(imgpts[0])
    pt_x = tuple(imgpts[1])
    pt_y = tuple(imgpts[2])
    pt_z = tuple(imgpts[3])

    # Draw line X (Red)
    cv2.line(img, origin, pt_x, (0, 0, 255), thickness)
    cv2.putText(img, "X", pt_x, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Draw line Y (Green)
    cv2.line(img, origin, pt_y, (0, 255, 0), thickness)
    cv2.putText(img, "Y", pt_y, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Draw line Z (Blue)
    cv2.line(img, origin, pt_z, (255, 0, 0), thickness)
    cv2.putText(img, "Z", pt_z, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    return img
