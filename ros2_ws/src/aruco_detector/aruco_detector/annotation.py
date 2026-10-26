from typing import List

import cv2
import numpy as np

from .aruco_logic import Detection

OUTLINE_COLOR_BGR = (0, 255, 0)
INNER_COLOR_BGR = (0, 215, 255)
LABEL_TEXT_COLOR_BGR = (0, 0, 0)
AXIS_COLORS_BGR = ((0, 0, 255), (0, 255, 0), (255, 0, 0))


def ensure_bgr(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image.copy()


def draw_outline(image: np.ndarray, corners: np.ndarray, color=OUTLINE_COLOR_BGR) -> None:
    cv2.polylines(image, [corners.astype(int)], True, color, 2)


def draw_id_label(image: np.ndarray, corners: np.ndarray, marker_id: int, color=OUTLINE_COLOR_BGR) -> None:
    origin_x, origin_y = (int(value) for value in corners[0])
    text = f"ID:{marker_id}"
    (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
    top = max(0, origin_y - text_height - 8)
    cv2.rectangle(image, (origin_x, top), (origin_x + text_width + 8, origin_y), color, -1)
    cv2.putText(
        image,
        text,
        (origin_x + 4, max(text_height + 2, origin_y - 4)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        LABEL_TEXT_COLOR_BGR,
        2,
    )


def project_axis_points(
    detection: Detection, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, length_m: float
) -> np.ndarray:
    axis_points = np.float32([[0, 0, 0], [length_m, 0, 0], [0, length_m, 0], [0, 0, length_m]])
    projected, _ = cv2.projectPoints(
        axis_points, detection["rvec"], detection["tvec"], camera_matrix, dist_coeffs
    )
    return projected.reshape(-1, 2).astype(int)


def draw_axes(
    image: np.ndarray,
    detection: Detection,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    length_m: float,
) -> None:
    points = project_axis_points(detection, camera_matrix, dist_coeffs, length_m)
    origin = tuple(int(value) for value in points[0])
    for axis_end, color in zip(points[1:], AXIS_COLORS_BGR):
        cv2.line(image, origin, tuple(int(value) for value in axis_end), color, 3)


def draw_detections(
    image: np.ndarray,
    detections: List[Detection],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_size_m: float,
) -> np.ndarray:
    annotated = ensure_bgr(image)
    for detection in detections:
        all_c = detection.get("all_corners", {})
        if all_c:
            for mid, c in all_c.items():
                col = INNER_COLOR_BGR if mid == 43 else OUTLINE_COLOR_BGR
                draw_outline(annotated, c, col)
                draw_id_label(annotated, c, mid, col)
        else:
            draw_outline(annotated, detection["corners"])
            draw_id_label(annotated, detection["corners"], detection["id"])

        # Draw coordinate axis at pad origin (0, 0, 0)
        draw_axes(annotated, detection, camera_matrix, dist_coeffs, 0.20)

        # Draw HUD info
        mode = detection.get("tracking_mode", "DETECTED")
        p = detection["pose"]
        hud = f"PAD: {mode} | Z={p['z']:.2f}m Dist={p['distance']:.2f}m"
        cv2.putText(annotated, hud, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return annotated
