from typing import Optional, Tuple

import numpy as np


def fallback_camera_matrix(width: int, height: int, horizontal_fov_rad: float) -> np.ndarray:
    focal = width / (2.0 * np.tan(horizontal_fov_rad / 2.0))
    return np.array(
        [[focal, 0.0, width / 2.0], [0.0, focal, height / 2.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def zero_distortion() -> np.ndarray:
    return np.zeros((5, 1), dtype=np.float64)


def camera_info_to_intrinsics(camera_info) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    if camera_info.k[0] <= 0.0 or camera_info.k[4] <= 0.0:
        return None
    camera_matrix = np.array(camera_info.k, dtype=np.float64).reshape((3, 3))
    if len(camera_info.d) == 0:
        return camera_matrix, zero_distortion()
    return camera_matrix, np.array(camera_info.d, dtype=np.float64).reshape((-1, 1))
