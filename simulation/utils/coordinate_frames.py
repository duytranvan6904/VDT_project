"""Small coordinate-frame helpers shared by simulation nodes."""

from __future__ import annotations

import numpy as np


def rotate_body_vector_to_world(quaternion_xyzw, vector_body) -> np.ndarray:
    """Rotate a vector from an odometry child/body frame into its world frame."""
    q = np.asarray(quaternion_xyzw, dtype=float)
    v = np.asarray(vector_body, dtype=float)
    if q.shape != (4,) or v.shape != (3,):
        raise ValueError('quaternion_xyzw must have 4 elements and vector_body 3')

    norm = float(np.linalg.norm(q))
    if norm < 1e-9:
        raise ValueError('quaternion must be nonzero')
    x, y, z, w = q / norm
    rotation = np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])
    return rotation @ v
