from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from typing import List, Tuple

import numpy as np


def load_cylinder_obstacles_from_sdf(
    sdf_path: str,
) -> List[Tuple[float, float, float, float, float]]:
    """Parse Gazebo SDF and return cylinder obstacles as (cx, cy, r, h, cz).

    cz is the center altitude of the cylinder (= h/2 if sitting on ground).
    """
    if not os.path.isfile(sdf_path):
        return []
    root = ET.parse(sdf_path).getroot()
    result = []
    for model in root.findall('./world/model'):
        name = model.attrib.get('name', '')
        if not (name.startswith('cyl_') or name.startswith('cylinder_obs_')):
            continue
        pose = model.findtext('pose', '').split()
        cyl = model.find('.//cylinder')
        if len(pose) < 3 or cyl is None:
            continue
        cx, cy = float(pose[0]), float(pose[1])
        r = float(cyl.findtext('radius', '0.5'))
        h = float(cyl.findtext('length', '1.0'))
        cz = float(pose[2])  # SDF pose z is the center of the cylinder
        result.append((cx, cy, r, h, cz))
    return result


def cylinder_nearest_point(
    drone_pos: np.ndarray,
    cx: float, cy: float, r: float, h: float, cz: float,
) -> np.ndarray:
    """Return the nearest surface point on a vertical cylinder to the drone.

    The cylinder axis is along Z from (cz - h/2) to (cz + h/2).
    """
    # Horizontal distance from cylinder axis
    dx = drone_pos[0] - cx
    dy = drone_pos[1] - cy
    dist_h = math.sqrt(dx * dx + dy * dy)

    # Nearest point on cylinder surface (horizontal)
    if dist_h > 1e-6:
        nx = cx + r * dx / dist_h
        ny = cy + r * dy / dist_h
    else:
        nx = cx + r  # arbitrary direction when on axis
        ny = cy

    # Nearest z on cylinder
    z_low = cz - h / 2.0
    z_high = cz + h / 2.0
    nz = float(np.clip(drone_pos[2], z_low, z_high))

    return np.array([nx, ny, nz])


def cloud_to_xyz(msg) -> np.ndarray:
    offsets = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    if n == 0 or any(k not in offsets for k in 'xyz'):
        return np.empty((0, 3))
    if len(msg.data) < n * msg.point_step:
        return np.empty((0, 3))
    buf = np.frombuffer(msg.data, dtype=np.uint8, count=n * msg.point_step)
    buf = buf.reshape(n, msg.point_step)
    dt = '>f4' if msg.is_bigendian else '<f4'
    cols = [
        np.ascontiguousarray(buf[:, offsets[k]:offsets[k] + 4]).view(dt).reshape(n)
        for k in 'xyz'
    ]
    xyz = np.stack(cols, axis=1).astype(float)
    return xyz[np.isfinite(xyz).all(axis=1)]


def voxel_downsample(xyz: np.ndarray, voxel: float) -> np.ndarray:
    if voxel <= 0.0 or len(xyz) == 0:
        return xyz
    keys = np.floor(xyz / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyz[idx]


def select_obstacle_points(
    cloud: np.ndarray,
    pos: np.ndarray,
    radius: float,
    suppress: float,
    max_points: int,
) -> List[Tuple[float, float, float]]:
    if len(cloud) == 0:
        return []
    d = np.linalg.norm(cloud - pos, axis=1)
    mask = d <= radius
    pts, d = cloud[mask], d[mask]
    out: List[Tuple[float, float, float]] = []
    while len(pts) and len(out) < max_points:
        p = pts[int(np.argmin(d))]
        out.append((float(p[0]), float(p[1]), float(p[2])))
        keep = np.linalg.norm(pts - p, axis=1) > suppress
        pts, d = pts[keep], d[keep]
    return out
