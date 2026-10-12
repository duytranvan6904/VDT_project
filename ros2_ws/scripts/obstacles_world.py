import math
import random

import numpy as np


def make_cylinders(seed, num_obs, map_size, max_h, clear_radius, start, extra, margin=1.0):
    rng = random.Random(seed)
    half = map_size / 2.0
    cyl = []
    for _ in range(num_obs * 200):
        if len(cyl) >= num_obs:
            break
        cx, cy = rng.uniform(-half, half), rng.uniform(-half, half)
        r, h = rng.uniform(0.3, 0.8), rng.uniform(2.0, max_h)
        if math.hypot(cx, cy) < clear_radius:
            continue
        if start and math.hypot(cx - start[0], cy - start[1]) < r + margin:
            continue
        cyl.append((cx, cy, r, h))
    return cyl + [tuple(float(v) for v in e) for e in extra or []]


def to_points(cyl, res):
    out = []
    for cx, cy, r, h in cyl:
        zs = np.arange(0.0, h + 1e-9, res)
        n = max(int(2 * math.pi * r / res), 8)
        th = np.arange(n) * (2 * math.pi / n)
        out.append(np.column_stack([
            np.tile(cx + r * np.cos(th), len(zs)),
            np.tile(cy + r * np.sin(th), len(zs)),
            np.repeat(zs, n)]))
        for rr in np.arange(r, 0.0, -res):
            m = max(int(2 * math.pi * rr / res), 4)
            t = np.arange(m) * (2 * math.pi / m)
            out.append(np.column_stack([cx + rr * np.cos(t), cy + rr * np.sin(t), np.full(m, h)]))
    if not out:
        return np.empty((0, 3), dtype=np.float32)
    return np.vstack(out).astype(np.float32)


def to_cloud_msg(pts, frame_id='world'):
    from sensor_msgs.msg import PointCloud2, PointField
    m = PointCloud2()
    m.header.frame_id = frame_id
    m.height, m.width = 1, len(pts)
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate('xyz')]
    m.is_bigendian = False
    m.point_step = 12
    m.row_step = 12 * len(pts)
    m.is_dense = True
    m.data = np.ascontiguousarray(pts, dtype=np.float32).tobytes()
    return m


def build_world(seed, num_obs, map_size, max_h, clear_radius, start, extra, res):
    cyl = make_cylinders(seed, num_obs, map_size, max_h, clear_radius, start, extra)
    obs = [(cx, cy, h / 2.0, r, h) for cx, cy, r, h in cyl]
    return cyl, obs, to_cloud_msg(to_points(cyl, res))