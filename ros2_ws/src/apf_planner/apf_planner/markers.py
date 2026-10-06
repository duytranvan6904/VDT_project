from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def _pt(v) -> Point:
    return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def _arrow(stamp, frame, ns, mid, start, vec, rgb, shaft, head, alpha):
    m = Marker()
    m.header.stamp = stamp
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.scale.x, m.scale.y = shaft, head
    m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, alpha)
    m.points = [_pt(start), _pt(np.asarray(start) + vec)]
    return m


def _body(stamp, frame, ns, mid, mtype, pos, scale, rgba):
    m = Marker()
    m.header.stamp = stamp
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = mtype
    m.action = Marker.ADD
    m.pose.position = _pt(pos)
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    return m


def _scaled(vec, ref, cap=2.0):
    n = float(np.linalg.norm(vec))
    if n <= 1e-4:
        return np.zeros(3)
    return vec / n * min(cap, n / ref)


def build_markers(stamp, frame, drone_pos, goal, result, params, cylinders, obstacle_points):
    arr = MarkerArray()

    if result is not None and drone_pos is not None:
        start = drone_pos
        arr.markers.append(_arrow(
            stamp, frame, 'apf_att', 0, start,
            _scaled(result.f_att, max(params.k_att, 1.0)),
            (0.0, 1.0, 0.0), 0.08, 0.15, 0.85))
        arr.markers.append(_arrow(
            stamp, frame, 'apf_rep', 1, start,
            _scaled(result.f_rep, max(params.k_rep * 0.0001, 1.0)),
            (1.0, 0.0, 0.0), 0.08, 0.15, 0.85))
        v = result.velocity
        vn = float(np.linalg.norm(v))
        vec = v / vn * min(2.0, vn) if vn > 0.05 else np.zeros(3)
        arr.markers.append(_arrow(
            stamp, frame, 'apf_total', 2, start, vec,
            (0.0, 0.0, 1.0), 0.10, 0.18, 0.9))
        if getattr(result, 'tangent_active', False):
            arr.markers.append(_arrow(
                stamp, frame, 'apf_tangent', 5, start,
                _scaled(result.f_tan, 1.0),
                (1.0, 0.1, 0.9), 0.09, 0.16, 0.95))

    if drone_pos is not None:
        arr.markers.append(_body(
            stamp, frame, 'apf_drone', 3, Marker.SPHERE, drone_pos,
            (0.35, 0.35, 0.35), (0.1, 0.8, 0.9, 0.8)))

    if goal is not None:
        arr.markers.append(_body(
            stamp, frame, 'apf_goal', 4, Marker.SPHERE, goal,
            (0.4, 0.4, 0.4), (1.0, 0.8, 0.0, 0.9)))

    for idx, (cx, cy, r, h, cz) in enumerate(cylinders):
        arr.markers.append(_body(
            stamp, frame, 'apf_obstacles', 10 + idx, Marker.CYLINDER,
            (cx, cy, cz), (2 * r, 2 * r, h), (0.95, 0.45, 0.15, 0.55)))

    pts = _body(stamp, frame, 'apf_obs_points', 6, Marker.SPHERE_LIST,
                (0.0, 0.0, 0.0), (0.2, 0.2, 0.2), (1.0, 0.1, 0.1, 0.9))
    pts.points = [_pt(p) for p in obstacle_points]
    arr.markers.append(pts)
    return arr
