"""Shared readers for geometry that belongs to the active Gazebo world."""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET


def load_hpad_pose_from_sdf(
    sdf_path: str,
    *,
    model_name: str = 'hpad_aruco',
    fallback: tuple[float, float, float] = (3.0, 1.0, 0.02),
) -> tuple[float, float, float]:
    """Read the H-Pad include pose so truth, control, and metrics share one source."""
    if not sdf_path or not os.path.isfile(sdf_path):
        return tuple(float(v) for v in fallback)

    try:
        root = ET.parse(sdf_path).getroot()
    except (ET.ParseError, OSError, ValueError):
        return tuple(float(v) for v in fallback)

    world = root.find('world')
    if world is None:
        return tuple(float(v) for v in fallback)

    for include in world.findall('include'):
        name = (include.findtext('name') or '').strip()
        uri = (include.findtext('uri') or '').strip()
        if name != model_name and 'arucotag' not in uri:
            continue
        pose_text = include.findtext('pose') or ''
        try:
            parts = [float(v) for v in pose_text.split()]
        except ValueError:
            continue
        if len(parts) >= 3:
            return parts[0], parts[1], parts[2]

    for model in world.findall('model'):
        if model.get('name') != model_name:
            continue
        pose_text = model.findtext('pose') or ''
        try:
            parts = [float(v) for v in pose_text.split()]
        except ValueError:
            continue
        if len(parts) >= 3:
            return parts[0], parts[1], parts[2]

    return tuple(float(v) for v in fallback)
