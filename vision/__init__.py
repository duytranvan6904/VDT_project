"""
Vision Package for Quadrotor H-Pad Tracking & Pose Estimation
Submodule developed for Intel RealSense D435 / D435i camera.
"""

from .realsense_stream import RealSenseCamera
from .aruco_detector import ArUcoDetector
from .depth_masker import DepthMasker
from .utils import (
    rvec_to_euler,
    rvec_to_quaternion,
    extract_bounding_box,
    draw_axis_3d
)

__all__ = [
    'RealSenseCamera',
    'ArUcoDetector',
    'DepthMasker',
    'rvec_to_euler',
    'rvec_to_quaternion',
    'extract_bounding_box',
    'draw_axis_3d'
]
