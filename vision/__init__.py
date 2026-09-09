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
from .target_state_ekf import EstimatorSnapshot, TargetStateEKF
from .target_state_ct_ekf import CTStateSnapshot, CoordinatedTurnEKF
from .target_tracking_policy import TrackingMode, classify_tracking_mode
from .target_state_imm import TargetStateIMM

__all__ = [
    'RealSenseCamera',
    'ArUcoDetector',
    'DepthMasker',
    'rvec_to_euler',
    'rvec_to_quaternion',
    'extract_bounding_box',
    'draw_axis_3d',
    'EstimatorSnapshot',
    'TargetStateEKF',
    'CTStateSnapshot',
    'CoordinatedTurnEKF',
    'TrackingMode',
    'classify_tracking_mode',
    'TargetStateIMM',
]
