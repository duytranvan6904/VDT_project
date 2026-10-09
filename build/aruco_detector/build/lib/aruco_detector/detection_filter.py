from typing import List

from .aruco_logic import Detection


def is_far_enough(detection: Detection, min_distance_m: float) -> bool:
    return detection["pose"]["distance"] >= min_distance_m


def is_high_enough(detection: Detection, min_z_m: float) -> bool:
    return detection["pose"]["z"] >= min_z_m


def filter_detections(
    detections: List[Detection], min_distance_m: float, min_z_m: float
) -> List[Detection]:
    return [
        detection
        for detection in detections
        if is_far_enough(detection, min_distance_m) and is_high_enough(detection, min_z_m)
    ]
