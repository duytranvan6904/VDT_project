"""Timeout policy for short-term prediction and safe tracking fallback."""

from __future__ import annotations

from enum import Enum


PREDICTION_HORIZON_S = 1.0
SEARCH_TIMEOUT_S = {"APPROACH": 2.0, "FOLLOW": 3.0}


class TrackingMode(str, Enum):
    TRACKING = "TRACKING"
    PREDICTING = "PREDICTING"
    PREDICTING_DEGRADED = "PREDICTING_DEGRADED"
    EXPIRED = "EXPIRED"


def classify_tracking_mode(
    detected: bool,
    age_since_measurement_s: float,
    phase: str = "FOLLOW",
) -> TrackingMode:
    """Classify estimator output for the mission FSM.

    ``PREDICTING`` is the only lost-measurement mode allowed to feed the
    planner.  Degraded prediction is reserved for gimbal reacquisition/hover;
    expired prediction must trigger the SEARCH safety behavior.
    """
    phase = phase.upper()
    if phase not in SEARCH_TIMEOUT_S:
        raise ValueError(f"unknown tracking phase: {phase}")
    if not isinstance(detected, (bool,)):
        raise TypeError("detected must be bool")
    if age_since_measurement_s < 0.0:
        raise ValueError("age_since_measurement_s must be non-negative")
    if detected:
        return TrackingMode.TRACKING
    if age_since_measurement_s <= PREDICTION_HORIZON_S:
        return TrackingMode.PREDICTING
    if age_since_measurement_s <= SEARCH_TIMEOUT_S[phase]:
        return TrackingMode.PREDICTING_DEGRADED
    return TrackingMode.EXPIRED
