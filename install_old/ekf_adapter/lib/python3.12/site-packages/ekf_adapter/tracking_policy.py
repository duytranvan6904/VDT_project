from enum import Enum


class TrackingMode(Enum):
    """Tên mode phải khớp với danh sách mà input_state_cache chấp nhận.

    ISC coi TRACKING, PREDICTING, PREDICTING_DEGRADED là EKF hợp lệ;
    EXPIRED hoặc chuỗi lạ là không hợp lệ.
    """

    TRACKING = "TRACKING"
    PREDICTING = "PREDICTING"
    PREDICTING_DEGRADED = "PREDICTING_DEGRADED"
    EXPIRED = "EXPIRED"


TRACKING_MAX_AGE_S = 0.25
COAST_LIMIT_S = {"APPROACH": 1.0, "FOLLOW": 2.0}
DEFAULT_PHASE = "FOLLOW"
DEGRADED_FRACTION = 0.5


def classify_tracking_mode(
    detected: bool,
    age_since_measurement_s: float,
    phase: str = DEFAULT_PHASE,
) -> TrackingMode:
    limit = COAST_LIMIT_S.get(phase, COAST_LIMIT_S[DEFAULT_PHASE])
    if detected and age_since_measurement_s <= TRACKING_MAX_AGE_S:
        return TrackingMode.TRACKING
    if age_since_measurement_s <= limit * DEGRADED_FRACTION:
        return TrackingMode.PREDICTING
    if age_since_measurement_s <= limit:
        return TrackingMode.PREDICTING_DEGRADED
    return TrackingMode.EXPIRED