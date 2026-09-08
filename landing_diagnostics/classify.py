from enum import Enum


class DriftCause(Enum):
    UNDERDAMPED = 'underdamped'
    STEADY_STATE_OFFSET = 'steady_state_offset'
    GROUND_EFFECT = 'ground_effect'
    UNKNOWN = 'setpoint_or_estimator_issue'


def classify_drift(stats, oscillation_threshold_hz=0.5, offset_threshold_m_s=0.15,
                    corr_threshold=0.5):
    if stats.oscillation_freq_hz > oscillation_threshold_hz and \
            stats.max_err_xy > 2 * stats.mean_err_xy:
        return DriftCause.UNDERDAMPED

    is_steady_offset = abs(stats.mean_vx) > offset_threshold_m_s or \
        abs(stats.mean_vy) > offset_threshold_m_s
    if is_steady_offset and stats.std_err_xy < stats.mean_err_xy:
        return DriftCause.STEADY_STATE_OFFSET

    if stats.altitude_error_corr < -corr_threshold:
        return DriftCause.GROUND_EFFECT

    return DriftCause.UNKNOWN