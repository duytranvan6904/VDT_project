import math

PITCH_DEADBAND_PX = 6.0
YAW_DEADBAND_PX = 12.0
YAW_FF_LIMIT = 0.15


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def slew_angle(current: float, target: float, max_step: float) -> float:
    error = wrap_angle(target - current)
    return wrap_angle(current + max(-max_step, min(max_step, error)))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def deadband(error: float, width: float) -> float:
    if abs(error) < width:
        return 0.0
    return error - math.copysign(width, error)


def pixel_pitch_correction(ev_pixels: float, k_pitch: float, focal_y: float, gain: float) -> float:
    return -k_pitch * (deadband(ev_pixels, PITCH_DEADBAND_PX) / focal_y) * gain


def tangential_yaw_feedforward(dx: float, dy: float, vx: float, vy: float) -> float:
    dist_h = math.hypot(dx, dy)
    if dist_h <= 0.0:
        return 0.0
    v_tan = (-dy * vx + dx * vy) / dist_h
    return max(-YAW_FF_LIMIT, min(YAW_FF_LIMIT, v_tan / dist_h))