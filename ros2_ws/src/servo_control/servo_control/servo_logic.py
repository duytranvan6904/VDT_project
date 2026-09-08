import math


def clamp(value, lo, hi):
    if not math.isfinite(value) or not math.isfinite(lo) or not math.isfinite(hi):
        return lo
    return max(lo, min(value, hi))


def angle_to_pwm_us(angle_deg, angle_min_deg, angle_max_deg, pwm_min_us, pwm_max_us):
    if angle_max_deg <= angle_min_deg:
        return pwm_min_us
    angle_clamped = clamp(angle_deg, angle_min_deg, angle_max_deg)
    ratio = (angle_clamped - angle_min_deg) / (angle_max_deg - angle_min_deg)
    return pwm_min_us + ratio * (pwm_max_us - pwm_min_us)


def control_angle_to_servo_angle(control_angle_deg, home_angle_deg, angle_min_deg, angle_max_deg):
    if not math.isfinite(control_angle_deg) or not math.isfinite(home_angle_deg):
        return clamp(home_angle_deg, angle_min_deg, angle_max_deg)
    servo_angle = home_angle_deg + control_angle_deg
    return clamp(servo_angle, angle_min_deg, angle_max_deg)