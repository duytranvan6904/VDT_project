import math

import pytest

from servo_control.servo_logic import (
    angle_to_pwm_us,
    clamp,
    control_angle_to_servo_angle,
)


def test_clamp_limits_value():
    assert clamp(200.0, 0.0, 180.0) == 180.0
    assert clamp(-10.0, 0.0, 180.0) == 0.0
    assert clamp(90.0, 0.0, 180.0) == 90.0


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_clamp_replaces_non_finite_value(value):
    assert clamp(value, 0.0, 180.0) == 0.0


def test_control_angle_is_offset_from_home_and_clamped():
    assert control_angle_to_servo_angle(20.0, 90.0, 0.0, 180.0) == 110.0
    assert control_angle_to_servo_angle(-200.0, 90.0, 0.0, 180.0) == 0.0


def test_invalid_control_angle_returns_safe_home():
    assert control_angle_to_servo_angle(math.nan, 90.0, 0.0, 180.0) == 90.0
    assert control_angle_to_servo_angle(10.0, math.inf, 0.0, 180.0) == 0.0


def test_angle_to_pwm_maps_range():
    assert angle_to_pwm_us(0.0, 0.0, 180.0, 600.0, 2400.0) == 600.0
    assert angle_to_pwm_us(90.0, 0.0, 180.0, 600.0, 2400.0) == 1500.0
    assert angle_to_pwm_us(180.0, 0.0, 180.0, 600.0, 2400.0) == 2400.0


def test_angle_to_pwm_handles_degenerate_range():
    assert angle_to_pwm_us(90.0, 10.0, 10.0, 600.0, 2400.0) == 600.0
