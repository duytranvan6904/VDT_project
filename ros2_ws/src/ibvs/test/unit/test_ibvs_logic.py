import math
from types import SimpleNamespace

import pytest

from ibvs.ibvs_logic import (
    PITCH_DEADBAND_PX,
    YAW_DEADBAND_PX,
    YAW_FF_LIMIT,
    clamp,
    deadband,
    pixel_pitch_correction,
    quaternion_to_yaw,
    slew_angle,
    tangential_yaw_feedforward,
    wrap_angle,
)


def euler_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return SimpleNamespace(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def angle_difference(a, b):
    return abs(wrap_angle(a - b))


@pytest.mark.parametrize("yaw", [0.0, 0.5, -0.5, math.pi / 2, -math.pi / 2, 3.0, -3.0])
def test_quaternion_to_yaw_round_trip(yaw):
    assert quaternion_to_yaw(euler_quaternion(0.0, 0.0, yaw)) == pytest.approx(yaw)


def test_quaternion_to_yaw_ignores_roll_and_pitch():
    q = euler_quaternion(0.3, -0.2, 1.1)
    assert quaternion_to_yaw(q) == pytest.approx(1.1)


def test_quaternion_to_yaw_identity_is_zero():
    assert quaternion_to_yaw(SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)) == 0.0


@pytest.mark.parametrize(
    "angle,expected",
    [(0.0, 0.0), (1.0, 1.0), (-1.0, -1.0), (2 * math.pi, 0.0), (1.5 * math.pi, -0.5 * math.pi), (-1.5 * math.pi, 0.5 * math.pi)],
)
def test_wrap_angle_values(angle, expected):
    assert wrap_angle(angle) == pytest.approx(expected, abs=1e-12)


@pytest.mark.parametrize("angle", [-20.0, -7.0, -math.pi, 0.0, 3.0, math.pi, 10.0, 100.0])
def test_wrap_angle_range_and_equivalence(angle):
    wrapped = wrap_angle(angle)
    assert -math.pi - 1e-12 <= wrapped <= math.pi
    assert math.cos(wrapped) == pytest.approx(math.cos(angle))
    assert math.sin(wrapped) == pytest.approx(math.sin(angle), abs=1e-9)


def test_slew_angle_limits_step():
    assert slew_angle(0.0, 1.0, 0.1) == pytest.approx(0.1)
    assert slew_angle(0.0, -1.0, 0.1) == pytest.approx(-0.1)


def test_slew_angle_reaches_target_within_step():
    assert slew_angle(0.0, 0.05, 0.1) == pytest.approx(0.05)


def test_slew_angle_zero_step_keeps_current():
    assert slew_angle(0.3, 1.0, 0.0) == pytest.approx(0.3)


def test_slew_angle_takes_shortest_path_across_pi():
    result = slew_angle(3.0, -3.0, 0.1)
    assert angle_difference(result, 3.1) < 1e-9
    result = slew_angle(-3.0, 3.0, 0.1)
    assert angle_difference(result, -3.1) < 1e-9


def test_slew_angle_reaches_target_across_pi():
    result = slew_angle(3.1, -3.1, 0.2)
    assert angle_difference(result, -3.1) < 1e-9


def test_slew_angle_output_is_wrapped():
    result = slew_angle(3.1, -3.1, 0.2)
    assert -math.pi <= result <= math.pi


@pytest.mark.parametrize("value,expected", [(5, 5), (-5, 0), (15, 10), (0, 0), (10, 10)])
def test_clamp(value, expected):
    assert clamp(value, 0, 10) == expected


@pytest.mark.parametrize("error,width", [(0.0, 6.0), (5.9, 6.0), (-5.9, 6.0), (11.9, 12.0), (-11.9, 12.0)])
def test_deadband_inside_returns_zero(error, width):
    assert deadband(error, width) == 0.0


def test_deadband_at_boundary_is_zero():
    assert deadband(6.0, 6.0) == 0.0
    assert deadband(-6.0, 6.0) == 0.0


def test_deadband_outside_subtracts_width_symmetrically():
    assert deadband(20.0, 12.0) == pytest.approx(8.0)
    assert deadband(-20.0, 12.0) == pytest.approx(-8.0)


def test_deadband_zero_width_is_identity():
    assert deadband(3.5, 0.0) == 3.5
    assert deadband(-3.5, 0.0) == -3.5


def test_deadband_constants():
    assert PITCH_DEADBAND_PX == 6.0
    assert YAW_DEADBAND_PX == 12.0


def test_pixel_pitch_correction_sign():
    assert pixel_pitch_correction(100.0, 0.8, 466.0, 0.5) < 0.0
    assert pixel_pitch_correction(-100.0, 0.8, 466.0, 0.5) > 0.0


def test_pixel_pitch_correction_zero_inside_deadband():
    assert pixel_pitch_correction(5.0, 0.8, 466.0, 0.5) == 0.0
    assert pixel_pitch_correction(-5.0, 0.8, 466.0, 0.5) == 0.0


def test_pixel_pitch_correction_value():
    expected = -0.8 * (20.0 / 466.0) * 0.5
    assert pixel_pitch_correction(26.0, 0.8, 466.0, 0.5) == pytest.approx(expected)


def test_pixel_pitch_correction_is_odd_function():
    up = pixel_pitch_correction(-40.0, 0.8, 466.0, 0.5)
    down = pixel_pitch_correction(40.0, 0.8, 466.0, 0.5)
    assert up == pytest.approx(-down)


def test_pixel_pitch_correction_scales_with_gain_and_k():
    base = pixel_pitch_correction(50.0, 0.8, 466.0, 0.5)
    assert pixel_pitch_correction(50.0, 1.6, 466.0, 0.5) == pytest.approx(2 * base)
    assert pixel_pitch_correction(50.0, 0.8, 466.0, 1.0) == pytest.approx(2 * base)
    assert pixel_pitch_correction(50.0, 0.8, 932.0, 0.5) == pytest.approx(0.5 * base)


def test_pixel_pitch_correction_zero_gain():
    assert pixel_pitch_correction(100.0, 0.8, 466.0, 0.0) == 0.0


def test_feedforward_positive_for_counter_clockwise_motion():
    assert tangential_yaw_feedforward(2.0, 0.0, 0.0, 0.2) == pytest.approx(0.1)


def test_feedforward_negative_for_clockwise_motion():
    assert tangential_yaw_feedforward(2.0, 0.0, 0.0, -0.2) == pytest.approx(-0.1)


def test_feedforward_zero_for_radial_motion():
    assert tangential_yaw_feedforward(2.0, 0.0, 1.0, 0.0) == pytest.approx(0.0)


def test_feedforward_is_clamped():
    assert tangential_yaw_feedforward(2.0, 0.0, 0.0, 5.0) == YAW_FF_LIMIT
    assert tangential_yaw_feedforward(2.0, 0.0, 0.0, -5.0) == -YAW_FF_LIMIT


def test_feedforward_zero_distance_returns_zero():
    assert tangential_yaw_feedforward(0.0, 0.0, 1.0, 1.0) == 0.0


def test_feedforward_decreases_with_distance():
    near = tangential_yaw_feedforward(2.0, 0.0, 0.0, 0.2)
    far = tangential_yaw_feedforward(4.0, 0.0, 0.0, 0.2)
    assert far == pytest.approx(near / 2)


@pytest.mark.parametrize("angle", [0.3, 1.0, 2.5, -1.7])
def test_feedforward_is_rotation_invariant(angle):
    dx, dy, vx, vy = 3.0, 1.0, 0.1, 0.2
    c, s = math.cos(angle), math.sin(angle)
    rotated = tangential_yaw_feedforward(c * dx - s * dy, s * dx + c * dy, c * vx - s * vy, s * vx + c * vy)
    assert rotated == pytest.approx(tangential_yaw_feedforward(dx, dy, vx, vy))