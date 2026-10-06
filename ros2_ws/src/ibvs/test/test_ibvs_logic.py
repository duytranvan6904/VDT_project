import math
from types import SimpleNamespace

import pytest

from ibvs_logic import (
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


def quat_from_yaw(yaw):
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class TestQuaternionToYaw:
    @pytest.mark.parametrize("yaw", [0.0, 0.5, -0.5, math.pi / 2, -math.pi / 2, 3.0, -3.0])
    def test_round_trip(self, yaw):
        assert quaternion_to_yaw(quat_from_yaw(yaw)) == pytest.approx(yaw, abs=1e-9)

    def test_identity(self):
        q = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        assert quaternion_to_yaw(q) == 0.0


class TestWrapAngle:
    @pytest.mark.parametrize(
        "angle, expected",
        [(0.0, 0.0), (math.pi / 2, math.pi / 2), (3 * math.pi / 2, -math.pi / 2),
         (-3 * math.pi / 2, math.pi / 2), (2 * math.pi, 0.0), (-2 * math.pi, 0.0)],
    )
    def test_values(self, angle, expected):
        assert wrap_angle(angle) == pytest.approx(expected, abs=1e-9)

    def test_range(self):
        for k in range(-40, 41):
            a = wrap_angle(k * 0.7)
            assert -math.pi <= a < math.pi


class TestSlewAngle:
    def test_within_step_reaches_target(self):
        assert slew_angle(0.0, 0.1, 0.5) == pytest.approx(0.1)

    def test_limited_by_step(self):
        assert slew_angle(0.0, 1.0, 0.2) == pytest.approx(0.2)
        assert slew_angle(0.0, -1.0, 0.2) == pytest.approx(-0.2)

    def test_shortest_path_across_wrap(self):
        result = slew_angle(math.radians(170), math.radians(-170), math.radians(5))
        assert result == pytest.approx(math.radians(175))

    def test_shortest_path_negative_side(self):
        result = slew_angle(math.radians(-170), math.radians(170), math.radians(5))
        assert result == pytest.approx(math.radians(-175))

    def test_zero_step_holds(self):
        assert slew_angle(0.3, 2.0, 0.0) == pytest.approx(0.3)


class TestDeadband:
    def test_inside_is_zero(self):
        assert deadband(5.9, PITCH_DEADBAND_PX) == 0.0
        assert deadband(-11.9, YAW_DEADBAND_PX) == 0.0

    def test_outside_shrinks_toward_zero(self):
        assert deadband(10.0, 6.0) == pytest.approx(4.0)
        assert deadband(-10.0, 6.0) == pytest.approx(-4.0)

    def test_boundary(self):
        assert deadband(6.0, 6.0) == pytest.approx(0.0)
        assert deadband(-12.0, 12.0) == pytest.approx(0.0)

    def test_continuous_and_odd(self):
        for e in (0.0, 3.0, 7.5, 30.0):
            assert deadband(-e, 6.0) == pytest.approx(-deadband(e, 6.0))


class TestClamp:
    def test_inside(self):
        assert clamp(0.5, -1.0, 1.0) == 0.5

    def test_below_and_above(self):
        assert clamp(-3.0, -1.0, 1.0) == -1.0
        assert clamp(3.0, -1.0, 1.0) == 1.0


class TestPixelPitchCorrection:
    def test_inside_deadband_is_zero(self):
        assert pixel_pitch_correction(5.0, 0.8, 466.0, 0.5) == pytest.approx(0.0)

    def test_marker_below_center_tilts_down(self):
        assert pixel_pitch_correction(50.0, 0.8, 466.0, 0.5) < 0.0

    def test_marker_above_center_tilts_up(self):
        assert pixel_pitch_correction(-50.0, 0.8, 466.0, 0.5) > 0.0

    def test_value(self):
        expected = -0.8 * (44.0 / 466.0) * 0.5
        assert pixel_pitch_correction(50.0, 0.8, 466.0, 0.5) == pytest.approx(expected)

    def test_scales_with_gain(self):
        a = pixel_pitch_correction(50.0, 0.8, 466.0, 0.5)
        b = pixel_pitch_correction(50.0, 0.8, 466.0, 1.0)
        assert b == pytest.approx(2.0 * a)


class TestTangentialYawFeedforward:
    def test_target_moving_radially_gives_zero(self):
        assert tangential_yaw_feedforward(5.0, 0.0, 1.0, 0.0) == pytest.approx(0.0)

    def test_target_moving_counterclockwise_is_positive(self):
        assert tangential_yaw_feedforward(5.0, 0.0, 0.0, 0.5) == pytest.approx(0.1)

    def test_target_moving_clockwise_is_negative(self):
        assert tangential_yaw_feedforward(5.0, 0.0, 0.0, -0.5) == pytest.approx(-0.1)

    def test_clamped(self):
        assert tangential_yaw_feedforward(2.0, 0.0, 0.0, 5.0) == pytest.approx(YAW_FF_LIMIT)
        assert tangential_yaw_feedforward(2.0, 0.0, 0.0, -5.0) == pytest.approx(-YAW_FF_LIMIT)

    def test_zero_distance_is_safe(self):
        assert tangential_yaw_feedforward(0.0, 0.0, 1.0, 1.0) == 0.0

    def test_rotation_invariant(self):
        a = tangential_yaw_feedforward(5.0, 0.0, 0.0, 0.5)
        b = tangential_yaw_feedforward(0.0, 5.0, -0.5, 0.0)
        assert a == pytest.approx(b)