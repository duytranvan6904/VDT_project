"""Regression tests for FOLLOW stand-off and yaw command stabilization."""

import math
import unittest

import numpy as np

from simulation.control.apf_planner import (
    APFCore,
    APFParams,
    limit_velocity_for_cylinder_clearance,
    make_follow_goal,
)
from simulation.control.ibvs_controller import _slew_angle, _wrap_angle
from simulation.control.tracking_control import (
    compose_follow_velocity,
    follow_motion_yaw,
    forward_heading_guard,
    slew_yaw_reference,
)


class TestFollowControl(unittest.TestCase):
    def test_yaw_alignment_scaling_preserves_pad_velocity_feedforward(self):
        command = compose_follow_velocity(
            guidance_xy=np.array([0.6, 0.0]),
            target_velocity_ff_xy=np.array([0.5, 0.0]),
            alignment_scale=0.25,
            max_speed=1.5,
        )

        # The guidance is slowed for image alignment, but the drone keeps pace
        # with the moving pad instead of falling below its 0.5 m/s speed.
        np.testing.assert_allclose(command, [0.65, 0.0])

    def test_follow_goal_keeps_standoff_and_drone_altitude(self):
        drone = np.array([0.0, 0.0, 3.0])
        target = np.array([5.0, 2.0, 0.02])

        goal = make_follow_goal(drone, target, 3.5)

        self.assertAlmostEqual(np.linalg.norm(goal - target), 3.5)
        self.assertAlmostEqual(
            np.linalg.norm(goal[:2] - target[:2]),
            math.sqrt(3.5**2 - (drone[2] - target[2])**2),
        )
        self.assertAlmostEqual(goal[2], drone[2])

        result = APFCore(APFParams(v_max=1.5, k_att=10.0)).compute(
            drone, goal, [],
        )
        self.assertAlmostEqual(result.velocity[2], 0.0)

    def test_yaw_slew_uses_shortest_path_across_pi(self):
        result = _slew_angle(math.radians(179.0), math.radians(-179.0), math.radians(10.0))

        self.assertAlmostEqual(result, math.radians(-179.0), places=6)
        self.assertLess(abs(_wrap_angle(result - math.radians(179.0))), math.radians(10.0))

    def test_follow_goal_decouples_standoff_from_altitude_fluctuations(self):
        target = np.array([5.0, 2.0, 0.0])
        drone_low = np.array([0.0, 0.0, 2.90])
        drone_high = np.array([0.0, 0.0, 3.10])

        goal_low = make_follow_goal(drone_low, target, 3.5, hold_altitude=True, cruise_altitude=3.0)
        goal_high = make_follow_goal(drone_high, target, 3.5, hold_altitude=True, cruise_altitude=3.0)

        standoff_low = float(np.linalg.norm(goal_low[:2] - target[:2]))
        standoff_high = float(np.linalg.norm(goal_high[:2] - target[:2]))

        # Horizontal standoff must remain identical despite altitude variations
        self.assertAlmostEqual(standoff_low, standoff_high, places=5)
        self.assertAlmostEqual(standoff_low, math.sqrt(3.5**2 - 3.0**2), places=5)
        self.assertAlmostEqual(goal_low[2], 3.0)
        self.assertAlmostEqual(goal_high[2], 3.0)

    def test_follow_goal_supports_configured_horizontal_standoff(self):
        drone = np.array([0.0, 0.0, 3.0])
        target = np.array([5.0, 0.0, 0.02])
        goal = make_follow_goal(
            drone, target, 2.2, hold_altitude=True, cruise_altitude=3.0,
            is_3d_distance=False,
        )

        self.assertAlmostEqual(np.linalg.norm(goal[:2] - target[:2]), 2.2)
        self.assertAlmostEqual(goal[2], 3.0)

    def test_moving_follow_goal_stays_behind_target_course_after_turn(self):
        drone = np.array([0.0, 0.0, 3.0])
        target = np.array([5.0, 2.0, 0.02])

        goal = make_follow_goal(
            drone,
            target,
            2.2,
            target_velocity_xy=np.array([0.0, 0.5]),
            hold_altitude=True,
            cruise_altitude=3.0,
            is_3d_distance=False,
        )

        np.testing.assert_allclose(goal[:2] - target[:2], [0.0, -2.2])
        self.assertAlmostEqual(goal[2], 3.0)

    def test_follow_yaw_tracks_motion_course_and_guard_blocks_reverse(self):
        yaw = follow_motion_yaw([0.0, 0.5], fallback_yaw=0.0)
        self.assertAlmostEqual(yaw, math.pi / 2.0)

        reverse = forward_heading_guard([-0.5, 0.0], body_yaw=0.0)
        forward = forward_heading_guard([0.5, 0.0], body_yaw=0.0)
        diagonal = forward_heading_guard([0.5, 0.5], body_yaw=0.0)
        perpendicular = forward_heading_guard([0.0, 0.5], body_yaw=0.0)

        np.testing.assert_allclose(reverse, [0.0, 0.0])
        np.testing.assert_allclose(forward, [0.5, 0.0])
        np.testing.assert_allclose(diagonal, [0.5, 0.5])
        np.testing.assert_allclose(perpendicular, [0.0, 0.0])

    def test_course_yaw_reference_limits_noisy_direction_jumps_across_pi(self):
        current = math.radians(179.0)
        target = math.radians(-179.0)

        result = slew_yaw_reference(
            current_yaw=current,
            target_yaw=target,
            max_rate=0.60,
            dt=0.05,
        )

        # The shortest-path update crosses the wrap boundary and changes by
        # at most the configured 1.72 degrees for one 20 Hz control cycle.
        self.assertAlmostEqual(
            result, _wrap_angle(math.radians(179.0) + 0.03), places=6,
        )
        self.assertLess(abs(_wrap_angle(result - current)), math.radians(2.0))

    def test_obstacle_clearance_limits_approach_speed_by_stopping_distance(self):
        pos = np.array([0.0, 0.0, 3.0])
        cmd = np.array([1.0, 0.0, 0.0])
        cylinder = [(1.0, 0.0, 0.5, 4.0, 2.0)]

        safe = limit_velocity_for_cylinder_clearance(pos, cmd, cylinder)

        self.assertGreater(safe[0], 0.0)
        self.assertLess(safe[0], 0.20)
        self.assertAlmostEqual(safe[1], 0.0)

    def test_obstacle_clearance_keeps_outward_or_vertically_separated_motion(self):
        pos = np.array([0.0, 0.0, 3.0])
        outward = np.array([-0.8, 0.2, 0.0])
        low_cylinder = [(0.5, 0.0, 0.3, 1.0, 0.5)]

        safe_outward = limit_velocity_for_cylinder_clearance(
            pos, outward, [(0.5, 0.0, 0.3, 4.0, 2.0)],
        )
        safe_above = limit_velocity_for_cylinder_clearance(pos, outward, low_cylinder)

        np.testing.assert_allclose(safe_outward, outward)
        np.testing.assert_allclose(safe_above, outward)

    def test_obstacle_clearance_brakes_measured_inward_momentum(self):
        pos = np.array([0.0, 0.0, 3.0])
        cmd = np.zeros(3)
        current = np.array([0.8, 0.0, 0.0])
        cylinder = [(1.0, 0.0, 0.5, 4.0, 2.0)]

        safe = limit_velocity_for_cylinder_clearance(
            pos, cmd, cylinder, current_velocity=current,
        )

        self.assertLess(safe[0], 0.0)


if __name__ == '__main__':
    unittest.main()
